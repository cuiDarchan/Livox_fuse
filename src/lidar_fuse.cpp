#include "lidar_fuse.h"

LidarFuse::LidarFuse(const ros::NodeHandle &nh) : nh_(nh) {
  Init(nh_);
  pub_livox_compensator_ =
      nh_.advertise<sensor_msgs::PointCloud2>(livox_fuse_topic_, 10);
  pub_rslidar_compensator_ =
      nh_.advertise<sensor_msgs::PointCloud2>(rslidar_fuse_topic_, 10);
  localization_sub_ =
      nh_.subscribe("/INSRaw", 100, &LidarFuse::INSCallback, this);
  tf2_transform_listener_ =
      std::make_shared<tf2_ros::TransformListener>(tf2_buffer_);

  BuildSynchronizer(livox_fuse_topic_, nh_, livox_config_vec_);
  BuildSynchronizer(rslidar_fuse_topic_, nh_, rslidar_config_vec_);
}

void LidarFuse::Init(const ros::NodeHandle &nh) {
  std::string package_path = ros::package::getPath("livox_fuse");
  ROS_INFO("livox_fuse package_path: %s", package_path.c_str());
  extrinsic_path_ = package_path + "/cfg/extrinsics/";
  yaml_path_ = package_path + "/cfg/FileDir.yaml";
  ROS_INFO("extrinsic_path: %s", extrinsic_path_.c_str());
  ROS_INFO("yaml_path: %s", yaml_path_.c_str());
  nh.getParam("/FileDir/is_compensator", is_compensator_);

  nh.param("/FileDir/pub_topic_names/rslidar", rslidar_fuse_topic_,
           std::string{""});
  ROS_INFO("Rslidar pub_topic_name: %s", rslidar_fuse_topic_.c_str());
  nh.param("/FileDir/pub_topic_names/livox", livox_fuse_topic_,
           std::string{""});
  ROS_INFO("Livox pub_topic_name: %s", livox_fuse_topic_.c_str());

  YAML::Node config = YAML::LoadFile(yaml_path_);
  int lidar_size = config["LidarInfos"].size();
  ROS_INFO(" Lidarinfo size:  %d", lidar_size);

  for (int i = 0; i < lidar_size; ++i) {
    auto lidar_info = config["LidarInfos"][i].as<LidarConfig>();
    if (lidar_info.lidar_type == "rslidar") {
      rslidar_config_vec_.push_back(lidar_info);
      std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>>
          rslidar_msg_filter(
              new message_filters::Subscriber<sensor_msgs::PointCloud2>(
                  nh_, lidar_info.lidar_topic, 3));
      rslidar_sub_ptr_vec_.push_back(rslidar_msg_filter);
    } else if (lidar_info.lidar_type == "livox") {
      livox_config_vec_.push_back(lidar_info);
      std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>>
          livox_msg_filter(
              new message_filters::Subscriber<sensor_msgs::PointCloud2>(
                  nh_, lidar_info.lidar_topic, 3));
      livox_sub_ptr_vec_.push_back(livox_msg_filter);
    } else {
      ROS_ERROR_STREAM("There is no " << lidar_info.lidar_type
                                      << " lidar_type!");
    }
  }
}

void LidarFuse::INSCallback(const roscpp_tutorials::INSRaw &ins_raw) {
  geometry_msgs::TransformStamped tf;
  static tf::TransformBroadcaster
      broadcaster;  // 广播broadcaster,发布tf转换消息
  tf.header.seq = ins_raw.header.sequence_num;
  tf.header.stamp = (ros::Time)(ins_raw.header.measured_timestamp * 1e-6);
  tf.header.frame_id = "world";
  tf.child_frame_id = "novatel";
  tf.transform.translation.x = ins_raw.position[0];
  tf.transform.translation.y = ins_raw.position[1];
  tf.transform.translation.z = ins_raw.position[2];
  tf.transform.rotation.x = ins_raw.orientation.x;
  tf.transform.rotation.y = ins_raw.orientation.y;
  tf.transform.rotation.z = ins_raw.orientation.z;
  tf.transform.rotation.w = ins_raw.orientation.w;
  //   ROS_INFO("INSRaw_callback!");
  broadcaster.sendTransform(tf);  // 广播tf消息
}

bool LidarFuse::LoadExtrinsic(const std::string &file_path,
                              Eigen::Affine3d *extrinsic) {
  try {
    YAML::Node config = YAML::LoadFile(file_path);
    if (!config["transform"]) {
      return false;
    }
    if (config["transform"]["translation"] && config["transform"]["rotation"]) {
      double tx = config["transform"]["translation"]["x"].as<double>();
      double ty = config["transform"]["translation"]["y"].as<double>();
      double tz = config["transform"]["translation"]["z"].as<double>();

      double qx = config["transform"]["rotation"]["x"].as<double>();
      double qy = config["transform"]["rotation"]["y"].as<double>();
      double qz = config["transform"]["rotation"]["z"].as<double>();
      double qw = config["transform"]["rotation"]["w"].as<double>();
      *extrinsic =
          Eigen::Translation3d(tx, ty, tz) * Eigen::Quaterniond(qw, qx, qy, qz);
    }
  } catch (const YAML::Exception &e) {
    ROS_INFO("TF ERROR!");
    return false;
  }
  return true;
}

void LidarFuse::GetTimestampInterval(const PointCloudPtr &point_cloud,
                                     double &timestamp_min,
                                     double &timestamp_max) {
  timestamp_max = 0.0;
  timestamp_min = std::numeric_limits<double>::max();
  int total = point_cloud->points.size();

  // get min time and max time
  for (int i = 0; i < total; ++i) {
    double timestamp = point_cloud->points[i].timestamp;
    if (timestamp < timestamp_min) {
      timestamp_min = timestamp;
    }
    if (timestamp > timestamp_max) {
      timestamp_max = timestamp;
    }
  }
}

bool LidarFuse::GetNovatelToWorldTrans(const double &timestamp,
                                       Eigen::Affine3d &pose,
                                       const std::string &child_frame_id) {
  ros::Time query_time(timestamp);
  std::string err_string;
  std::string world_frame_id = "world";
  if (!tf2_buffer_.canTransform(world_frame_id, child_frame_id, query_time,
                                ros::Duration(0.01), &err_string)) {
    ROS_WARN_STREAM("Can not find transform. "
                    << std::fixed << timestamp
                    << " .Error info: " << err_string);
    return false;
  }

  geometry_msgs::TransformStamped stamped_transform;

  try {
    stamped_transform =
        tf2_buffer_.lookupTransform(world_frame_id, child_frame_id, query_time);
  } catch (tf2::TransformException &ex) {
    ROS_ERROR_STREAM(ex.what());
    return false;
  }

  pose = Eigen::Translation3d(stamped_transform.transform.translation.x,
                              stamped_transform.transform.translation.y,
                              stamped_transform.transform.translation.z) *
         Eigen::Quaterniond(stamped_transform.transform.rotation.w,
                            stamped_transform.transform.rotation.x,
                            stamped_transform.transform.rotation.y,
                            stamped_transform.transform.rotation.z);
  return true;
}

bool LidarFuse::MotionCompensation(const PointCloudPtr &point_cloud,
                                   const double timestamp_min,
                                   const double timestamp_max,
                                   const Eigen::Affine3d &pose_min_time,
                                   const Eigen::Affine3d &pose_max_time) {
  using std::abs;
  using std::acos;
  using std::sin;

  Eigen::Vector3d translation =
      pose_min_time.translation() - pose_max_time.translation();
  Eigen::Quaterniond q_max(pose_max_time.linear());
  Eigen::Quaterniond q_min(pose_min_time.linear());
  Eigen::Quaterniond q1(q_max.conjugate() * q_min);
  Eigen::Quaterniond q0(Eigen::Quaterniond::Identity());
  q1.normalize();
  translation = q_max.conjugate() * translation;

  double d = q0.dot(q1);
  double abs_d = abs(d);
  double f = 1.0 / static_cast<double>(timestamp_max - timestamp_min);

  // Threshold for a "significant" rotation from min_time to max_time:
  // The LiDAR range accuracy is ~2 cm. Over 70 meters range, it means an angle
  // of 0.02 / 70 =
  // 0.0003 rad. So, we consider a rotation "significant" only if the scalar
  // part of quaternion is
  // less than cos(0.0003 / 2) = 1 - 1e-8.
  if (abs_d < 1.0 - 1.0e-8) {
    double theta = acos(abs_d);
    double sin_theta = sin(theta);
    double c1_sign = (d > 0) ? 1 : -1;
    for (auto &point : point_cloud->points) {
      float x_scalar = point.x;
      if (std::isnan(x_scalar)) {
        continue;
      }
      float y_scalar = point.y;
      float z_scalar = point.z;
      Eigen::Vector3d p(x_scalar, y_scalar, z_scalar);

      double tp = point.timestamp;
      double t = (timestamp_max - tp) * f;

      Eigen::Translation3d ti(t * translation);

      double c0 = sin((1 - t) * theta) / sin_theta;
      double c1 = sin(t * theta) / sin_theta * c1_sign;
      Eigen::Quaterniond qi(c0 * q0.coeffs() + c1 * q1.coeffs());

      Eigen::Affine3d trans = ti * qi;
      p = trans * p;
      point.x = p.x();
      point.y = p.y();
      point.z = p.z();
    }
  }
  // Not a "significant" rotation. Do translation only.
  for (auto &point : point_cloud->points) {
    float x_scalar = point.x;
    if (std::isnan(x_scalar)) {
      continue;
    }
    float y_scalar = point.y;
    float z_scalar = point.z;
    Eigen::Vector3d p(x_scalar, y_scalar, z_scalar);

    double tp = point.timestamp;
    double t = (timestamp_max - tp) * f;
    Eigen::Translation3d ti(t * translation);

    p = ti * p;

    point.x = p.x();
    point.y = p.y();
    point.z = p.z();
  }
  return true;
}

bool LidarFuse::LivoxCompensator(const PointCloudPtr &point_cloud) {
  double timestamp_max = 0, timestamp_min = 0;
  GetTimestampInterval(point_cloud, timestamp_min, timestamp_max);
  ROS_INFO_STREAM(std::setiosflags(std::ios::fixed)
                  << std::setprecision(6) << "Lidar compensator time_min = "
                  << timestamp_min << ", time_max = " << timestamp_max);
  Eigen::Affine3d pose_min_time;
  Eigen::Affine3d pose_max_time;
  std::string frame_id = "novatel";
  if (GetNovatelToWorldTrans(timestamp_min, pose_min_time, frame_id) &&
      GetNovatelToWorldTrans(timestamp_max, pose_max_time, frame_id)) {
    ROS_INFO("motion compensation!");
    MotionCompensation(point_cloud, timestamp_min, timestamp_max, pose_min_time,
                       pose_max_time);
  }
  return true;
}

void LidarFuse::FuseandCompensatorPointCloud(
    std::vector<PointCloudPtr> &point_cloud_vec,
    const std::string &lidar_type) {
  PointCloudPtr all_point_cloud_ptr(new PointCloud);
  for (auto point_cloud : point_cloud_vec) {
    *all_point_cloud_ptr += *point_cloud;
  }
  if (is_compensator_) {
    LivoxCompensator(all_point_cloud_ptr);
  }
  sensor_msgs::PointCloud2 all_point_cloud_msg;
  pcl::toROSMsg(*all_point_cloud_ptr, all_point_cloud_msg);
  all_point_cloud_msg.header.stamp = ros::Time::now();
  all_point_cloud_msg.header.frame_id = "novatel";
  if (lidar_type == "livox") {
    pub_livox_compensator_.publish(all_point_cloud_msg);
    ROS_INFO("Livox_fuse publish %d point cloud: %f s",
             all_point_cloud_ptr->points.size(), ros::Time::now().toSec());
    livox_pc_vec_.clear();
  } else if (lidar_type == "rslidar") {
    pub_rslidar_compensator_.publish(all_point_cloud_msg);
    ROS_INFO(" Rslidar_fuse publish %d point cloud: %f s",
             all_point_cloud_ptr->points.size(), ros::Time::now().toSec());
    rslidar_pc_vec_.clear();
  }
}

void LidarFuse::ConstructCloudVec(std::vector<PointCloudPtr> &point_cloud_vec,
                                  const int pc_size) {
  //   ROS_INFO("ConstructCloudVec Start ");
  for (int i = 0; i < pc_size; i++) {
    PointCloudPtr tmp_point_cloud_ptr(new PointCloud);
    point_cloud_vec.push_back(tmp_point_cloud_ptr);
  }
}

void LidarFuse::TransformCloud(const std::string extrinsic_path,
                               const std::vector<LidarConfig> &lidar_config_vec,
                               std::vector<PointCloudPtr> &input_cloud_vec,
                               std::vector<PointCloudPtr> &output_cloud_vec,
                               const int &size) {
  //   ROS_INFO("Transform Cloud Start ");
  double time_base = ros::Time::now().toSec();  // 加载开始
  Eigen::Affine3d extrinsic;
  for (int i = 0; i < size; i++) {
    if (LoadExtrinsic(extrinsic_path + lidar_config_vec[i].extrinsic_name,
                      &extrinsic)) {
      pcl::transformPointCloud(*input_cloud_vec[i], *input_cloud_vec[i],
                               extrinsic);
      output_cloud_vec.push_back(input_cloud_vec[i]);
    }
  }
  //   ROS_INFO("Load extrinsic time: %f s", (ros::Time::now().toSec() -
  //   time_base));
  //   ROS_INFO("output_cloud_vec_size: %d ", output_cloud_vec.size());
  FuseandCompensatorPointCloud(output_cloud_vec,
                               lidar_config_vec[0].lidar_type);
}

void LidarFuse::BuildSynchronizer(
    const std::string &lidar_topic, ros::NodeHandle &nh,
    const std::vector<LidarConfig> &lidar_config_vec) {
  int size = lidar_config_vec.size();

  if (lidar_topic == livox_fuse_topic_) {
    switch (size) {
      case 0:
        return;
      case 1:
        livox_sub_ptr_vec_[0]->registerCallback(&LidarFuse::livox_callback,
                                                this);
        ROS_INFO("livox one msg callback!");
        break;
      case 2:
        livox_sync2_ptr_ = std::make_shared<Sync2>(
            MySyncPolicy2(10), *livox_sub_ptr_vec_[0], *livox_sub_ptr_vec_[1]);
        livox_sync2_ptr_->registerCallback(
            boost::bind(&LidarFuse::livox_callback2, this, _1, _2));
        ROS_INFO("livox two msg callback!");
        break;
      case 3:
        livox_sync3_ptr_ = std::make_shared<Sync3>(
            MySyncPolicy3(10), *livox_sub_ptr_vec_[0], *livox_sub_ptr_vec_[1],
            *livox_sub_ptr_vec_[2]);
        livox_sync3_ptr_->registerCallback(
            boost::bind(&LidarFuse::livox_callback3, this, _1, _2, _3));
        ROS_INFO("livox three msg callback!");
        break;
      case 4:
        livox_sync4_ptr_ = std::make_shared<Sync4>(
            MySyncPolicy4(10), *livox_sub_ptr_vec_[0], *livox_sub_ptr_vec_[1],
            *livox_sub_ptr_vec_[2], *livox_sub_ptr_vec_[3]);
        livox_sync4_ptr_->registerCallback(
            boost::bind(&LidarFuse::livox_callback4, this, _1, _2, _3, _4));
        ROS_INFO("livox four msg callback!");
        break;
      case 5:
        livox_sync5_ptr_ = std::make_shared<Sync5>(
            MySyncPolicy5(10), *livox_sub_ptr_vec_[0], *livox_sub_ptr_vec_[1],
            *livox_sub_ptr_vec_[2], *livox_sub_ptr_vec_[3],
            *livox_sub_ptr_vec_[4]);
        livox_sync5_ptr_->registerCallback(
            boost::bind(&LidarFuse::livox_callback5, this, _1, _2, _3, _4, _5));
        ROS_INFO("livox five msg callback!");
        break;
      default:
        ROS_ERROR_STREAM(" Livox Couldn't support "
                         << size << " msg callback! Please Add msg callback!");
        break;
    }
  }
  // rslidar
  else if (lidar_topic == rslidar_fuse_topic_) {
    switch (size) {
      case 0:
        return;
      case 1:
        rslidar_sub_ptr_vec_[0]->registerCallback(&LidarFuse::rslidar_callback,
                                                  this);
        ROS_INFO("rslidar one msg callback!");
        break;
      case 2:
        rslidar_sync2_ptr_ =
            std::make_shared<Sync2>(MySyncPolicy2(10), *rslidar_sub_ptr_vec_[0],
                                    *rslidar_sub_ptr_vec_[1]);
        rslidar_sync2_ptr_->registerCallback(
            boost::bind(&LidarFuse::rslidar_callback2, this, _1, _2));
        ROS_INFO("rslidar two msg callback!");
        break;
      case 3:
        rslidar_sync3_ptr_ = std::make_shared<Sync3>(
            MySyncPolicy3(10), *rslidar_sub_ptr_vec_[0],
            *rslidar_sub_ptr_vec_[1], *rslidar_sub_ptr_vec_[2]);
        rslidar_sync3_ptr_->registerCallback(
            boost::bind(&LidarFuse::rslidar_callback3, this, _1, _2, _3));
        ROS_INFO("rslidar three msg callback!");
        break;
      case 4:
        rslidar_sync4_ptr_ = std::make_shared<Sync4>(
            MySyncPolicy4(10), *rslidar_sub_ptr_vec_[0],
            *rslidar_sub_ptr_vec_[1], *rslidar_sub_ptr_vec_[2],
            *rslidar_sub_ptr_vec_[3]);
        rslidar_sync4_ptr_->registerCallback(
            boost::bind(&LidarFuse::rslidar_callback4, this, _1, _2, _3, _4));
        ROS_INFO("rslidar four msg callback!");
        break;
      case 5:
        rslidar_sync5_ptr_ = std::make_shared<Sync5>(
            MySyncPolicy5(10), *rslidar_sub_ptr_vec_[0],
            *rslidar_sub_ptr_vec_[1], *rslidar_sub_ptr_vec_[2],
            *rslidar_sub_ptr_vec_[3], *rslidar_sub_ptr_vec_[4]);
        rslidar_sync5_ptr_->registerCallback(boost::bind(
            &LidarFuse::rslidar_callback5, this, _1, _2, _3, _4, _5));
        ROS_INFO("rslidar five msg callback!");
        break;
      default:
        ROS_ERROR_STREAM(" Rslidar Couldn't support "
                         << size << " msg callback! Please Add msg callback!");
        break;
    }

  } else {
    ROS_ERROR("There is no topic: %s", lidar_topic.c_str());
  }
}

// livox_callback group
void LidarFuse::livox_callback(const sP::ConstPtr &cloud1) {
  ROS_INFO(" livox_callback start ");
  constexpr int point_msg_size = 1;
  std::vector<PointCloudPtr> point_cloud_vec;
  ConstructCloudVec(point_cloud_vec, point_msg_size);
  pcl::fromROSMsg(*cloud1, *point_cloud_vec[0]);
  TransformCloud(extrinsic_path_, livox_config_vec_, point_cloud_vec,
                 livox_pc_vec_, point_msg_size);
}

void LidarFuse::livox_callback2(const sP::ConstPtr &cloud1,
                                const sP::ConstPtr &cloud2) {
  ROS_INFO(" livox_callback2 start ");
  constexpr int point_msg_size = 2;
  std::vector<PointCloudPtr> point_cloud_vec;
  ConstructCloudVec(point_cloud_vec, point_msg_size);
  pcl::fromROSMsg(*cloud1, *point_cloud_vec[0]);
  pcl::fromROSMsg(*cloud2, *point_cloud_vec[1]);
  TransformCloud(extrinsic_path_, livox_config_vec_, point_cloud_vec,
                 livox_pc_vec_, point_msg_size);
}

void LidarFuse::livox_callback3(const sP::ConstPtr &cloud1,
                                const sP::ConstPtr &cloud2,
                                const sP::ConstPtr &cloud3) {
  ROS_INFO(" livox_callback3 start ");
  constexpr int point_msg_size = 3;
  std::vector<PointCloudPtr> point_cloud_vec;
  ConstructCloudVec(point_cloud_vec, point_msg_size);
  pcl::fromROSMsg(*cloud1, *point_cloud_vec[0]);
  pcl::fromROSMsg(*cloud2, *point_cloud_vec[1]);
  pcl::fromROSMsg(*cloud3, *point_cloud_vec[2]);
  TransformCloud(extrinsic_path_, livox_config_vec_, point_cloud_vec,
                 livox_pc_vec_, point_msg_size);
}

void LidarFuse::livox_callback4(const sP::ConstPtr &cloud1,
                                const sP::ConstPtr &cloud2,
                                const sP::ConstPtr &cloud3,
                                const sP::ConstPtr &cloud4) {
  ROS_INFO(" livox_callback4 start ");
  constexpr int point_msg_size = 4;
  std::vector<PointCloudPtr> point_cloud_vec;
  ConstructCloudVec(point_cloud_vec, point_msg_size);
  pcl::fromROSMsg(*cloud1, *point_cloud_vec[0]);
  pcl::fromROSMsg(*cloud2, *point_cloud_vec[1]);
  pcl::fromROSMsg(*cloud3, *point_cloud_vec[2]);
  pcl::fromROSMsg(*cloud4, *point_cloud_vec[3]);
  TransformCloud(extrinsic_path_, livox_config_vec_, point_cloud_vec,
                 livox_pc_vec_, point_msg_size);
}

void LidarFuse::livox_callback5(const sP::ConstPtr &cloud1,
                                const sP::ConstPtr &cloud2,
                                const sP::ConstPtr &cloud3,
                                const sP::ConstPtr &cloud4,
                                const sP::ConstPtr &cloud5) {
  ROS_INFO(" livox_callback5 start ");
  constexpr int point_msg_size = 5;
  std::vector<PointCloudPtr> point_cloud_vec;
  ConstructCloudVec(point_cloud_vec, point_msg_size);
  pcl::fromROSMsg(*cloud1, *point_cloud_vec[0]);
  pcl::fromROSMsg(*cloud2, *point_cloud_vec[1]);
  pcl::fromROSMsg(*cloud3, *point_cloud_vec[2]);
  pcl::fromROSMsg(*cloud4, *point_cloud_vec[3]);
  pcl::fromROSMsg(*cloud5, *point_cloud_vec[4]);
  TransformCloud(extrinsic_path_, livox_config_vec_, point_cloud_vec,
                 livox_pc_vec_, point_msg_size);
}

// rslidar_callback group
void LidarFuse::rslidar_callback(const sP::ConstPtr &cloud1) {
  ROS_INFO(" rslidar_callback start ");
  constexpr int point_msg_size = 1;
  std::vector<PointCloudPtr> point_cloud_vec;
  ConstructCloudVec(point_cloud_vec, point_msg_size);
  pcl::fromROSMsg(*cloud1, *point_cloud_vec[0]);
  TransformCloud(extrinsic_path_, rslidar_config_vec_, point_cloud_vec,
                 rslidar_pc_vec_, point_msg_size);
}

void LidarFuse::rslidar_callback2(const sP::ConstPtr &cloud1,
                                  const sP::ConstPtr &cloud2) {
  ROS_INFO(" rslidar_callback2 start ");
  constexpr int point_msg_size = 2;
  std::vector<PointCloudPtr> point_cloud_vec;
  ConstructCloudVec(point_cloud_vec, point_msg_size);
  pcl::fromROSMsg(*cloud1, *point_cloud_vec[0]);
  pcl::fromROSMsg(*cloud2, *point_cloud_vec[1]);
  TransformCloud(extrinsic_path_, rslidar_config_vec_, point_cloud_vec,
                 rslidar_pc_vec_, point_msg_size);
}

void LidarFuse::rslidar_callback3(const sP::ConstPtr &cloud1,
                                  const sP::ConstPtr &cloud2,
                                  const sP::ConstPtr &cloud3) {
  ROS_INFO(" rslidar_callback3 start ");
  constexpr int point_msg_size = 3;
  std::vector<PointCloudPtr> point_cloud_vec;
  ConstructCloudVec(point_cloud_vec, point_msg_size);
  pcl::fromROSMsg(*cloud1, *point_cloud_vec[0]);
  pcl::fromROSMsg(*cloud2, *point_cloud_vec[1]);
  pcl::fromROSMsg(*cloud3, *point_cloud_vec[2]);
  TransformCloud(extrinsic_path_, rslidar_config_vec_, point_cloud_vec,
                 rslidar_pc_vec_, point_msg_size);
}

void LidarFuse::rslidar_callback4(const sP::ConstPtr &cloud1,
                                  const sP::ConstPtr &cloud2,
                                  const sP::ConstPtr &cloud3,
                                  const sP::ConstPtr &cloud4) {
  ROS_INFO(" rslidar_callback4 start ");
  constexpr int point_msg_size = 4;
  std::vector<PointCloudPtr> point_cloud_vec;
  ConstructCloudVec(point_cloud_vec, point_msg_size);
  pcl::fromROSMsg(*cloud1, *point_cloud_vec[0]);
  pcl::fromROSMsg(*cloud2, *point_cloud_vec[1]);
  pcl::fromROSMsg(*cloud3, *point_cloud_vec[2]);
  pcl::fromROSMsg(*cloud4, *point_cloud_vec[3]);
  TransformCloud(extrinsic_path_, rslidar_config_vec_, point_cloud_vec,
                 rslidar_pc_vec_, point_msg_size);
}

void LidarFuse::rslidar_callback5(const sP::ConstPtr &cloud1,
                                  const sP::ConstPtr &cloud2,
                                  const sP::ConstPtr &cloud3,
                                  const sP::ConstPtr &cloud4,
                                  const sP::ConstPtr &cloud5) {
  ROS_INFO(" rslidar_callback5 start ");
  constexpr int point_msg_size = 5;
  std::vector<PointCloudPtr> point_cloud_vec;
  ConstructCloudVec(point_cloud_vec, point_msg_size);
  pcl::fromROSMsg(*cloud1, *point_cloud_vec[0]);
  pcl::fromROSMsg(*cloud2, *point_cloud_vec[1]);
  pcl::fromROSMsg(*cloud3, *point_cloud_vec[2]);
  pcl::fromROSMsg(*cloud4, *point_cloud_vec[3]);
  pcl::fromROSMsg(*cloud5, *point_cloud_vec[4]);
  TransformCloud(extrinsic_path_, rslidar_config_vec_, point_cloud_vec,
                 rslidar_pc_vec_, point_msg_size);
}

// 可变参数模板, 尝试后不可用
// template <class T>
// void LidarFuse::PrintArgs(T t) {
//   std::cout << "PrintArgs point size: " << t->points.size() << std::endl;
//   PointCloudPtr tmp_cloud(new PointCloud);
//   pcl::fromROSMsg(*t, *tmp_cloud);
//   livox_pc_vec_.push_back(tmp_cloud);
// }
// template <typename... Args>
// void LidarFuse::livox_callback(Args... arg) {
//   ROS_INFO(" livox_callback start ");
//   int a[]={(PrintArgs(arg),0)...};
//   std::vector<PointCloudPtr> point_cloud_vec = {PrintArgs(arg)...};
//   int point_msg_size = point_cloud_vec.size();
//   std::cout << "point_cloud_vec size: " << point_msg_size << std::endl;
//   std::vector<PointCloudPtr> output_cloud;
//   TransformCloud(extrinsic_path_, livox_config_vec_, livox_pc_vec_,
//                  output_cloud, point_msg_size);
// }