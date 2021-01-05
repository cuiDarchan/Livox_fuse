/******************************************************************************
 * @author: cuiDarchan
 *****************************************************************************/

// C++
#include <math.h>
#include <omp.h>
#include <stdio.h>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>

// ROS
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "Eigen/Dense"

// msgs
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include "livox_fuse/Localization.h"
#include "std_msgs/UInt32.h"

// pcl
#include <pcl/common/transforms.h>  //	pcl::transformPointCloud 用到这个头文件
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// yaml
#include "yaml-cpp/yaml.h"
using namespace message_filters;

struct PointXYZIRT {
  PCL_ADD_POINT4D
  uint8_t intensity;
  uint8_t ring;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)(
        uint8_t, ring, ring)(double, timestamp, timestamp))

typedef PointXYZIRT Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::PointCloud<Point>::Ptr PointCloudPtr;

// 文件参数
ros::Publisher pub_livox_compensator_;
ros::Publisher pub_localization_tf_;
ros::Subscriber sub_localization_;
std::vector<PointCloudPtr> livox_pc_vec_;
std::string file_path_;
std::vector<std::string> extrinsic_names_vec_;
std::vector<std::string> topic_names_vec_;
tf2_ros::Buffer tf2_buffer_;

void Init(ros::NodeHandle &nh) {
  nh.getParam("/FileDir/file_path", file_path_);
  ROS_INFO("file_path: %s", file_path_.c_str());
  nh.param("/FileDir/extrinsic_names", extrinsic_names_vec_,
           std::vector<std::string>{""});
  for (auto extrinsic_name : extrinsic_names_vec_) {
    ROS_INFO("extrinsic_name: %s", extrinsic_name.c_str());
  }
  nh.param("/FileDir/topic_names", topic_names_vec_,
           std::vector<std::string>{""});
  for (auto topic_name : topic_names_vec_) {
    ROS_INFO("topic_name: %s", topic_name.c_str());
  }
}

void LocalizationCallback(const livox_fuse::Localization localization) {
  // ROS_INFO("Localizatin_callback!");
  geometry_msgs::TransformStamped tf;
  static tf::TransformBroadcaster
      broadcaster_;  // 广播broadcaster,发布tf转换消息
  tf.header.seq = localization.header.sequence_num;
  tf.header.stamp = (ros::Time)(localization.header.measured_timestamp * 1e-6);
  tf.header.frame_id = "world";
  tf.child_frame_id = "novatel";
  tf.transform.translation.x = localization.position[0];
  tf.transform.translation.y = localization.position[1];
  tf.transform.translation.z = localization.position[2];
  tf.transform.rotation.x = localization.orientation.x;
  tf.transform.rotation.y = localization.orientation.y;
  tf.transform.rotation.z = localization.orientation.z;
  tf.transform.rotation.w = localization.orientation.w;
  broadcaster_.sendTransform(tf);  // 广播tf消息
}

bool LoadExtrinsic(const std::string &file_path, Eigen::Affine3d *extrinsic) {
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

void GetTimestampInterval(const PointCloudPtr &point_cloud,
                          double &timestamp_min, double &timestamp_max) {
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

bool GetNovatelToWorldTrans(const double &timestamp, Eigen::Affine3d &pose,
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

bool MotionCompensation(const PointCloudPtr &point_cloud,
                        const double timestamp_min, const double timestamp_max,
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

bool LivoxCompensator(const PointCloudPtr &point_cloud) {
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

void FuseandCompensatorPointCloud(std::vector<PointCloudPtr> &point_cloud_vec) {
  PointCloudPtr all_point_cloud_ptr(new PointCloud);
  for (auto point_cloud : point_cloud_vec) {
    *all_point_cloud_ptr += *point_cloud;
  }
  LivoxCompensator(all_point_cloud_ptr);
  sensor_msgs::PointCloud2 all_point_cloud_msg;
  pcl::toROSMsg(*all_point_cloud_ptr, all_point_cloud_msg);
  all_point_cloud_msg.header.stamp = ros::Time::now();
  all_point_cloud_msg.header.frame_id = "novatel";
  pub_livox_compensator_.publish(all_point_cloud_msg);
  ROS_INFO("append point cloud: %f s", ros::Time::now().toSec());
  livox_pc_vec_.clear();
}

void callback(const sensor_msgs::PointCloud2::ConstPtr &point_cloud1,
              const sensor_msgs::PointCloud2::ConstPtr &point_cloud2,
              const sensor_msgs::PointCloud2::ConstPtr &point_cloud3,
              const sensor_msgs::PointCloud2::ConstPtr &point_cloud4,
              const sensor_msgs::PointCloud2::ConstPtr &point_cloud5,
              const sensor_msgs::PointCloud2::ConstPtr &point_cloud6,
              const sensor_msgs::PointCloud2::ConstPtr &point_cloud7,
              const sensor_msgs::PointCloud2::ConstPtr &point_cloud8,
              const sensor_msgs::PointCloud2::ConstPtr &point_cloud9) {
  std::vector<PointCloudPtr> point_cloud_vec;
  for (int i = 0; i < 9; i++) {
    PointCloudPtr tmp_point_cloud_ptr(new PointCloud);
    point_cloud_vec.push_back(tmp_point_cloud_ptr);
  }
  pcl::fromROSMsg(*point_cloud1, *point_cloud_vec[0]);
  pcl::fromROSMsg(*point_cloud2, *point_cloud_vec[1]);
  pcl::fromROSMsg(*point_cloud3, *point_cloud_vec[2]);
  pcl::fromROSMsg(*point_cloud4, *point_cloud_vec[3]);
  pcl::fromROSMsg(*point_cloud5, *point_cloud_vec[4]);
  pcl::fromROSMsg(*point_cloud6, *point_cloud_vec[5]);
  pcl::fromROSMsg(*point_cloud7, *point_cloud_vec[6]);
  pcl::fromROSMsg(*point_cloud8, *point_cloud_vec[7]);
  pcl::fromROSMsg(*point_cloud9, *point_cloud_vec[8]);

  double time_base = ros::Time::now().toSec();  // 加载开始
  Eigen::Affine3d extrinsic;
  for (int i = 0; i < 9; i++) {
    if (LoadExtrinsic(file_path_ + extrinsic_names_vec_[i], &extrinsic)) {
      pcl::transformPointCloud(*point_cloud_vec[i], *point_cloud_vec[i],
                               extrinsic);
      livox_pc_vec_.push_back(point_cloud_vec[i]);
    }
  }
  ROS_INFO("extrinsic time: %f s", (ros::Time::now().toSec() - time_base));
  ROS_INFO("livox_pc_vec_size: %d ", livox_pc_vec_.size());
  FuseandCompensatorPointCloud(livox_pc_vec_);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "livox_fuse");
  ros::NodeHandle nh("~");
  Init(nh);
  tf2_ros::TransformListener *tf2_transform_listener_ = nullptr;

  pub_livox_compensator_ = nh.advertise<sensor_msgs::PointCloud2>(
      "/livox_lidar_front/compensator/PointCloud2", 10);

  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_front(
      nh, topic_names_vec_[0], 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_back(
      nh, topic_names_vec_[1], 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_left_back(
      nh, topic_names_vec_[2], 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_left_front(
      nh, topic_names_vec_[3], 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_right_back(
      nh, topic_names_vec_[4], 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_right_front(
      nh, topic_names_vec_[5], 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_up_left(
      nh, topic_names_vec_[6], 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_up_mid(
      nh, topic_names_vec_[7], 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_up_right(
      nh, topic_names_vec_[8], 1);
  typedef sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2, sensor_msgs::PointCloud2,
      sensor_msgs::PointCloud2, sensor_msgs::PointCloud2,
      sensor_msgs::PointCloud2, sensor_msgs::PointCloud2,
      sensor_msgs::PointCloud2, sensor_msgs::PointCloud2,
      sensor_msgs::PointCloud2>
      MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument,
  Synchronizer<MySyncPolicy> sync(
      MySyncPolicy(10), sub_front, sub_back, sub_left_back, sub_left_front,
      sub_right_back, sub_right_front, sub_up_left, sub_up_mid, sub_up_right);
  sync.registerCallback(
      boost::bind(&callback, _1, _2, _3, _4, _5, _6, _7, _8, _9));

  sub_localization_ = nh.subscribe<livox_fuse::Localization>(
      "/localization/localization", 10, &LocalizationCallback);
  tf2_transform_listener_ = new tf2_ros::TransformListener(tf2_buffer_);

  ros::spin();
  return 0;
}