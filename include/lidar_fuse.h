/******************************************************************************
 * @author: cuiDarchan
 *****************************************************************************/

// C++
#include <math.h>
#include <omp.h>
#include <stdio.h>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <initializer_list>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>

// ROS
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "Eigen/Dense"

// msgs
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include "roscpp_tutorials/INSRaw.h"
#include "roscpp_tutorials/Localization.h"
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
using sP = sensor_msgs::PointCloud2;

typedef sync_policies::ApproximateTime<sP, sP> MySyncPolicy2;
typedef sync_policies::ApproximateTime<sP, sP, sP> MySyncPolicy3;
typedef sync_policies::ApproximateTime<sP, sP, sP, sP> MySyncPolicy4;
typedef sync_policies::ApproximateTime<sP, sP, sP, sP, sP> MySyncPolicy5;

using Sync2 = message_filters::Synchronizer<MySyncPolicy2>;
using Sync3 = message_filters::Synchronizer<MySyncPolicy3>;
using Sync4 = message_filters::Synchronizer<MySyncPolicy4>;
using Sync5 = message_filters::Synchronizer<MySyncPolicy5>;

struct LidarConfig {
  std::string lidar_type;
  std::string lidar_topic;
  std::string frame_id;
  std::string extrinsic_name;
};
namespace YAML {
template <>
struct convert<LidarConfig> {
  static Node encode(const LidarConfig &lidar_cfg) {
    Node node;
    node.push_back(lidar_cfg.lidar_type);
    node.push_back(lidar_cfg.frame_id);
    node.push_back(lidar_cfg.lidar_topic);
    node.push_back(lidar_cfg.extrinsic_name);
    return node;
  }
  static bool decode(const Node &node, LidarConfig &lidar_cfg) {
    lidar_cfg.lidar_type = node["lidar_type"].as<std::string>();
    lidar_cfg.frame_id = node["frame_id"].as<std::string>();
    lidar_cfg.lidar_topic = node["lidar_topic"].as<std::string>();
    lidar_cfg.extrinsic_name = node["extrinsic_name"].as<std::string>();
    return true;
  }
};
}

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

class LidarFuse {
 public:
  LidarFuse(const ros::NodeHandle &nh);
  void Init(const ros::NodeHandle &nh);
  void INSCallback(const roscpp_tutorials::INSRaw &ins_raw);
  bool LoadExtrinsic(const std::string &file_path, Eigen::Affine3d *extrinsic);
  void GetTimestampInterval(const PointCloudPtr &point_cloud,
                            double &timestamp_min, double &timestamp_max);
  bool GetNovatelToWorldTrans(const double &timestamp, Eigen::Affine3d &pose,
                              const std::string &child_frame_id);
  bool MotionCompensation(const PointCloudPtr &point_cloud,
                          const double timestamp_min,
                          const double timestamp_max,
                          const Eigen::Affine3d &pose_min_time,
                          const Eigen::Affine3d &pose_max_time);
  bool LivoxCompensator(const PointCloudPtr &point_cloud);
  void FuseandCompensatorPointCloud(std::vector<PointCloudPtr> &point_cloud_vec,
                                    const std::string &lidar_type);
  void ConstructCloudVec(std::vector<PointCloudPtr> &point_cloud_vec,
                         const int pc_size);
  void TransformCloud(const std::string extrinsic_path,
                      const std::vector<LidarConfig> &lidar_config_vec,
                      std::vector<PointCloudPtr> &input_cloud_vec,
                      std::vector<PointCloudPtr> &output_cloud_vec,
                      const int &size);
  void BuildSynchronizer(const std::string &lidar_topic, ros::NodeHandle &nh,
                         const std::vector<LidarConfig> &lidar_config_vec);

  //   template <class T>
  //   void PrintArgs(T t);
  //   template <typename... Args>
  //   void livox_callback(Args... arg);

  // livox_callback
  void livox_callback(const sP::ConstPtr &cloud1);
  void livox_callback2(const sP::ConstPtr &cloud1, const sP::ConstPtr &cloud2);
  void livox_callback3(const sP::ConstPtr &cloud1, const sP::ConstPtr &cloud2,
                       const sP::ConstPtr &cloud3);
  void livox_callback4(const sP::ConstPtr &cloud1, const sP::ConstPtr &cloud2,
                       const sP::ConstPtr &cloud3, const sP::ConstPtr &cloud4);
  void livox_callback5(const sP::ConstPtr &cloud1, const sP::ConstPtr &cloud2,
                       const sP::ConstPtr &cloud3, const sP::ConstPtr &cloud4,
                       const sP::ConstPtr &cloud5);
  // rslidar_callback
  void rslidar_callback(const sP::ConstPtr &cloud1);
  void rslidar_callback2(const sP::ConstPtr &cloud1,
                         const sP::ConstPtr &cloud2);
  void rslidar_callback3(const sP::ConstPtr &cloud1, const sP::ConstPtr &cloud2,
                         const sP::ConstPtr &cloud3);
  void rslidar_callback4(const sP::ConstPtr &cloud1, const sP::ConstPtr &cloud2,
                         const sP::ConstPtr &cloud3,
                         const sP::ConstPtr &cloud4);
  void rslidar_callback5(const sP::ConstPtr &cloud1, const sP::ConstPtr &cloud2,
                         const sP::ConstPtr &cloud3, const sP::ConstPtr &cloud4,
                         const sP::ConstPtr &cloud5);

 private:
  ros::NodeHandle nh_;
  bool is_compensator_ = false;
  // 外参路径
  std::string extrinsic_path_;
  std::string yaml_path_;
  // topic
  std::string rslidar_fuse_topic_;
  std::string livox_fuse_topic_;

  // 文件配置
  std::vector<LidarConfig> rslidar_config_vec_;
  std::vector<LidarConfig> livox_config_vec_;

  // 发布数据
  ros::Publisher pub_livox_compensator_;
  ros::Publisher pub_rslidar_compensator_;
  std::vector<PointCloudPtr> livox_pc_vec_;
  std::vector<PointCloudPtr> rslidar_pc_vec_;

  // sub_ptr_vec
  std::vector<std::shared_ptr<message_filters::Subscriber<sP>>>
      livox_sub_ptr_vec_;
  std::vector<std::shared_ptr<message_filters::Subscriber<sP>>>
      rslidar_sub_ptr_vec_;

  // INSRaw
  ros::Publisher localization_tf_pub_;
  ros::Subscriber localization_sub_;
  tf2_ros::Buffer tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_transform_listener_;

  // Sync_policies group
  std::shared_ptr<Sync2> livox_sync2_ptr_;
  std::shared_ptr<Sync2> rslidar_sync2_ptr_;
  std::shared_ptr<Sync3> livox_sync3_ptr_;
  std::shared_ptr<Sync3> rslidar_sync3_ptr_;
  std::shared_ptr<Sync4> livox_sync4_ptr_;
  std::shared_ptr<Sync4> rslidar_sync4_ptr_;
  std::shared_ptr<Sync5> livox_sync5_ptr_;
  std::shared_ptr<Sync5> rslidar_sync5_ptr_;
};