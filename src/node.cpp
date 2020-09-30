/******************************************************************************
 * Copyright 2019 The CIDI Authors. All Rights Reserved.
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
#include "Eigen/Dense"

// msgs
#include <sensor_msgs/PointCloud2.h>
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
std::vector<PointCloudPtr> livox_pc_vec_;
std::string file_path_;
std::vector<std::string> extrinsic_names_vec_;
std::vector<std::string> topic_names_vec_;

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

void AppendPointCloud(std::vector<PointCloudPtr> &point_cloud_vec) {
  PointCloudPtr all_point_cloud_ptr(new PointCloud);
  for (auto point_cloud : point_cloud_vec) {
    *all_point_cloud_ptr += *point_cloud;
  }
  sensor_msgs::PointCloud2 all_point_cloud_msg;
  pcl::toROSMsg(*all_point_cloud_ptr, all_point_cloud_msg);
  all_point_cloud_msg.header.stamp = ros::Time::now();  //
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
  AppendPointCloud(livox_pc_vec_);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "livox_fuse");
  ros::NodeHandle nh("~");
  Init(nh);

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

  ros::spin();
  return 0;
}