
#include "lidar_fuse.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "livox_fuse");
  ros::NodeHandle nh;
  LidarFuse lidar_fuse(nh);

  ros::spin();
  return 0;
}