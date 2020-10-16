# 激光点云数据融合工具

## 1. 软件环境 
### 1.1  **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04 ，ROS Kinetic 
### 1.2  **yaml-cpp**
Follow [Yaml-cpp Installation] (http://wiki.ros.org/yaml_cpp)
### 1.3  **Eigen**
Recommend version [3.3.7](http://eigen.tuxfamily.org/index.php?title=Main_Page).  
### 1.4  **PCL**
Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).

## 2. 使用方法
### 2.1  **配置文件**
cfg/extrinsics: 包含激光的yaml外参文件，这里包含cat和zhdl两种车型  
FileDir.yaml:  
    file_path: 外参存放绝对路径，注意包含cat/zhdl  
    extrinsic_names: 激光外参的文件名  
    topic_names: 激光数据的topic名字，注意与上面的extrinsic_names对应.  
### 2.2  **具体步骤**
```
cd catkin_ws
catkin_make -DCATKIN_WHITELIST_PACKAGES="livox_fuse"
roslaunch livox_fuse livox_fuse.launch
rosbag play *.bag
``` 