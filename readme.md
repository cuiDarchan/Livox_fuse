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
cfg/extrinsics: 包含激光的yaml外参文件 
FileDir.yaml:  
    is_compensator: 融合数据前，是否开启运动补偿  
    pub_topic_names: 发布的融合后的两类激光topic  
    LidarInfos: 激光配置的数组，可自由组合（暂时支持两类激光各最多5路）
                每一类激光包括：类型，原始topic名，frame_id，外参文件名  

### 2.2  **具体步骤**
```
cd catkin_ws
catkin_make -DCATKIN_WHITELIST_PACKAGES="livox_fuse"
roslaunch livox_fuse livox_fuse.launch
rosbag play *.bag
``` 
## 3. 样例数据
样例数据保存在data目录下，
    test1：包含9路大疆topic和定位数据;
    test2：包含4路速腾和3路大疆激光 
```
tar -xvf test.bag.tar.gz  
```   