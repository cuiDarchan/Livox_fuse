cmake_minimum_required(VERSION 2.8.3)
project(livox_fuse)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_definitions(-std=c++11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -fopenmp")

find_package(yaml-cpp REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  message_filters
  geometry_msgs
  tf2_ros
  tf
  message_generation
)

add_message_files(
	FILES
	MeasureState.msg
	LocalizationStatus.msg
	Uncertainty.msg
	ModuleStatus.msg
	ModuleName.msg
	MsgHeader.msg
	Quaterniond.msg
	Quaternionf.msg
	Localization.msg
	IMURaw.msg
	INSRaw.msg
	INSStdevRaw.msg
	MobileyeObstaclesGo.msg
	MobileyeObstacle.msg
	MobileyeObstacles.msg
	RadarRaw.msg
	RadarRawObject.msg
	Point3d.msg
	PerceptionObstacle.msg
	PerceptionObstacles.msg
	SimpleLatDebug.msg
	SimpleLonDebug.msg
	Debug.msg
	EpbCmd.msg
	PassageTurn.msg
	WayPoint.msg
	Control.msg	
	HMIInput.msg
    Lanemarkers.msg
	Lanemarker.msg
	LaneParam.msg
	MobileyeLanemarkers.msg
	MobileyeLanemarker.msg
	VideoEncodedData.msg
	PathPoint.msg
	TrajectoryPoint.msg
	Trajectory.msg
	PredictionObstacle.msg
	PredictionObstacles.msg
	CameraDetection.msg
	Detection.msg
	UltrasonicRaws.msg
	UltrasonicRawObject.msg
	ChassisInfo.msg
	Odometry.msg
	Curved.msg
	CurveSign.msg
	JunctionROIPolygon.msg
	LaneBoundaryType.msg
	LaneSampleAssociation.msg
	NavigationLane.msg
	NavigationLaneBoundary.msg
	NavigationMap.msg
	Passage.msg
	PassageSpeed.msg
	PassageType.msg
	PolygonSign.msg
	RoadROIBoundary.msg
	ROIRegion.msg
	Signal.msg
	Vec3d.msg
	BestGnssPos.msg
	Heading.msg
	Transform.msg
	ControlLoopToken.msg
	EncodePointCloud.msg
	GearCmd.msg
	LightCmd.msg
    PadMessage.msg
	ParkingSpace.msg
	VehicleSignalCmd.msg
	WiperCmd.msg
	ADCTrajectory.msg	
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp std_msgs 
   CATKIN_DEPENDS message_runtime
   DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_node src/node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node 
   ${catkin_LIBRARIES}
   yaml-cpp
   ${PCL_LIBRARIES}
   ${EIGEN3_LIBRARIES}
)

