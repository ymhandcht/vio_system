cmake_minimum_required(VERSION 3.0.2)
project(feature_tracker)

find_package(cv_bridge REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  camera_model
)
find_package(OpenCV 3.2.0 REQUIRED)
catkin_package()
find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  /home/action/vio_system/src/feature_tracker/include/feature_tracker
)
add_executable(feature_tracker 
  src/feature_tracker_node.cpp
  src/parameters.cpp
  src/feature_tracker.cpp)
  target_link_libraries(feature_tracker
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES})