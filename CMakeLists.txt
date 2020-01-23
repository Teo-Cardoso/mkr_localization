cmake_minimum_required(VERSION 2.8.3)
project(bir_marker_localization)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2
)

find_package(OpenCV 3.3.1 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES marker_localization
  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport roscpp rospy sensor_msgs std_msgs tf tf2 message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(single_marker_identification src/aruco_identification.cpp src/single_marker_identification.cpp src/single_marker_estimator.cpp)
add_executable(landmarks_pose_estimator src/aruco_identification.cpp src/single_marker_identification.cpp src/full_marker_estimator.cpp src/landmark_estimator.cpp)


target_link_libraries(single_marker_identification
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(landmarks_pose_estimator
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)