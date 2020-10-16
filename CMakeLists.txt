cmake_minimum_required(VERSION 3.0.2)
project(marker_localization)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  tf2
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport roscpp sensor_msgs std_msgs tf2 message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(libmarker_detect
  src/marker_detect.cpp
)

target_link_libraries(libmarker_detect
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(test_time src/time_test_marker.cpp)

# add_executable(single_marker_identification src/aruco_identification.cpp src/single_marker_identification.cpp src/single_marker_estimator.cpp)
# add_executable(robot_pose_estimator src/aruco_identification.cpp src/single_marker_identification.cpp src/full_marker_estimator.cpp src/landmark_estimator.cpp)

target_link_libraries(test_time
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
	libmarker_detect
)

# target_link_libraries(single_marker_identification
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
# )

# target_link_libraries(robot_pose_estimator
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
# )

if(CATKIN_ENABLE_TESTING)
	add_definitions(-std=c++11)
	
	find_package(rostest REQUIRED)
	catkin_add_gtest(marker_detect_test test/marker_detect_test.cpp src/marker_detect.cpp)
	
	if(TARGET marker_detect_test)
  	target_link_libraries(marker_detect_test libmarker_detect ${catkin_LIBRARIES} ${OpenCV_LIBS})
	endif()
        
endif()
