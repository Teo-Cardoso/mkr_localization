cmake_minimum_required(VERSION 3.0.2)
project(marker_localization)

add_compile_options(-std=c++11)

##### Find Packages and Dependencies #####
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  roslint
  message_generation
)

find_package(OpenCV REQUIRED)

add_message_files( FILES
  MarkerPose.msg
  MarkerPoseArray.msg
)

generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
)

roslint_cpp()

catkin_package(
  INCLUDE_DIRS include test/resource
  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport roscpp sensor_msgs std_msgs tf2 message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(libmarkers_type
  src/marker.cpp
  src/marker_transform.cpp
)

add_library(libmarker
  src/marker_detect.cpp
  src/marker_pose_estimator.cpp
)

add_library(libfree_marker
  src/free_markers_localization.cpp
)

target_link_libraries(libmarkers_type
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(libmarker
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  libmarkers_type
)

target_link_libraries(libfree_marker
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  libmarker
)

add_executable(free_markers_localization src/free_markers_localization_node.cpp)

# add_executable(single_marker_identification src/aruco_identification.cpp src/single_marker_identification.cpp src/single_marker_estimator.cpp)
# add_executable(robot_pose_estimator src/aruco_identification.cpp src/single_marker_identification.cpp src/full_marker_estimator.cpp src/landmark_estimator.cpp)

target_link_libraries(free_markers_localization
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
	libmarker
  libfree_marker
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

	catkin_add_gtest(marker_detect_test
    test/marker_detect_test.cpp
  )

  catkin_add_gtest(marker_vector_test
    test/marker_vector_test.cpp
  )

  catkin_add_gtest(marker_transform_vector_test
    test/marker_transform_vector_test.cpp
  )
  
  add_rostest_gtest(free_markers_localization_test
    test/free_markers_localization.test
    test/free_markers_localization_test.cpp
  )

	if(TARGET marker_detect_test)
  	target_link_libraries(marker_detect_test libmarker ${catkin_LIBRARIES} ${OpenCV_LIBS})
	  target_compile_definitions(marker_detect_test PUBLIC TEST_RESOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/resource/")
	endif()

  if(TARGET marker_vector_test)
  	target_link_libraries(marker_vector_test libmarker ${catkin_LIBRARIES} ${OpenCV_LIBS})
	endif()

  if(TARGET marker_transform_vector_test)
  	target_link_libraries(marker_transform_vector_test libmarker ${catkin_LIBRARIES} ${OpenCV_LIBS})
	endif()

  target_link_libraries(free_markers_localization_test
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    libmarker
    libfree_marker
  )
        
endif()
