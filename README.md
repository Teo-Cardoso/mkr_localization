# Marker Localization

Marker Localization has the objective of providing a robot position estimate Aruco-based using the robot camera.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

- ##### ROS
- ##### OpenCV 3.3.1

### Installing

A step by step series of examples that tell you how to get a development env running

#### Download the Package
```
~/catkin_ws/src$ git clone -b develop https://github.com/Brazilian-Institute-of-Robotics/msub_marker_localization.git
```

#### Download the Package ROS Dependencies

```
~/catkin_ws$ rosdep install --from-paths src --ignore-src -r -y
```

#### Build the package
```
~/catkin_ws$ catkin build marker_localization
```
> If any error occurs here, build it again.

## How it works

### Set Configuration File
Inside config folder there is a default config file. It's possible create how many as needed.

Important: The variable set using a vector must always have a floating point.
> Example: params vector: [1.0, 2.0, 3.0]

Look up that 1.0 represents an integer 1.

#### Configuration Words:

| Configuration Name:       | Type             | Example Value                             |
|---------------------------|------------------|-------------------------------------------|
| start_aruco_pose_estimate | bool             | true                                      |
| debug                     | bool             | true                                      |
| pose_topic_name           | string           | zed_right_pose                            |
| enable_tf                 | bool             | true                                      |
| image_topic               | string           | /camera/right/image_raw                   |
| marker_publish_topic_name | string           | /found_markers_right                      |
| camera_transform_name     | string           | zed_right_tf                              |
| camera_reference_tf       | string           | world                                     |
| camera_matrix             | vector of double | [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0] |
| camera_distortion         | vector of double | [k1, k2, k3, k4, k5]                      |
| marker_length             | vector of double | [10.0, 15.0]                              |
| markers_                  | vector of double | [0.0, 1.0, 2.0, 3.0]                      |
| marker_name               | string           | aruco_6x6_                                |
| aruco_transform_namespace | string           | panelTF/                                  |
| aruco_reference_tf        | string           | /tag                                      |
| publish_frequecy          | int              | 60                                        |


![image](flow.png)
