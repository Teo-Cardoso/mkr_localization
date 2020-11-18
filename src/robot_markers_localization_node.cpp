#include <ros/ros.h>
#include <marker_localization/robot_markers_localization.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "robot_markers_localization");
  ros::NodeHandle node;

  bir::RobotMarkersLocalization robotMarkersLocalization;

  ros::spin();
}
