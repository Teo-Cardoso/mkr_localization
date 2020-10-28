#include <ros/ros.h>
#include <marker_localization/free_markers_localization.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "free_markers_localization");
  ros::NodeHandle node;

  bir::FreeMarkersLocalization freeMarkersLocalization;

  ros::spin();
}
