#include <ros/ros.h>
#include <marker_localization/single_marker_identification.hpp>

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "single_marker_identification", ros::init_options::AnonymousName);
    ros::NodeHandle privateNode("~");
    tf::TransformBroadcaster broadcaster;
    bir::SingleMarkerIdentifier singleMarkerIdentifier(privateNode);

    while(ros::ok()) {
        ros::spinOnce();
        
        if(singleMarkerIdentifier.update_is_available()) {
            // uint64_t initTime = ros::Time::now().toNSec();
            bir::MarkerVector detectMarkers = singleMarkerIdentifier.detectMarkers();

            if(!detectMarkers.empty()) {
                bir::MarkerPose markersPose;
                singleMarkerIdentifier.estimatePose(detectMarkers, markersPose);
                const ros::Time NOW = ros::Time::now();
                for (int index = 0; index < markersPose.size(); index++)
                    broadcaster.sendTransform(tf::StampedTransform(markersPose.poses.at(index), NOW, 
                                      "/camera_rgb_optical_frame", "id_" + std::to_string(markersPose.ids.at(index))));
            }
            // uint64_t finalTime = ros::Time::now().toNSec();
            // ROS_WARN_STREAM("Duration: " << std::to_string((double)((finalTime-initTime))*1e-6) << "ms.");
        }

        ROS_INFO_ONCE("\nLocalization is working properly! Did you found your robot? Good Luck!");
    }

}