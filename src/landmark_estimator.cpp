#include <ros/ros.h>
#include <marker_localization/full_marker_estimator.hpp>
#include <tf/transform_broadcaster.h>

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "full_marker_estimator", ros::init_options::AnonymousName);
    ros::NodeHandle privateNode("~");
    tf::TransformBroadcaster broadcaster;
    bir::SingleMarkerIdentifier singleMarkerIdentifier(privateNode);
    bir::FullMarkerEstimator fullMarkerEstimator(privateNode);

    while(ros::ok()) {
        ros::spinOnce();
        
        if(singleMarkerIdentifier.update_is_available()) {
            bir::MarkerVector detectMarkers = singleMarkerIdentifier.detectMarkers();

            if(!detectMarkers.empty()) {
                // uint64_t initTime = ros::Time::now().toNSec();
                bir::MarkerPose markersPose;
                tf::Transform baseTransform;
                singleMarkerIdentifier.estimatePose(detectMarkers, markersPose);
                fullMarkerEstimator.estimatePose(markersPose, baseTransform);
                fullMarkerEstimator.publish(baseTransform);
                if(fullMarkerEstimator.getEnablePublishTF()) {
                    tf::StampedTransform transform;
                    if(!fullMarkerEstimator.getRobotAsParent()) {
                        transform = tf::StampedTransform(baseTransform, ros::Time::now(), 
                                            fullMarkerEstimator.getBaseTFName(), fullMarkerEstimator.getMapTFName());
                    } else {
                        transform = tf::StampedTransform(baseTransform, ros::Time::now(), 
                                            fullMarkerEstimator.getMapTFName(), fullMarkerEstimator.getBaseTFName());
                    }
                    broadcaster.sendTransform(transform);
                }
                // uint64_t finalTime = ros::Time::now().toNSec();
                
                // ROS_WARN_STREAM("Duration: " << std::to_string((double)((finalTime-initTime))*1e-6) << "ms.");
                ROS_INFO_ONCE("\nLocalization is working properly! Did you found your robot? Good Luck!");
            }
        }
    }
}