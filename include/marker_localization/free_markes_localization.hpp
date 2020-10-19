#ifndef MARKER_LOCALIZATION_FREE_MARKERS_LOCALIZATION_HPP
#define MARKER_LOCALIZATION_FREE_MARKERS_LOCALIZATION_HPP

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/aruco.hpp>
#include <marker_localization/marker_detect.hpp>
#include <marker_localization/marker_pose_estimator.hpp>
#include <cv_bridge/cv_bridge.h>
#include <memory>

namespace bir {    
    class FreeMarkersLocalization {

    public:
        FreeMarkersLocalization();
        bir::MarkerVector getDetectedMarkers();
        bir::MarkersTransforms getMarkersTransforms(const bir::MarkerVector& detected_and_expected_markers);
        void publishTF(const bir::MarkersTransforms&);
        void publishPose(const bir::MarkersTransforms&);
        void publishImage(const bir::MarkerVector& detected_and_expected_markers);

    private:
        ros::NodeHandle node_, privateNode_;
        image_transport::ImageTransport imgTransport_;
        image_transport::Subscriber subImageTopic_;

        bool update_, enablePublishTF_, enablePublishImage_, enablePublishPose_;
        cv::Mat image_, cameraMatrix_, distCoeffs_;
        cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary_;

        std::string markerTFSufix_, cameraTFName_;
        std::vector< std::pair<int, std::vector<int>> > expectedMarkers_;
        std::vector<int> expectedMarkersIds_;

        std::unique_ptr<bir::MarkerPoseEstimator> poseEstimator_;

        void initializeMarkersLists();
        void initializeCameraParameters();
        void subImageTopicCallback(const sensor_msgs::ImageConstPtr&); 
    };

}

#endif