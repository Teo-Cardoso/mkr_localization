#ifndef MARKER_LOCALIZATION_SINGLE_MARKER_IDENTIFICATION_HPP
#define MARKER_LOCALIZATION_SINGLE_MARKER_IDENTIFICATION_HPP

#include <ros/ros.h>
#include <marker_localization/aruco_identification.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <utility>
#include <string>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <marker_localization/full_marker_estimator.hpp>

namespace bir {    
    struct MarkerPose {
        std::vector<int> ids;
        std::vector<tf::Transform> poses;
        std::vector<double> areas;

        int size() {return ids.size();}
    };


    class SingleMarkerIdentifier {
        public:
        explicit SingleMarkerIdentifier(ros::NodeHandle&);
        bool update_is_available() {return update_;}
        
        bir::MarkerVector detectMarkers();
        void estimatePose(bir::MarkerVector&, bir::MarkerPose&);
        
        private:
        ros::NodeHandle node_;
        image_transport::ImageTransport imgTransport_;
        image_transport::Subscriber subImageTopic_;
        bir::ArucoDetector arucoDetector_;
        bool update_{false}, enableDebug_, enablePublishTF_;
        cv::Mat image_, cameraMatrix_, distCoeffs_;
        cv::Rect roiRectangle_;
        unsigned short roiCounter_{0};
        unsigned roiOffset_[2]{0, 0};
        std::string markerTFName_, cameraTFName_;
        std::vector< std::pair<int, std::vector<int>> > expectedMarkers_; // Vector of IDs and Length.
        std::vector<int> expectedMarkersIds_;
        
        void getRotationAndTranslationValues(bir::MarkerVector&, std::vector<cv::Vec3d>&, std::vector<cv::Vec3d>&, 
                                                                              std::vector<double>&, std::vector<int>&);
        void fillUpCorner(bir::MarkerVector&, std::vector<int>&, std::vector<std::vector<cv::Point2f>>&, 
                                                                                                    std::vector<int>&);
        void initializeMarkersLists();
        void initializeCameraParameters();
        void updateImageROI(const std::vector<std::vector<cv::Point2f>>&, const cv::Size&);
        void subImageTopicCallback(const sensor_msgs::ImageConstPtr&); 

    };

}

#endif