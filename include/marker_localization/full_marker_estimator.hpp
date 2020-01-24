#ifndef MARKER_LOCALIZATION_FULL_MARKER_ESTIMATOR_HPP
#define MARKER_LOCALIZATION_FULL_MARKER_ESTIMATOR_HPP

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <numeric>
#include <deque>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <marker_localization/single_marker_identification.hpp>
#include <boost/array.hpp>
namespace bir {

    struct MarkerPose;

    class FullMarkerEstimator {
        public:
            FullMarkerEstimator(ros::NodeHandle&);
            // FullMarkerEstimator(ros::NodeHandle&, bir::SingleMarkerIdentifier&);
            ~FullMarkerEstimator();
            
            void estimatePose(bir::MarkerPose&, tf::Transform&);
            void publish(const tf::Transform&);

        private:
            ros::NodeHandle node_;
            ros::Publisher posePublisher_;
            bool enableDebug_, enablePublishTF_, staticBaseCameraTransform_, staticVariance_,staticMarker_;
            std::string markerTFName_, cameraTFName_, mapTFName_, baseTFName_;
            tf::TransformListener transformListener_;
            tf::StampedTransform baseCameraTransform_;
            std::vector<tf::StampedTransform> staticMarkersTransforms_;
            std::vector<double> varianceValues_;
            std::vector<int> staticMarkersId_;
            std::deque<tf::Transform> oldCameraTransforms_;
            
            void estimateCameraPositions(bir::MarkerPose& marker_poses,
                                                          std::pair<std::vector<double>, std::vector<tf::Transform>>&);
            void getCameraPoseAverage(std::pair<std::vector<double>, std::vector<tf::Transform>>&, tf::Transform&);

    };
}

template <typename vector_type> std::vector<vector_type> vectorFromMat(const cv::Mat& mat) {
    std::vector<vector_type> res_vector;
    if (mat.isContinuous()) {
        res_vector.assign((vector_type*)mat.data, (vector_type*)mat.data + mat.total());
    } else {
        for (int i = 0; i < mat.rows; ++i) {
            res_vector.insert(res_vector.end(), mat.ptr<vector_type>(i), mat.ptr<vector_type>(i)+mat.cols);
        }
    }
    return res_vector;
}

#endif