#ifndef MARKER_LOCALIZATION_MARKERPOSEESTIMATOR_HPP
#define MARKER_LOCALIZATION_MARKERPOSEESTIMATOR_HPP

#include <vector>
#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <marker_localization/marker_detect.hpp>
#include <stdexcept>

tf2::Quaternion quaternionFromRodrigues(const cv::Vec3d& rotation_value) {
    double rotationAngle = cv::norm(rotation_value);
    cv::Vec3d rotationAxis = rotation_value/ rotationAngle;
    tf2::Quaternion quaternion;
    quaternion.setRotation(tf2::Vector3(rotationAxis[0], rotationAxis[1], rotationAxis[2]), rotationAngle);
    return quaternion.normalized();
}

namespace bir {    

    struct MarkersTransforms {
        std::vector<int> ids;
        std::vector<tf2::Transform> transforms;
        std::vector<float> areas;
        std::vector<float> projections_erro;

        std::size_t size() {return ids.size();}
    };

    class MarkerPoseEstimator {
    public:
        MarkerPoseEstimator(const cv::Mat& camera_matrix, const cv::Mat& distortion_coef);
        MarkerPoseEstimator(const std::vector<std::pair<int, std::vector<int>>>& expected_markers, 
                            const cv::Mat& camera_matrix, 
                            const cv::Mat& distortion_coef  );
        
        void setExpectedMarkers(const std::vector<std::pair<int, std::vector<int>>>& expected_markers);
        bir::MarkersTransforms estimatePose(const bir::MarkerVector&);
        
    private:
        std::vector<std::pair<int, std::vector<int>>> expectedMarkers_;
        cv::Mat cameraMatrix_, distCoeffs_;
        std::vector<int> expectedMarkersIds_;
        
        void getRotationAndTranslationValues(   const bir::MarkerVector& marker_vector,
                                                std::vector<cv::Vec3d>& p_rotation_values, 
                                                std::vector<cv::Vec3d>& p_translation_values,
                                                std::vector<float>& area,
                                                std::vector<int>& ids_orders);

        void fillUpCorner(  const bir::MarkerVector& marker_vector, 
                            const std::vector<int>& markers_id,
                            std::vector<std::vector<cv::Point2f>>& corner_container, 
                            std::vector<int>& ids_orders,
                            std::vector<float>& area);
    };
}

#endif