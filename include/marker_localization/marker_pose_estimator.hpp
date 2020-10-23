#ifndef MARKER_LOCALIZATION_MARKERPOSEESTIMATOR_HPP
#define MARKER_LOCALIZATION_MARKERPOSEESTIMATOR_HPP

#include <vector>
#include <utility>
#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <marker_localization/marker_detect.hpp>
#include <stdexcept>

namespace bir {
    
    /**
     * @brief return a tf2::Quaternion from a rotation using
     *  rodrigues representation
     * 
     * @param cv::Vec3d rotation_value 
     * @return tf2::Quaternion 
     */
    tf2::Quaternion quaternionFromRodrigues(const cv::Vec3d& rotation_value) {
        double rotationAngle = cv::norm(rotation_value);
        cv::Vec3d rotationAxis = rotation_value/ rotationAngle;
        tf2::Quaternion quaternion;
        quaternion.setRotation(tf2::Vector3(rotationAxis[0], rotationAxis[1], rotationAxis[2]), rotationAngle);
        return quaternion.normalized();
    }

    struct MarkersTransforms {
        std::vector<int> ids;
        std::vector<tf2::Transform> transforms;
        std::vector<float> areas;
        std::vector<float> projections_erro;

        std::size_t size() const {return ids.size();}
    };

    class MarkerPoseEstimator {
    public:
        /**
         * @brief Construct a new Marker Pose Estimator object
         * 
         * @param camera_matrix 
         * @param distortion_coef 
         */
        MarkerPoseEstimator(cv::Mat& camera_matrix, cv::Mat& distortion_coef);
        
        /**
         * @brief Construct a new Marker Pose Estimator object
         * 
         * @param expected_markers 
         * @param camera_matrix 
         * @param distortion_coef 
         */
        MarkerPoseEstimator(const std::vector<std::pair<int, std::vector<int>>>& expected_markers, 
                                                cv::Mat& camera_matrix, cv::Mat& distortion_coef  );
        
        /**
         * @brief Set the Expected Markers
         * 
         * @param expected_markers 
         */
        void setExpectedMarkers(const std::vector<std::pair<int, std::vector<int>>>& expected_markers);
        
        /**
         * @brief Given a marker vector return a marker tranform.
         * 
         * @return bir::MarkersTransforms 
         */
        bir::MarkersTransforms estimatePose(const bir::MarkerVector&);
        
    private:
        std::vector<std::pair<int, std::vector<int>>> expectedMarkers_;
        cv::Mat cameraMatrix_, distCoeffs_;
        std::vector<int> expectedMarkersIds_;
        
        /**
         * @brief Get the Rotation And Translation Values object
         * 
         * @param input marker_vector 
         * @param output p_rotation_values 
         * @param output p_translation_values 
         * @param output area 
         * @param output ids_orders 
         */
        void getRotationAndTranslationValues(   const bir::MarkerVector& marker_vector,
                                                std::vector<cv::Vec3d>& p_rotation_values, 
                                                std::vector<cv::Vec3d>& p_translation_values,
                                                std::vector<float>& area,
                                                std::vector<int>& ids_orders);
        /**
         * @brief Fill up the corner vector with the corners of the right markers
         *  This method is used to select the corners of the expected markers and
         *  markers with the same length.
         * 
         * @param input marker_vector 
         * @param input markers_id 
         * @param output corner_container 
         * @param output ids_orders 
         * @param output area 
         */
        void fillUpCorner(  const bir::MarkerVector& marker_vector, 
                            const std::vector<int>& markers_id,
                            std::vector<std::vector<cv::Point2f>>& corner_container, 
                            std::vector<int>& ids_orders,
                            std::vector<float>& area);
    };
}

#endif
