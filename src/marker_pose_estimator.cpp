#include <marker_localization/marker_pose_estimator.hpp>

bir::MarkerPoseEstimator::MarkerPoseEstimator(cv::Mat& camera_matrix, cv::Mat& distortion_coef):
    expectedMarkers_(std::vector<std::pair<int, std::vector<int>>>()), 
    cameraMatrix_(std::move(camera_matrix)), 
    distCoeffs_(std::move(distortion_coef))
{

}

bir::MarkerPoseEstimator::MarkerPoseEstimator(  const std::vector<std::pair<int, std::vector<int>>>& expected_markers,
                                                cv::Mat& camera_matrix, 
                                                cv::Mat& distortion_coef  ):
    expectedMarkers_(expected_markers), 
    cameraMatrix_(std::move(camera_matrix)), 
    distCoeffs_(std::move(distortion_coef))                                                
{

}

void bir::MarkerPoseEstimator::setExpectedMarkers(const std::vector<std::pair<int, std::vector<int>>>& exp_markers) {
    expectedMarkers_ = exp_markers;
}

bir::MarkersTransforms bir::MarkerPoseEstimator::estimatePose(const bir::MarkerVector& marker_vector) {
    bir::MarkersTransforms markers_transforms;
    if (marker_vector.isEmpty()) return markers_transforms;

    std::vector<cv::Vec3d> rotationValues, translationValues;
    
    rotationValues.reserve(marker_vector.size());
    translationValues.reserve(marker_vector.size());
    markers_transforms.areas.reserve(marker_vector.size());
    markers_transforms.ids.reserve(marker_vector.size());

    getRotationAndTranslationValues(marker_vector,
                                    rotationValues,
                                    translationValues,
                                    markers_transforms.areas,
                                    markers_transforms.ids  );

    const size_t markers_transforms_size = markers_transforms.size();
    
    markers_transforms.transforms.reserve(markers_transforms_size);
    markers_transforms.projections_erro.reserve(markers_transforms_size);

    for (int index = 0; index < markers_transforms_size; index++) {
        tf2::Transform transform;

        const double translation[3] = { translationValues.at(index)[0],
                                        translationValues.at(index)[1], 
                                        translationValues.at(index)[2]  };

        transform.setOrigin(tf2::Vector3(translation[0], translation[1], translation[2]));
        transform.setRotation(quaternionFromRodrigues(rotationValues.at(index)));

        markers_transforms.transforms.push_back(std::move(transform));
        markers_transforms.projections_erro.push_back(0.00); //TODO
    }

    return markers_transforms;
}

void bir::MarkerPoseEstimator::getRotationAndTranslationValues( const bir::MarkerVector& marker_vector,
                                                                std::vector<cv::Vec3d>& p_rotation_values,
                                                                std::vector<cv::Vec3d>& p_translation_values,
                                                                std::vector<float>& area,
                                                                std::vector<int>& ids_orders    )
{
    if (marker_vector.isEmpty()) return;

    for (auto marker_type : expectedMarkers_) {
        std::vector<std::vector<cv::Point2f>> corner;
        fillUpCorner(marker_vector, marker_type.second, corner, ids_orders, area);
        
        std::vector<cv::Vec3d> rotationValues, translationValues;
        try {
            cv::aruco::estimatePoseSingleMarkers(   corner, 
                                                    marker_type.first * 1E-3, // Convert mm to m
                                                    cameraMatrix_, 
                                                    distCoeffs_,
                                                    rotationValues,
                                                    translationValues  );
        } catch (cv::Exception& e) {
            ROS_ERROR("getRotationAndTranslationValues: %s", e.what());
        }
        
        p_rotation_values.insert(p_rotation_values.end(), rotationValues.begin(), rotationValues.end());
        p_translation_values.insert(p_translation_values.end(), translationValues.begin(), translationValues.end());
    }
}

void bir::MarkerPoseEstimator::fillUpCorner(const bir::MarkerVector& marker_vector,
                                            const std::vector<int>& markers_id,
                                            std::vector<std::vector<cv::Point2f>>& corner_container,
                                            std::vector<int>& ids_orders,
                                            std::vector<float>& area )
{
    for (int id : markers_id) {
        if (marker_vector.hasID(id)) {
            try {
                bir::Marker marker = marker_vector.withID(id);
                ids_orders.push_back(marker.id);
                corner_container.push_back(std::move(marker.corner));
                area.push_back(cv::contourArea(corner_container.back()));
            } catch (std::exception& e) {
                ROS_ERROR("fillUpCorner: %s", e.what());
            }
        }
    }
}
