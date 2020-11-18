#ifndef MARKER_LOCALIZATION_MARKER_POSE_ESTIMATOR_H
#define MARKER_LOCALIZATION_MARKER_POSE_ESTIMATOR_H

#include <vector>
#include <utility>
#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <marker_localization/marker_detect.h>
#include <marker_localization/marker.h>
#include <marker_localization/marker_transform.h>
#include <stdexcept>

/*
MIT License

Copyright (c) 2020 Etevaldo Cardoso

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

namespace bir
{
/**
 * @brief return a tf2::Quaternion from a rotation using
 *  rodrigues representation
 *
 * @param cv::Vec3d rotation_value
 * @return tf2::Quaternion
 */
tf2::Quaternion quaternionFromRodrigues(const cv::Vec3d& rotation_value)
{
  double rotationAngle = cv::norm(rotation_value);
  cv::Vec3d rotationAxis = rotation_value / rotationAngle;
  tf2::Quaternion quaternion;
  quaternion.setRotation(tf2::Vector3(rotationAxis[0], rotationAxis[1], rotationAxis[2]), rotationAngle);
  return quaternion.normalized();
}

class MarkerPoseEstimator
{
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
  MarkerPoseEstimator(const std::vector<std::pair<int, std::vector<int>>>& expected_markers, cv::Mat& camera_matrix,
                      cv::Mat& distortion_coef);

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
  bir::MarkerTransformVector estimatePose(const bir::MarkerVector&);

private:
  cv::Mat cameraMatrix_, distCoeffs_;
  std::vector<std::pair<int, std::vector<int>>> expectedMarkers_;

  /**
   * @brief Get the Rotation And Translation Values object
   *
   * @param input marker_vector
   * @param output p_rotation_values
   * @param output p_translation_values
   * @param output area
   * @param output ids_orders
   */
  void getRotationAndTranslationValues(const bir::MarkerVector& marker_vector,
                                       std::vector<cv::Vec3d>& p_rotation_values,
                                       std::vector<cv::Vec3d>& p_translation_values, std::vector<float>& area,
                                       std::vector<int>& ids, std::vector<std::vector<cv::Point2f>> corners);
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
  void fillUpCorner(const bir::MarkerVector& marker_vector, const std::vector<int>& markers_id,
                    std::vector<std::vector<cv::Point2f>>& corner_container, std::vector<int>& ids_orders,
                    std::vector<float>& area);
};
}  // namespace bir

#endif  // MARKER_LOCALIZATION_MARKER_POSE_ESTIMATOR_H
