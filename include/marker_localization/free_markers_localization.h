#ifndef MARKER_LOCALIZATION_FREE_MARKERS_LOCALIZATION_H
#define MARKER_LOCALIZATION_FREE_MARKERS_LOCALIZATION_H

#include <string>
#include <vector>
#include <utility>
#include <memory>

#include <ros/ros.h>
#include <boost/bind.hpp>
#include <image_transport/image_transport.h>

#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/aruco.hpp>
#include <cv_bridge/cv_bridge.h>

#include <marker_localization/marker.h>
#include <marker_localization/marker_detect.h>
#include <marker_localization/marker_pose_estimator.h>
#include <marker_localization/MarkerPoseArray.h>

#include <geometry_msgs/PoseArray.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace bir
{
/**
 * @brief This class handles the node struct of free markers localization.
 *  Its target is to detect and estimate the pose of free markers.
 *  (Free markers are markers that aren't in a map struct)
 *  And with the pose estimated send it through a topic or tf.
 *
 */
class FreeMarkersLocalization
{
public:
  /**
   * @brief Construct a new Free Markers Localization object
   *
   */
  FreeMarkersLocalization();

private:
  ros::NodeHandle node_, privateNode_;
  image_transport::ImageTransport imgTransport_;
  image_transport::Subscriber subImageTopic_;
  image_transport::Publisher pubImageTopic_;
  ros::Publisher pubMarkersPoseTopic_;
  tf2_ros::TransformBroadcaster tfBroadcaster_;

  bool enablePublishTF_, enablePublishImage_, enablePublishPose_;
  cv::Mat cameraMatrix_, distCoeffs_;
  cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary_;

  std::string markerTFPrefix_, cameraTFName_;

  std::vector<std::pair<int, std::vector<int>>> expectedMarkers_;
  std::vector<int> expectedMarkersIds_;

  std::unique_ptr<MarkerPoseEstimator> poseEstimator_;

  /**
   * @brief Get the Detected Markers in the image.
   *
   * @return bir::MarkerVector with detected markers as elements.
   */
  bir::MarkerVector getDetectedMarkers(const cv::Mat& image);

  void initializeMarkersLists();
  void initializeCameraParameters();
  void publishTF(bir::MarkerTransformVector&);
  void publishPose(bir::MarkerTransformVector&);
  void publishImage(cv::Mat& image, bir::MarkerVector& markers_to_draw);
  void runDetectionAndEstimation(cv::Mat& image);
  void subImageTopicCallback(const sensor_msgs::ImageConstPtr&);
  void managerSubscribers();
};

}  // namespace bir

#endif  // MARKER_LOCALIZATION_FREE_MARKERS_LOCALIZATION_H
