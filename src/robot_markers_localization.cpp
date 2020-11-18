#include <marker_localization/robot_markers_localization.h>

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

bir::RobotMarkersLocalization::RobotMarkersLocalization() : node_(), privateNode_("~"), tfListener_(tfBuffer_)
{
  enable_tf_ = privateNode_.param<bool>("enable/tf", true);
  enable_publish_ = privateNode_.param<bool>("enable/publish_pose", true);
  map_parent_frame_ = !(privateNode_.param<bool>("robot_as_parent", false));

  map_name_ = privateNode_.param<std::string>("map_frame", "map");
  base_name_ = privateNode_.param<std::string>("base_link_frame", "base_link");

  markersSub_ = node_.subscribe("/detected_markers/output", 2, &bir::RobotMarkersLocalization::markerSubCallback, this);
  if (enable_publish_)
  {
    robotPosePub_ = node_.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 10);
  }
}

std::pair<std::vector<double>, std::vector<tf2::Transform>>
bir::RobotMarkersLocalization::estimateCameraPoses(const marker_localization::MarkerPoseArrayConstPtr& markers_pose)
{
  std::pair<std::vector<double>, std::vector<tf2::Transform>> cameraPoses;
  std::vector<double>& weights = cameraPoses.first;
  std::vector<tf2::Transform>& rawCameraTransforms = cameraPoses.second;

  for (auto marker : markers_pose->markers)
  {
    tf2::Transform cameraMarker, markerWorld;
    tf2::fromMsg(marker.marker_pose, cameraMarker);

    try
    {
      tf2::fromMsg(
          tfBuffer_
              .lookupTransform(map_name_, markers_pose->frame_prefix + std::to_string(marker.marker_id), ros::Time(0))
              .transform,
          markerWorld);
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
      continue;
    }

    tf2::Transform cameraWorld = markerWorld * cameraMarker.inverse();

    weights.push_back(marker.error);
    rawCameraTransforms.push_back(cameraWorld);
  }

  return cameraPoses;
}

tf2::Transform bir::RobotMarkersLocalization::getCameraPoseAverage(
    std::pair<std::vector<double>, std::vector<tf2::Transform>>& camera_poses)
{
  std::vector<double>& weights = camera_poses.first;
  std::vector<tf2::Transform>& rawCameraTransforms = camera_poses.second;
  tf2::Vector3 translateAverage(0.0, 0.0, 0.0);
  cv::Mat quaternionAverageMatrix = cv::Mat::zeros(4, 4, CV_64F);

  double totalWeight = 0;
  const int transformSize = rawCameraTransforms.size();

  for (int index = 0; index < transformSize; index++)
  {
    const double weight = errorFunction(weights.at(index));
    totalWeight += weight;

    translateAverage += rawCameraTransforms.at(index).getOrigin() * weight;

    cv::Mat quaternion(4, 1, CV_64F);
    const tf2::Quaternion rotation = rawCameraTransforms.at(index).getRotation();
    quaternion.at<double>(0, 0) = rotation.getW();
    quaternion.at<double>(1, 0) = rotation.getX();
    quaternion.at<double>(2, 0) = rotation.getY();
    quaternion.at<double>(3, 0) = rotation.getZ();
    quaternionAverageMatrix += (weight * (quaternion * quaternion.t()));
  }

  translateAverage /= totalWeight;
  quaternionAverageMatrix /= totalWeight;

  /* Get Quaternion Average */
  const tf2::Quaternion quaternionAverage = getQuaternionAverage(quaternionAverageMatrix);

  return tf2::Transform(quaternionAverage, translateAverage);
}

void bir::RobotMarkersLocalization::publishTF(const geometry_msgs::TransformStamped& base_link_transform)
{
  if (!enable_tf_)
    return;

  tfBroadcaster_.sendTransform(base_link_transform);
}

void bir::RobotMarkersLocalization::publishPose(const geometry_msgs::TransformStamped& base_link_transform)
{
  if (!enable_publish_)
    return;

  geometry_msgs::PoseWithCovarianceStamped pose;
  pose.header = base_link_transform.header;
  pose.header.frame_id = base_name_;
  pose.pose.pose.position.x = base_link_transform.transform.translation.x;
  pose.pose.pose.position.y = base_link_transform.transform.translation.y;
  pose.pose.pose.position.z = base_link_transform.transform.translation.z;
  pose.pose.pose.orientation = base_link_transform.transform.rotation;

  robotPosePub_.publish(pose);
}

tf2::Transform bir::RobotMarkersLocalization::getBaseTransform(const std::string& frame_id,
                                                               const tf2::Transform& camera_pose)
{
  tf2::Transform baseCamera, worldBase;
  try
  {
    tf2::fromMsg(tfBuffer_.lookupTransform(frame_id, base_name_, ros::Time(0)).transform, baseCamera);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_FATAL("%s", ex.what());
    ROS_ISSUE_BREAK();
  }

  return camera_pose * baseCamera;
}

void bir::RobotMarkersLocalization::markerSubCallback(const marker_localization::MarkerPoseArrayConstPtr& markers)
{
  auto rawPoses = estimateCameraPoses(markers);
  auto cameraPose = getCameraPoseAverage(rawPoses);
  auto worldPose = getBaseTransform(markers->header.frame_id, cameraPose);

  geometry_msgs::TransformStamped msg;
  msg.header.stamp = ros::Time::now();
  if (map_parent_frame_)
  {
    msg.child_frame_id = base_name_;
    msg.header.frame_id = map_name_;
    msg.transform = tf2::toMsg(worldPose);
  }
  else
  {
    msg.child_frame_id = map_name_;
    msg.header.frame_id = base_name_;
    msg.transform = tf2::toMsg(worldPose.inverse());
  }

  publishTF(msg);
  publishPose(msg);
}

tf2::Quaternion bir::RobotMarkersLocalization::getQuaternionAverage(const cv::Mat& quaternion_average_matrix)
{
  cv::Mat E, V;
  cv::eigen(quaternion_average_matrix, E, V);
  std::vector<double> eigenValues = vectorFromMat<double>(E);
  const int index = std::max_element(eigenValues.begin(), eigenValues.end()) - eigenValues.begin();

  /* Find Quaternion Average */
  tf2::Quaternion quaternionAverage;
  quaternionAverage.setW(V.at<double>(index, 0));
  quaternionAverage.setX(V.at<double>(index, 1));
  quaternionAverage.setY(V.at<double>(index, 2));
  quaternionAverage.setZ(V.at<double>(index, 3));

  return quaternionAverage.normalized();
}

inline double bir::RobotMarkersLocalization::errorFunction(double error)
{
  return exp(-2 * error);
}
