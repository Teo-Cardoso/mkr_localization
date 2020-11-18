#include <opencv4/opencv2/opencv.hpp>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include "marker_localization/MarkerPoseArray.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>

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
class RobotMarkersLocalization
{
public:
  RobotMarkersLocalization();

private:
  ros::NodeHandle node_;
  ros::NodeHandle privateNode_;

  // Publishers and Subscribers
  ros::Subscriber markersSub_;
  ros::Publisher robotPosePub_;

  // TF
  tf2_ros::TransformBroadcaster tfBroadcaster_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;

  // Parameters
  bool enable_tf_, enable_publish_, map_parent_frame_;
  std::string map_name_, base_name_;

  std::pair<std::vector<double>, std::vector<tf2::Transform>>
  estimateCameraPoses(const marker_localization::MarkerPoseArrayConstPtr& markers_pose);
  tf2::Transform getCameraPoseAverage(std::pair<std::vector<double>, std::vector<tf2::Transform>>& camera_poses);
  tf2::Transform getBaseTransform(const std::string& frame_id, const tf2::Transform& camera_pose);
  void publishTF(const geometry_msgs::TransformStamped& base_link_tansform);
  void publishPose(const geometry_msgs::TransformStamped& base_link_tansform);
  inline double errorFunction(double);
  tf2::Quaternion getQuaternionAverage(const cv::Mat& quaternion_average_matrix);
  void markerSubCallback(const marker_localization::MarkerPoseArrayConstPtr&);
};

template <typename vector_type>
std::vector<vector_type> vectorFromMat(const cv::Mat& mat)
{
  std::vector<vector_type> res_vector;
  if (mat.isContinuous())
  {
    res_vector.assign((vector_type*)mat.data, (vector_type*)mat.data + mat.total());
  }
  else
  {
    for (int i = 0; i < mat.rows; ++i)
    {
      res_vector.insert(res_vector.end(), mat.ptr<vector_type>(i), mat.ptr<vector_type>(i) + mat.cols);
    }
  }
  return res_vector;
}
}  // namespace bir
