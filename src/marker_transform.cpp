#include <utility>
#include <vector>
#include <marker_localization/marker_transform.h>

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

bir::MarkerTransform::MarkerTransform(const MarkerTransform& marker_transform)
  : Marker(marker_transform.id, marker_transform.corner)
  , transform(marker_transform.transform)
  , error(marker_transform.error)
{
}

bir::MarkerTransform::MarkerTransform(const Marker& marker, const tf2::Transform& p_transform, float p_error)
  : Marker(marker), transform(p_transform), error(p_error)
{
}

bir::MarkerTransform::MarkerTransform(int id, const std::vector<cv::Point2f>& corners, const tf2::Transform& transform,
                                      float p_error)
  : Marker(id, corners), transform(transform), error(p_error)
{
}

bool bir::MarkerTransform::operator==(int id) const
{
  return (this->id == id);
}

bool bir::MarkerTransform::operator==(MarkerTransform other) const
{
  return ((id == other.id) && (corner == other.corner) && (transform == other.transform) && (error == other.error));
}

geometry_msgs::TransformStamped bir::MarkerTransform::toTransformStamped(const std::string& parent,
                                                                         const std::string& prefix,
                                                                         const ros::Time& timestamp)
{
  geometry_msgs::TransformStamped transformMsg;
  transformMsg.header.frame_id = parent;
  transformMsg.header.stamp = timestamp;
  transformMsg.child_frame_id = prefix + std::to_string(id);
  transformMsg.transform = tf2::toMsg(transform);

  return transformMsg;
}

marker_localization::MarkerPose bir::MarkerTransform::toMarkerPose()
{
  marker_localization::MarkerPose markerMsg;
  markerMsg.marker_id = id;
  markerMsg.marker_pose = tf2::toMsg(transform);
  markerMsg.error = error;

  return markerMsg;
}

bir::MarkerTransform::~MarkerTransform()
{
}

std::vector<bir::Marker> bir::vecMarkerT2vecMarker(const std::vector<bir::MarkerTransform>& marker_transform_vec)
{
  std::vector<bir::Marker> marker_vec;
  for (auto marker_transform : marker_transform_vec)
  {
    marker_vec.push_back(marker_transform);
  }
  return marker_vec;
}

bir::MarkerTransformVector::MarkerTransformVector()
{
}

bir::MarkerTransformVector::MarkerTransformVector(const bir::MarkerTransformVector& other)
  : MarkerVector(other.ids_, other.corners_)
  , transforms_(other.transforms_)
  , areas_(other.areas_)
  , projectionsErros_(other.projectionsErros_)
{
}

bir::MarkerTransformVector::MarkerTransformVector(bir::MarkerTransformVector&& other) noexcept
  : MarkerVector(std::move(MarkerVector(other.ids_, other.corners_)))
  , transforms_(std::move(other.transforms_))
  , areas_(std::move(other.areas_))
  , projectionsErros_(std::move(other.projectionsErros_))
{
  other.clear();
}

bir::MarkerTransformVector::MarkerTransformVector(const std::vector<bir::MarkerTransform>& marker_transform_vector)
{
  for (auto marker_transform : marker_transform_vector)
  {
    pushBack(marker_transform);
  }
}

bir::MarkerTransformVector::MarkerTransformVector(std::vector<int>& ids, std::vector<std::vector<cv::Point2f>>& corners,
                                                  std::vector<tf2::Transform>& transforms, std::vector<float>& areas,
                                                  std::vector<float>& erros)
  : MarkerVector(ids, corners), transforms_(transforms), areas_(areas), projectionsErros_(erros)
{
}

bir::MarkerTransform bir::MarkerTransformVector::at(int index)
{
  return { ids_.at(index), corners_.at(index), transforms_.at(index), projectionsErros_.at(index) };
}

bir::MarkerTransform bir::MarkerTransformVector::withID(int desired_id)
{
  std::vector<int>::const_iterator it =
      std::find_if(ids_.begin(), ids_.end(), [desired_id](int id) { return (id == desired_id); });

  if (it != ids_.end())
  {
    const auto interator_distance = std::distance<std::vector<int>::const_iterator>(ids_.begin(), it);
    return { *it, corners_.at(interator_distance), transforms_.at(interator_distance),
             projectionsErros_.at(interator_distance) };
  }

  throw(std::invalid_argument("ID was not found inside marker transform vector."));
  return { 0, std::vector<cv::Point2f>({ cv::Point2f(), cv::Point2f(), cv::Point2f(), cv::Point2f() }),
           tf2::Transform(), 0 };
}

void bir::MarkerTransformVector::clear()
{
  this->ids_.clear();
  this->corners_.clear();
  this->transforms_.clear();
  this->areas_.clear();
  this->projectionsErros_.clear();
}

void bir::MarkerTransformVector::pushBack(const bir::MarkerTransform& marker_transform)
{
  ids_.push_back(marker_transform.id);
  corners_.push_back(marker_transform.corner);
  transforms_.push_back(marker_transform.transform);
  areas_.push_back(marker_transform.area());
  projectionsErros_.push_back(marker_transform.error);
}
