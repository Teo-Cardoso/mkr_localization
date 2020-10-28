#include <marker_localization/marker.h>

bir::Marker::Marker(const bir::Marker& marker) : id(marker.id), corner(marker.corner)
{
  if (corner.size() != 4)
  {
    throw(std::invalid_argument("Wrong corner vector size. Expected: 4 | Received: " + std::to_string(corner.size())));
  }
}

bir::Marker::Marker(int id, const std::vector<cv::Point2f>& corner) : id(id), corner(corner)
{
  if (corner.size() != 4)
  {
    throw(std::invalid_argument("Wrong corner vector size. Expected: 4 | Received: " + std::to_string(corner.size())));
  }
}

bir::Marker::~Marker()
{
}

bool bir::Marker::operator==(int id) const
{
  return (this->id == id);
}

bool bir::Marker::operator==(Marker marker) const
{
  return (this->id == marker.id && this->corner == marker.corner);
}

bir::MarkerVector::MarkerVector()
{
}

bir::MarkerVector::MarkerVector(const bir::MarkerVector& marker_vector)
  : ids_(marker_vector.ids_), corners_(marker_vector.corners_)
{
}

bir::MarkerVector::MarkerVector(bir::MarkerVector&& marker_vector) noexcept
  : ids_(std::move(marker_vector.ids_)), corners_(std::move(marker_vector.corners_))
{
  marker_vector.ids_.clear();
  marker_vector.corners_.clear();
}

bir::MarkerVector& bir::MarkerVector::operator=(const MarkerVector& marker_vector)
{
  ids_ = marker_vector.ids_;
  corners_ = marker_vector.corners_;
  return *this;
}

bir::MarkerVector& bir::MarkerVector::operator=(MarkerVector&& marker_vector)
{
  ids_ = std::move(marker_vector.ids_);
  corners_ = std::move(marker_vector.corners_);
  return *this;
}

bir::MarkerVector::MarkerVector(const std::vector<bir::Marker>& vector_of_markers)
{
  for (const Marker& marker : vector_of_markers)
  {
    pushBack(marker);
  }
}

bir::MarkerVector::MarkerVector(const std::vector<int>& ids, const std::vector<std::vector<cv::Point2f>>& corners)
{
  this->ids_ = ids;
  this->corners_ = corners;
}

bir::MarkerVector::~MarkerVector()
{
}

bir::Marker bir::MarkerVector::at(const int index)
{
  return { getIDs().at(index), getCorners().at(index) };
}

bir::Marker bir::MarkerVector::withID(int desired_id) const
{
  std::vector<int>::const_iterator it =
      std::find_if(ids_.begin(), ids_.end(), [desired_id](int id) { return (id == desired_id); });

  if (it != ids_.end())
    return { *it, corners_.at(std::distance(ids_.begin(), it)) };

  throw(std::invalid_argument("ID was not found inside marker vector."));
  return { 0, std::vector<cv::Point2f>({ cv::Point2f(), cv::Point2f(), cv::Point2f(), cv::Point2f() }) };
}

bool bir::MarkerVector::hasID(int desired_id) const
{
  return std::any_of(ids_.begin(), ids_.end(), [desired_id](int id) { return (id == desired_id); });
}

void bir::MarkerVector::pushBack(const bir::Marker& marker)
{
  this->ids_.push_back(marker.id);
  this->corners_.push_back(marker.corner);
}

std::vector<std::vector<cv::Point2f>>& bir::MarkerVector::getCorners()
{
  return this->corners_;
}

std::vector<int>& bir::MarkerVector::getIDs()
{
  return this->ids_;
}