#include <marker_localization/marker_detect.hpp>

bir::MarkerDetect* bir::MarkerDetect::markerDetect_ = nullptr;

bir::MarkerDetect* bir::MarkerDetect::GetInstance(cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary)
{
  if (markerDetect_ == nullptr)
  {
    markerDetect_ = new MarkerDetect(dictionary);
  }
  else
  {
    markerDetect_->setDictionary(dictionary);
  }

  return markerDetect_;
}

bir::MarkerDetect* bir::MarkerDetect::GetInstance()
{
  if (markerDetect_ == nullptr)
  {
    markerDetect_ = new MarkerDetect();
  }

  return markerDetect_;
}

bir::MarkerDetect::MarkerDetect(cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary)
{
  setDictionary(dictionary);

  _parameters = cv::aruco::DetectorParameters::create();
#if (CV_VERSION_MAJOR > 3 || CV_VERSION_MAJOR == 3 && CV_VERSION_MINOR > 2)
  _parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
#else
  _parameters->doCornerRefinement = true;
#endif
}

void bir::MarkerDetect::setDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary)
{
  _dictionary = cv::aruco::getPredefinedDictionary(dictionary);
}

bir::MarkerVector bir::MarkerDetect::detect(const cv::Mat& img, cv::Point2f offset)
{
  MarkerVector marker_vector;

  if (img.empty())
    return marker_vector;

  cv::aruco::detectMarkers(img, _dictionary, marker_vector.getCorners(), marker_vector.getIDs(), _parameters);

  if (offset != cv::Point2f(0, 0))
  {
    for (int index = 0; index < marker_vector.getIDs().size(); index++)
    {
      for (int index_offset = 0; index_offset < 4; index_offset++)
      {
        /*
            Add the offset take off from the image to the  corner.
            Needed because the marker position is computed from the corner position.
        */
        marker_vector.getCorners().at(index).at(index_offset) += offset;
      }
    }
  }

  return std::move(marker_vector);
}

bir::Marker::Marker() : id(), corner()
{
}

bir::Marker::Marker(const bir::Marker& marker) : id(marker.id), corner(marker.corner)
{
}

bir::Marker::Marker(int id, const std::vector<cv::Point2f>& corner) : id(id), corner(corner)
{
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

bir::MarkerVector::MarkerVector(std::vector<int>& ids, std::vector<std::vector<cv::Point2f>>& corners)
{
  this->ids_ = std::move(ids);
  this->corners_ = std::move(corners);
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
  return bir::Marker();
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
