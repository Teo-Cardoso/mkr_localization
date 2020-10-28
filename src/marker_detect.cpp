#include <marker_localization/marker_detect.h>

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
