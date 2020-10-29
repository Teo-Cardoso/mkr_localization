#include <marker_localization/marker_detect.h>

bir::MarkerDetect::MarkerDetect(cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary)
{
  setDictionary(dictionary);

  parameters_ = cv::aruco::DetectorParameters::create();
#if (CV_VERSION_MAJOR > 3 || CV_VERSION_MAJOR == 3 && CV_VERSION_MINOR > 2)
  parameters_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
#else
  parameters_->doCornerRefinement = true;
#endif
}

void bir::MarkerDetect::setDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary)
{
  dictionary_ = cv::aruco::getPredefinedDictionary(dictionary);
}

bir::MarkerVector bir::MarkerDetect::detect(const cv::Mat& img, cv::Point2f offset)
{
  MarkerVector marker_vector;

  if (img.empty())
    return marker_vector;

  cv::aruco::detectMarkers(img, dictionary_, marker_vector.getCorners(), marker_vector.getIDs(), parameters_);

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

  return marker_vector;
}
