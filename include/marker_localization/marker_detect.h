#ifndef MARKER_LOCALIZATION_MARKER_DETECT_H
#define MARKER_LOCALIZATION_MARKER_DETECT_H

#include <vector>
#include <numeric>

#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/aruco.hpp>

#include <marker_localization/marker.h>

namespace bir
{
class MarkerDetect
{
public:
  MarkerDetect(cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary = cv::aruco::DICT_ARUCO_ORIGINAL);
  
  MarkerVector detect(const cv::Mat&, cv::Point2f offset = cv::Point2f(0.0, 0.0));
  void setDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary);
  void setParameters(cv::aruco::DetectorParameters& parameters);

private:
  cv::Ptr<cv::aruco::DetectorParameters> parameters_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  //todo: use dynamic reconfigure in the parameters/dictionary
};

}  // namespace bir

#endif  // MARKER_LOCALIZATION_MARKER_DETECT_H
