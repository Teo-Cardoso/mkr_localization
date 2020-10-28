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
  cv::Ptr<cv::aruco::DetectorParameters> _parameters;

  MarkerDetect(const MarkerDetect& other) = delete;
  void operator=(const MarkerDetect& other) = delete;

  static MarkerDetect* markerDetect_;
  static MarkerDetect* GetInstance();
  static MarkerDetect* GetInstance(cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary);
  void setDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary);
  MarkerVector detect(const cv::Mat&, cv::Point2f offset = cv::Point2f(0.0, 0.0));

protected:
  MarkerDetect(cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary = cv::aruco::DICT_ARUCO_ORIGINAL);

  cv::Ptr<cv::aruco::Dictionary> _dictionary;
};

}  // namespace bir

#endif  // MARKER_LOCALIZATION_MARKER_DETECT_H
