#ifndef MARKER_LOCALIZATION_MARKER_DETECT_H
#define MARKER_LOCALIZATION_MARKER_DETECT_H

#include <vector>
#include <numeric>

#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/aruco.hpp>

#include <marker_localization/marker.h>

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
  // todo: use dynamic reconfigure in the parameters/dictionary
};

}  // namespace bir

#endif  // MARKER_LOCALIZATION_MARKER_DETECT_H
