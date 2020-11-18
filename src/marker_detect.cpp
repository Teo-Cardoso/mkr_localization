#include <marker_localization/marker_detect.h>

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

bir::MarkerDetect::MarkerDetect(cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary)
{
  setDictionary(dictionary);

  parameters_ = cv::aruco::DetectorParameters::create();
#if (CV_VERSION_MAJOR > 3 || (CV_VERSION_MAJOR == 3 && CV_VERSION_MINOR > 2))
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
