#ifndef MARKER_LOCALIZATION_MARKER_DECTECT_HPP
#define MARKER_LOCALIZATION_MARKER_DECTECT_HPP

#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/aruco.hpp>
#include <vector>
#include <numeric>

namespace bir
{
class Marker
{
public:
  Marker();
  Marker(const bir::Marker&);
  Marker(int, const std::vector<cv::Point2f>&);
  virtual ~Marker();

  const unsigned int id;
  const std::vector<cv::Point2f> corner;

  double area() const
  {
    return cv::contourArea(corner);
  };
  bool operator==(int id) const;
  bool operator==(Marker marker) const;
};

class MarkerVector
{
public:
  MarkerVector();
  MarkerVector(const bir::MarkerVector&);
  MarkerVector(bir::MarkerVector&&) noexcept;
  MarkerVector(const std::vector<bir::Marker>&);
  MarkerVector(std::vector<int>& ids, std::vector<std::vector<cv::Point2f>>& corners);

  virtual ~MarkerVector();

  size_t size() const
  {
    return ids_.size();
  }
  bool isEmpty() const
  {
    return this->size() == (size_t)0;
  }
  void clear()
  {
    this->ids_.clear();
    this->corners_.clear();
  }

  bir::Marker at(const int index);  // Return a Marker Object.
  bir::Marker withID(int desired_id) const;
  bool hasID(int) const;

  void pushBack(const bir::Marker&);

  std::vector<std::vector<cv::Point2f>>& getCorners();
  std::vector<int>& getIDs();

  MarkerVector& operator=(const MarkerVector&);
  MarkerVector& operator=(MarkerVector&&);

private:
  std::vector<int> ids_;
  std::vector<std::vector<cv::Point2f>> corners_;
};

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

#endif