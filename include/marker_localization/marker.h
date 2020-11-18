#ifndef MARKER_LOCALIZATION_MARKER_H
#define MARKER_LOCALIZATION_MARKER_H

#include <vector>
#include <opencv4/opencv2/aruco.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <exception>
#include <string>
#include <utility>

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
/**
 * @brief Base class implementing Marker class.
 * Markers has id and corners attributes.
 *
 */
class Marker
{
public:
  /**
   * @brief Default constructor deleted
   * No Empty Markers Allowed
   */
  Marker() = delete;

  /**
   * @brief Copy Constructor
   *
   */
  Marker(const bir::Marker&);

  /**
   * @brief Construct a new Marker object
   * using the id and the corners
   *
   */
  Marker(int, const std::vector<cv::Point2f>&);
  virtual ~Marker();

  const unsigned int id;
  const std::vector<cv::Point2f> corner;

  /**
   * @brief Using the corners compute the markers
   * area in pixels.
   *
   * @return double
   */
  double area() const
  {
    return cv::contourArea(corner);
  };

  /**
   * @brief Overload == operator with a int
   * If the marker id is equal to the integer
   * return true, false otherwise.
   *
   * @param id
   * @return true
   * @return false
   */
  bool operator==(int id) const;

  /**
   * @brief Overlaod == operator with other Marker
   * If markers ids and corners are equal return
   * true, false otherwise.
   *
   * @param marker
   * @return true
   * @return false
   */
  bool operator==(Marker marker) const;
};

/**
 * @brief Base class implementing a vector-like
 * class for Markers.
 *
 * Note: std::vector<Marker> wasn't used because
 * there is interesting to have direct access to
 * a vector of ids and vector of corners.
 *
 */
class MarkerVector
{
public:
  /**
   * @brief Construct a new and empty Marker Vector object
   *
   */
  MarkerVector();

  /**
   * @brief Construct a new Marker Vector object using
   * copy constructor.
   *
   */
  MarkerVector(const bir::MarkerVector&);

  /**
   * @brief Construct a new Marker Vector object using
   * move constructor.
   *
   */
  MarkerVector(bir::MarkerVector&&) noexcept;

  /**
   * @brief Construct a new Marker Vector object
   * using a std::vector<Marker>
   */
  MarkerVector(const std::vector<bir::Marker>&);

  /**
   * @brief Construct a new Marker Vector object
   * using a vector of ids and a vector of vector of corners.
   *
   * @param ids
   * @param corners
   */
  MarkerVector(const std::vector<int>& ids, const std::vector<std::vector<cv::Point2f>>& corners);

  virtual ~MarkerVector();

  /**
   * @brief Return the number of Markers stored in the object
   *
   * @return size_t
   */
  size_t size() const
  {
    return ids_.size();
  }

  /**
   * @brief Return if the Marker vector is empty
   *
   * @return true if empty
   * @return false otherwise
   */
  bool isEmpty() const
  {
    return this->size() == (size_t)0;
  }

  /**
   * @brief Clear the vectors
   *
   */
  virtual void clear()
  {
    this->ids_.clear();
    this->corners_.clear();
  }

  /**
   * @brief Return the Marker object at a specific index
   *
   * @param index
   * @return bir::Marker
   */
  bir::Marker at(const int index);

  /**
   * @brief Return a Marker object with a specific id.
   *
   * @param desired_id
   * @return bir::Marker
   */
  bir::Marker withID(int desired_id) const;

  /**
   * @brief Return with the marker vector contains a marker
   * with a specific id.
   *
   * @param id
   * @return true if has the marker with the specific id.
   * @return false otherwise
   */
  bool hasID(int) const;

  /**
   * @brief Add a marker into the marker vector
   *
   */
  void pushBack(const bir::Marker&);

  /**
   * @brief Get the Vector of Corners
   *
   * @return std::vector<std::vector<cv::Point2f>>&
   */
  std::vector<std::vector<cv::Point2f>>& getCorners();

  /**
   * @brief Get the Vector of IDs
   *
   * @return std::vector<int>&
   */
  std::vector<int>& getIDs();

  /**
   * @brief Copy assignment
   *
   * @return MarkerVector&
   */
  MarkerVector& operator=(const MarkerVector&);

  /**
   * @brief Move assigment
   *
   * @return MarkerVector&
   */
  MarkerVector& operator=(MarkerVector&&);

protected:
  std::vector<int> ids_;
  std::vector<std::vector<cv::Point2f>> corners_;
};

}  // namespace bir

#endif  // MARKER_LOCALIZATION_MARKER_H
