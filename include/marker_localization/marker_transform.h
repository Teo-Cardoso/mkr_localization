#ifndef MARKER_LOCALIZATION_MARKER_TRANSFORM_H
#define MARKER_LOCALIZATION_MARKER_TRANSFORM_H

#include <vector>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <marker_localization/marker.h>
#include <geometry_msgs/TransformStamped.h>
#include <marker_localization/MarkerPose.h>
#include <marker_localization/MarkerPoseArray.h>
#include <utility>

namespace bir
{

/**
 * @brief Class implementing Marker Transform 
 * class using Marker as base.
 * Markers Transform has id, corners, transform and 
 * error as attributes.
 * 
 */
class MarkerTransform : public Marker
{
public:
  /**
  * @brief Default constructor deleted 
  * No Empty MarkersTransform Allowed
  */
  MarkerTransform() = delete;

  /**
   * @brief Copy Constructor
   * 
   */
  MarkerTransform(const MarkerTransform&);

  /**
   * @brief Construct a new Marker Transform object using
   * a Marker object, tf2::transform and an error.
   * 
   * @param error 
   */
  MarkerTransform(const Marker&, const tf2::Transform&, float error);

  /**
   * @brief Construct a new Marker Transform object
   * using the marker id, the corners, a tf2::transform and an error.
   * 
   * @param error 
   */
  MarkerTransform(int, const std::vector<cv::Point2f>&, const tf2::Transform&, float error);

  virtual ~MarkerTransform();

  const tf2::Transform transform;
  const float error;

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
   * @brief Overlaod == operator with other Marker Transform
   * If markers ids, corners, transform and error are equal return
   * true, false otherwise.
   * 
   * @param MarkerTransform 
   * @return true 
   * @return false 
   */
  bool operator==(MarkerTransform) const;

  /**
   * @brief Transform the MarkerTransform into the
   * geometry_msgs::TransformStamped.
   * 
   * @param parent Parent Link
   * @param prefix Prefix used before marker id
   * @param timestamp 
   * @return geometry_msgs::TransformStamped 
   */
  geometry_msgs::TransformStamped toTransformStamped(const std::string& parent, const std::string& prefix,
                                                     const ros::Time& timestamp);
  
  /**
   * @brief Transform MarkerTransform into the
   * marker_localization::MarkerPose
   * 
   * @return marker_localization::MarkerPose 
   */
  marker_localization::MarkerPose toMarkerPose();
};

/**
 * @brief Transform a vector of Markers Transforms into a 
 * vector of Markers.
 * 
 * @return std::vector<bir::Marker> 
 */
std::vector<bir::Marker> vecMarkerT2vecMarker(const std::vector<bir::MarkerTransform>&);

/**
 * @brief Class implementing a vector-like class for
 * Markers Transforms
 * 
 */
class MarkerTransformVector : public MarkerVector
{
public:

  /**
   * @brief Construct a new empty Marker Transform Vector object
   * 
   */
  MarkerTransformVector();

  /**
   * @brief Construct a new Marker Transform Vector object using 
   * copy constructor.
   * 
   */
  MarkerTransformVector(const bir::MarkerTransformVector&);

  /**
   * @brief Construct a new Marker Transform VVector object using 
   * move constructor.
   * 
   */
  MarkerTransformVector(bir::MarkerTransformVector&&) noexcept;

  /**
   * @brief Construct a new Marker Transform Vector object using
   * a vector of MarkerTransform
   * 
   */
  MarkerTransformVector(const std::vector<bir::MarkerTransform>&);

  /**
   * @brief Construct a new Marker Transform Vector object using
   * the vectors of id, corner, transform, area and erro.
   * 
   * @param ids 
   * @param corners 
   * @param transforms 
   * @param areas 
   * @param erros 
   */
  MarkerTransformVector(std::vector<int>& ids, std::vector<std::vector<cv::Point2f>>& corners,
                        std::vector<tf2::Transform>& transforms, std::vector<float>& areas, std::vector<float>& erros);

  /**
   * @brief Return the number of MarkerTransform stored in the object
   * 
   * @return std::size_t 
   */
  std::size_t size() const
  {
    return ids_.size();
  }

  /**
   * @brief Return the MarkerTransform object at a specific index
   * 
   * @param index 
   * @return MarkerTransform 
   */
  MarkerTransform at(int index);

  /**
   * @brief Return a MarkerTransform object with a specific id.
   * 
   * @param desired_id 
   * @return MarkerTransform 
   */
  MarkerTransform withID(int desired_id);

  /**
   * @brief Add a MarkerTransform into the MarkerTransformVector
   * 
   */
  void pushBack(const bir::MarkerTransform&);

  /**
   * @brief Clear the vectors
   * 
   */
  virtual void clear() override;

  /**
   * @brief Get the Transforms vector
   * 
   * @return std::vector<tf2::Transform>& 
   */
  std::vector<tf2::Transform>& getTransforms()
  {
    return transforms_;
  };

  /**
   * @brief Get the Areas vector
   * 
   * @return std::vector<float>& 
   */
  std::vector<float>& getAreas()
  {
    return areas_;
  };

  /**
   * @brief Get the Projections Erros vector
   * 
   * @return std::vector<float>& 
   */
  std::vector<float>& getProjectionsErros()
  {
    return projectionsErros_;
  };

protected:
  std::vector<tf2::Transform> transforms_;
  std::vector<float> areas_;
  std::vector<float> projectionsErros_;
};

}  // namespace bir

#endif  // MARKER_LOCALIZATION_MARKER_TRANSFORM_H
