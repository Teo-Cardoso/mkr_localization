#include <iostream>
#include <string>
#include <vector>
#include <utility>

#include <gtest/gtest.h>

#include <marker_localization/marker_transform.h>
#include <marker_localization/MarkerPose.h>
#include <tf2/LinearMath/Transform.h>

TEST(MarkerTransformTest, test_markertransform_constructors)
{
  const auto corners =
      std::vector<cv::Point2f>({ cv::Point2f(4, 4), cv::Point2f(5, 4), cv::Point2f(5, 5), cv::Point2f(4, 5) });
  const auto transform = tf2::Transform();

  // Explicit Constructor
  bir::MarkerTransform marker_transform1 = { 10, corners, transform, 11.33 };

  EXPECT_EQ(10, marker_transform1.id);
  EXPECT_EQ(corners, marker_transform1.corner);
  EXPECT_EQ(transform, marker_transform1.transform);
  EXPECT_NEAR(11.33, marker_transform1.error, 0.001);

  // Copy Constructor
  auto marker_transform2 = marker_transform1;

  EXPECT_EQ(10, marker_transform2.id);
  EXPECT_EQ(corners, marker_transform2.corner);
  EXPECT_EQ(transform, marker_transform2.transform);
  EXPECT_NEAR(11.33, marker_transform2.error, 0.001);

  // Marker-based Constructor
  auto marker1 = bir::Marker(1, corners);
  auto marker_transform3 = bir::MarkerTransform(marker1, transform, 14.12);

  EXPECT_EQ(1, marker1.id);
  EXPECT_EQ(corners, marker1.corner);
  EXPECT_EQ(transform, transform);
  EXPECT_NEAR(14.12, marker_transform3.error, 0.001);
}

TEST(MarkerTransformTest, test_markertransform_equal)
{
  const auto corners =
      std::vector<cv::Point2f>({ cv::Point2f(4, 4), cv::Point2f(5, 4), cv::Point2f(5, 5), cv::Point2f(4, 5) });
  const auto transform = tf2::Transform();
  bir::MarkerTransform marker_transform1 = { 10, corners, transform, 11.33 };

  EXPECT_TRUE(marker_transform1 == 10);
  EXPECT_FALSE(marker_transform1 == 14);

  auto marker1 = bir::Marker(1, corners);
  auto marker2 = bir::Marker(10, corners);

  EXPECT_FALSE(marker1 == marker_transform1);
  EXPECT_TRUE(marker2 == marker_transform1);

  auto marker_transform2 = marker_transform1;
  bir::MarkerTransform markertransform3 = { 16, corners, transform, 12.33 };
  bir::MarkerTransform markertransform4 = { 10, corners, transform, 12.33 };
  bir::MarkerTransform markertransform5 = { 16, corners, transform, 11.33 };

  EXPECT_TRUE(marker_transform2 == marker_transform1);
  EXPECT_FALSE(markertransform3 == marker_transform1);
  EXPECT_FALSE(markertransform4 == marker_transform1);
  EXPECT_FALSE(markertransform5 == marker_transform1);
}

TEST(MarkerTransformTest, test_markertransform2transformstamped_msg)
{
  auto time_now = ros::Time(0);
  geometry_msgs::TransformStamped reference;
  reference.header.frame_id = "camera";
  reference.header.stamp = time_now;
  reference.child_frame_id = "id_10";
  reference.transform.translation.x = 0.00;
  reference.transform.translation.y = 0.00;
  reference.transform.translation.z = 0.00;
  reference.transform.rotation.w = 1.00;
  reference.transform.rotation.x = 0.00;
  reference.transform.rotation.y = 0.00;
  reference.transform.rotation.z = 0.00;

  const auto corners =
      std::vector<cv::Point2f>({ cv::Point2f(4, 4), cv::Point2f(5, 4), cv::Point2f(5, 5), cv::Point2f(4, 5) });
  const auto transform = tf2::Transform(tf2::Quaternion(0, 0, 0, 1), { 0, 0, 0 });
  bir::MarkerTransform marker_transform1 = { 10, corners, transform, 11.33 };

  auto marker_transform_stamped1 = marker_transform1.toTransformStamped("camera", "id_", time_now);

  EXPECT_EQ(reference.header.frame_id, marker_transform_stamped1.header.frame_id);
  EXPECT_EQ(reference.header.stamp, marker_transform_stamped1.header.stamp);
  EXPECT_EQ(reference.child_frame_id, marker_transform_stamped1.child_frame_id);
  EXPECT_NEAR(reference.transform.translation.x, marker_transform_stamped1.transform.translation.x, 0.001);
  EXPECT_NEAR(reference.transform.translation.y, marker_transform_stamped1.transform.translation.y, 0.001);
  EXPECT_NEAR(reference.transform.translation.z, marker_transform_stamped1.transform.translation.z, 0.001);
  EXPECT_NEAR(reference.transform.rotation.w, marker_transform_stamped1.transform.rotation.w, 0.001);
  EXPECT_NEAR(reference.transform.rotation.x, marker_transform_stamped1.transform.rotation.x, 0.001);
  EXPECT_NEAR(reference.transform.rotation.y, marker_transform_stamped1.transform.rotation.y, 0.001);
  EXPECT_NEAR(reference.transform.rotation.z, marker_transform_stamped1.transform.rotation.z, 0.001);
}

TEST(MarkerTransformTest, test_markertransform2markerpose_msg)
{
  auto time_now = ros::Time(0);
  geometry_msgs::Transform reference_transform;
  reference_transform.translation.x = 0.00;
  reference_transform.translation.y = 0.00;
  reference_transform.translation.z = 0.00;
  reference_transform.rotation.w = 1.00;
  reference_transform.rotation.x = 0.00;
  reference_transform.rotation.y = 0.00;
  reference_transform.rotation.z = 0.00;

  marker_localization::MarkerPose reference;
  reference.marker_id = 10;
  reference.marker_pose = reference_transform;
  reference.error = 0.02;

  const auto corners =
      std::vector<cv::Point2f>({ cv::Point2f(4, 4), cv::Point2f(5, 4), cv::Point2f(5, 5), cv::Point2f(4, 5) });
  const auto transform = tf2::Transform(tf2::Quaternion(0, 0, 0, 1), { 0, 0, 0 });
  bir::MarkerTransform marker_transform1 = { 10, corners, transform, 0.02 };

  auto marker_pose1 = marker_transform1.toMarkerPose();

  EXPECT_EQ(reference.marker_id, marker_pose1.marker_id);
  EXPECT_NEAR(reference.error, marker_pose1.error, 0.001);
  EXPECT_NEAR(reference.marker_pose.translation.x, marker_pose1.marker_pose.translation.x, 0.001);
  EXPECT_NEAR(reference.marker_pose.translation.y, marker_pose1.marker_pose.translation.y, 0.001);
  EXPECT_NEAR(reference.marker_pose.translation.z, marker_pose1.marker_pose.translation.z, 0.001);
  EXPECT_NEAR(reference.marker_pose.rotation.w, marker_pose1.marker_pose.rotation.w, 0.001);
  EXPECT_NEAR(reference.marker_pose.rotation.x, marker_pose1.marker_pose.rotation.x, 0.001);
  EXPECT_NEAR(reference.marker_pose.rotation.y, marker_pose1.marker_pose.rotation.y, 0.001);
  EXPECT_NEAR(reference.marker_pose.rotation.z, marker_pose1.marker_pose.rotation.z, 0.001);
}

TEST(MarkerTransformTest, test_vecMarkerT2vecMarker)
{
  const auto corners =
      std::vector<cv::Point2f>({ cv::Point2f(4, 4), cv::Point2f(5, 4), cv::Point2f(5, 5), cv::Point2f(4, 5) });
  const auto transform = tf2::Transform(tf2::Quaternion(0, 0, 0, 1), { 0, 0, 0 });

  bir::MarkerTransform marker_transform0 = { 0, corners, transform, 0.02 };
  bir::MarkerTransform marker_transform1 = { 1, corners, transform, 0.02 };
  bir::MarkerTransform marker_transform2 = { 2, corners, transform, 0.02 };
  bir::MarkerTransform marker_transform3 = { 3, corners, transform, 0.02 };
  bir::MarkerTransform marker_transform4 = { 4, corners, transform, 0.02 };
  bir::MarkerTransform marker_transform5 = { 5, corners, transform, 0.02 };
  bir::MarkerTransform marker_transform6 = { 6, corners, transform, 0.02 };
  bir::MarkerTransform marker_transform7 = { 7, corners, transform, 0.02 };

  std::vector<bir::MarkerTransform> vecMarkerTransform = { marker_transform0, marker_transform1, marker_transform2,
                                                           marker_transform3, marker_transform4, marker_transform7,
                                                           marker_transform6, marker_transform5 };

  std::vector<bir::Marker> vecMarker = bir::vecMarkerT2vecMarker(vecMarkerTransform);

  EXPECT_EQ(vecMarker.at(0).id, 0);
  EXPECT_EQ(vecMarker.at(1).id, 1);
  EXPECT_EQ(vecMarker.at(2).id, 2);
  EXPECT_EQ(vecMarker.at(3).id, 3);
  EXPECT_EQ(vecMarker.at(4).id, 4);
  EXPECT_EQ(vecMarker.at(5).id, 7);
  EXPECT_EQ(vecMarker.at(6).id, 6);
  EXPECT_EQ(vecMarker.at(7).id, 5);
}

TEST(MarkerTransformVector, test_mtvector_constructors)
{
  // Empty Constructor
  bir::MarkerTransformVector transform_vector1;
  EXPECT_TRUE(transform_vector1.isEmpty());

  // Vector of Transforms Constructor
  const auto corners =
      std::vector<cv::Point2f>({ cv::Point2f(4, 4), cv::Point2f(5, 4), cv::Point2f(5, 5), cv::Point2f(4, 5) });
  const auto transform = tf2::Transform(tf2::Quaternion(0, 0, 0, 1), { 0, 0, 0 });

  bir::MarkerTransform marker_transform0 = { 0, corners, transform, 0.02 };
  bir::MarkerTransform marker_transform1 = { 1, corners, transform, 0.02 };
  bir::MarkerTransform marker_transform2 = { 2, corners, transform, 0.02 };
  bir::MarkerTransform marker_transform3 = { 3, corners, transform, 0.02 };
  bir::MarkerTransform marker_transform4 = { 4, corners, transform, 0.02 };
  bir::MarkerTransform marker_transform5 = { 5, corners, transform, 0.02 };
  bir::MarkerTransform marker_transform6 = { 6, corners, transform, 0.02 };
  bir::MarkerTransform marker_transform7 = { 7, corners, transform, 0.02 };

  std::vector<bir::MarkerTransform> vecMarkerTransform = { marker_transform0, marker_transform1, marker_transform2,
                                                           marker_transform3, marker_transform4, marker_transform7,
                                                           marker_transform6, marker_transform5 };

  bir::MarkerTransformVector transform_vector2 = vecMarkerTransform;
  EXPECT_EQ(8, (int)transform_vector2.size());

  // Copy Constructor
  auto transform_vector3 = transform_vector2;
  EXPECT_FALSE(transform_vector3.isEmpty());
  EXPECT_EQ(8, (int)transform_vector3.size());

  // Move Constructor
  auto transform_vector4 = std::move(transform_vector3);
  EXPECT_EQ(0, (int)transform_vector3.size());
  EXPECT_TRUE(transform_vector3.isEmpty());
  EXPECT_FALSE(transform_vector4.isEmpty());
  EXPECT_EQ(8, (int)transform_vector4.size());

  // Explicit Constructor
  std::vector<int> ids_vec = { 1, 2, 3 };
  std::vector<std::vector<cv::Point2f>> corners_vec = { corners, corners, corners };
  std::vector<tf2::Transform> transforms_vec = { transform, transform, transform };
  std::vector<float> error_vec = { 0.00, 0.00, 0.00 };
  std::vector<float> area_vec = { 0.00, 0.00, 0.00 };

  bir::MarkerTransformVector transform_vector5(ids_vec, corners_vec, transforms_vec, area_vec, error_vec);
  EXPECT_EQ(3, (int)transform_vector5.size());
}

TEST(MarkerTransformVector, test_mtvector_at)
{
  const auto corners =
      std::vector<cv::Point2f>({ cv::Point2f(4, 4), cv::Point2f(5, 4), cv::Point2f(5, 5), cv::Point2f(4, 5) });
  const auto transform = tf2::Transform(tf2::Quaternion(0, 0, 0, 1), { 0, 0, 0 });

  bir::MarkerTransform marker_transform0 = { 0, corners, transform, 0.02 };
  bir::MarkerTransform marker_transform1 = { 1, corners, transform, 0.02 };
  bir::MarkerTransform marker_transform2 = { 2, corners, transform, 0.02 };
  bir::MarkerTransform marker_transform3 = { 3, corners, transform, 0.02 };
  bir::MarkerTransform marker_transform4 = { 4, corners, transform, 0.02 };
  bir::MarkerTransform marker_transform5 = { 5, corners, transform, 0.02 };
  bir::MarkerTransform marker_transform6 = { 6, corners, transform, 0.02 };
  bir::MarkerTransform marker_transform7 = { 7, corners, transform, 0.02 };

  std::vector<bir::MarkerTransform> vecMarkerTransform = { marker_transform0, marker_transform1, marker_transform2,
                                                           marker_transform3, marker_transform4, marker_transform7,
                                                           marker_transform6, marker_transform5 };

  bir::MarkerTransformVector transform_vector1 = vecMarkerTransform;

  EXPECT_EQ(0, transform_vector1.at(0).id);
  EXPECT_EQ(1, transform_vector1.at(1).id);
  EXPECT_EQ(7, transform_vector1.at(5).id);
  EXPECT_NE(2, transform_vector1.at(1).id);
}

TEST(MarkerTransformVector, test_mtvector_hasid)
{
  const auto corners =
      std::vector<cv::Point2f>({ cv::Point2f(4, 4), cv::Point2f(5, 4), cv::Point2f(5, 5), cv::Point2f(4, 5) });
  const auto transform = tf2::Transform(tf2::Quaternion(0, 0, 0, 1), { 0, 0, 0 });

  bir::MarkerTransform marker_transform0 = { 0, corners, transform, 0.02 };
  bir::MarkerTransform marker_transform1 = { 1, corners, transform, 0.02 };
  bir::MarkerTransform marker_transform2 = { 2, corners, transform, 0.02 };
  bir::MarkerTransform marker_transform3 = { 3, corners, transform, 0.02 };
  bir::MarkerTransform marker_transform4 = { 4, corners, transform, 0.02 };
  bir::MarkerTransform marker_transform5 = { 5, corners, transform, 0.02 };
  bir::MarkerTransform marker_transform6 = { 6, corners, transform, 0.02 };
  bir::MarkerTransform marker_transform7 = { 7, corners, transform, 0.02 };

  std::vector<bir::MarkerTransform> vecMarkerTransform = { marker_transform0, marker_transform1, marker_transform2,
                                                           marker_transform3, marker_transform4, marker_transform7,
                                                           marker_transform6, marker_transform5 };

  bir::MarkerTransformVector transform_vector1 = vecMarkerTransform;

  EXPECT_TRUE(transform_vector1.hasID(0));
  EXPECT_FALSE(transform_vector1.hasID(15));
  EXPECT_FALSE(transform_vector1.hasID(8));
  EXPECT_TRUE(transform_vector1.hasID(7));
}

TEST(MarkerTransformVector, test_mtvector_withid)
{
  const auto corners =
      std::vector<cv::Point2f>({ cv::Point2f(4, 4), cv::Point2f(5, 4), cv::Point2f(5, 5), cv::Point2f(4, 5) });
  const auto transform = tf2::Transform(tf2::Quaternion(0, 0, 0, 1), { 0, 0, 0 });

  bir::MarkerTransform marker_transform0 = { 0, corners, transform, 0.02 };
  bir::MarkerTransform marker_transform1 = { 1, corners, transform, 0.02 };
  bir::MarkerTransform marker_transform2 = { 2, corners, transform, 0.02 };
  bir::MarkerTransform marker_transform3 = { 3, corners, transform, 0.02 };
  bir::MarkerTransform marker_transform4 = { 4, corners, transform, 0.02 };
  bir::MarkerTransform marker_transform5 = { 5, corners, transform, 0.02 };
  bir::MarkerTransform marker_transform6 = { 6, corners, transform, 0.02 };
  bir::MarkerTransform marker_transform7 = { 7, corners, transform, 0.02 };

  std::vector<bir::MarkerTransform> vecMarkerTransform = { marker_transform1, marker_transform0, marker_transform3,
                                                           marker_transform2, marker_transform4, marker_transform7,
                                                           marker_transform6, marker_transform5 };

  bir::MarkerTransformVector transform_vector1 = vecMarkerTransform;

  EXPECT_EQ(marker_transform0, transform_vector1.withID(0));
  EXPECT_EQ(marker_transform2, transform_vector1.withID(2));
  EXPECT_EQ(marker_transform7, transform_vector1.withID(7));
  EXPECT_THROW(transform_vector1.withID(12), std::invalid_argument);
}

TEST(MarkerTransformVector, test_mtvector_pushback)
{
  const auto corners =
      std::vector<cv::Point2f>({ cv::Point2f(4, 4), cv::Point2f(5, 4), cv::Point2f(5, 5), cv::Point2f(4, 5) });
  const auto transform = tf2::Transform(tf2::Quaternion(0, 0, 0, 1), { 0, 0, 0 });

  bir::MarkerTransform marker_transform0 = { 0, corners, transform, 0.02 };
  bir::MarkerTransform marker_transform1 = { 1, corners, transform, 0.02 };
  bir::MarkerTransform marker_transform2 = { 2, corners, transform, 0.02 };
  bir::MarkerTransform marker_transform3 = { 3, corners, transform, 0.02 };
  bir::MarkerTransform marker_transform4 = { 4, corners, transform, 0.02 };
  bir::MarkerTransform marker_transform5 = { 5, corners, transform, 0.02 };
  bir::MarkerTransform marker_transform6 = { 6, corners, transform, 0.02 };
  bir::MarkerTransform marker_transform7 = { 7, corners, transform, 0.02 };

  bir::MarkerTransformVector transform_vector1;
  
  EXPECT_TRUE(transform_vector1.isEmpty());
  transform_vector1.pushBack(marker_transform0);
  EXPECT_FALSE(transform_vector1.isEmpty());
  EXPECT_EQ(1, (int)transform_vector1.size());
  transform_vector1.pushBack(marker_transform1);
  transform_vector1.pushBack(marker_transform2);
  transform_vector1.pushBack(marker_transform3);
  EXPECT_EQ(4, (int)transform_vector1.size());
  EXPECT_EQ(marker_transform1, transform_vector1.at(1));
  transform_vector1.pushBack(marker_transform7);
  EXPECT_EQ(marker_transform7, transform_vector1.at(4));

  transform_vector1.clear();

  EXPECT_TRUE(transform_vector1.isEmpty());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
