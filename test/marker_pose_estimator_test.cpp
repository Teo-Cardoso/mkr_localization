#include <gtest/gtest.h>

#include <vector>

#include <opencv4/opencv2/opencv.hpp>

#include <marker_localization/marker.h>
#include <marker_localization/marker_transform.h>
#include <marker_localization/marker_pose_estimator.h>

TEST(MarkerPoseEstimatorTest, test_estimatepose)
{
  const std::vector<double> cameraMatrixValuesVector = { 100.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 1.0 };
  const std::vector<double> distortionCoefVector = { 0.0, 0.0, 0.0, 0.0, 0.0 };
  const std::vector<std::pair<int, std::vector<int>>> expectIds = { std::make_pair(100, std::vector<int>({ 40 })) };

  cv::Mat cameraMatrix = (cv::Mat1d(3, 3) << cameraMatrixValuesVector[0], 0, cameraMatrixValuesVector[2], 0,
                          cameraMatrixValuesVector[4], cameraMatrixValuesVector[5], 0, 0, 1);

  cv::Mat distCoeffs = (cv::Mat1d(1, 5) << distortionCoefVector[0], distortionCoefVector[1], distortionCoefVector[2],
                        distortionCoefVector[3], distortionCoefVector[4]);

  const std::vector<cv::Point2f> corners = {
    { 358.998, 309.340 }, { 404.177, 310.019 }, { 409.807, 350.777 }, { 361.701, 350.460 }
  };

  auto marker40 = bir::Marker(40, corners);
  auto marker41 = bir::Marker(41, corners);
  bir::MarkerVector markerVector = std::vector<bir::Marker>({ marker40, marker41 });

  bir::MarkerPoseEstimator poseEstimator(expectIds, cameraMatrix, distCoeffs);
  bir::MarkerTransformVector resultVector = poseEstimator.estimatePose(markerVector);

  EXPECT_FALSE(resultVector.isEmpty());
  EXPECT_EQ(1, (int)resultVector.size());
  EXPECT_TRUE(resultVector.hasID(40));
  EXPECT_FALSE(resultVector.hasID(41));

  bir::MarkerTransform result = resultVector.withID(40);

  EXPECT_NEAR(0.9297, result.transform.getOrigin().x(), 0.001);
  EXPECT_NEAR(0.8003, result.transform.getOrigin().y(), 0.001);
  EXPECT_NEAR(0.2424, result.transform.getOrigin().z(), 0.001);

  EXPECT_NEAR(-0.001, result.transform.getRotation().w(), 0.001);
  EXPECT_NEAR(0.9985, result.transform.getRotation().x(), 0.001);
  EXPECT_NEAR(-0.0518, result.transform.getRotation().y(), 0.001);
  EXPECT_NEAR(-0.0176, result.transform.getRotation().z(), 0.001);
}

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}