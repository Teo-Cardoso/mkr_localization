#include <iostream>
#include <string>
#include <vector>
#include <utility>

#include <gtest/gtest.h>

#include <marker_localization/marker.h>
#include <marker_localization/marker_detect.h>

#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/aruco.hpp>

std::string RESOURCE_DIR;

TEST(MarkerDetectTest, board_0_34_6x6_test)
{
  std::string file_path = RESOURCE_DIR + std::string("resource_1_aruco_board_6x6.jpg");
  cv::Mat image = cv::imread(file_path);
  bir::MarkerDetect detector(cv::aruco::DICT_6X6_1000);
  auto marker_vector = detector.detect(image);

  EXPECT_EQ(35, marker_vector.size());

  for (int id = 0; id < 35; id++)
  {
    EXPECT_EQ(true, marker_vector.hasID(id));
    EXPECT_EQ(false, marker_vector.hasID(35 + id));
  }

  for (int id = 0; id < 35; id++)
  {
    EXPECT_EQ(id, marker_vector.withID(id).id);
  }
}

TEST(MarkerDetectTest, markers_printed_6x6_test)
{
  std::string file_path = RESOURCE_DIR + std::string("resource_2_aruco_markers_printed_6x6.jpg");
  cv::Mat image = cv::imread(file_path);

  bir::MarkerDetect detector(cv::aruco::DICT_6X6_1000);
  auto marker_vector = detector.detect(image);
  
  std::vector<int> expected_ids = { 23, 40, 62, 98, 124, 203 };
  std::vector<int> not_expected_ids = { 1, 20, 24, 32, 54, 75, 980 };

  EXPECT_EQ(6, marker_vector.size());

  for (int index = 0; index < expected_ids.size(); index++)
  {
    EXPECT_TRUE(marker_vector.hasID(expected_ids.at(index)));
  }

  for (int index = 0; index < not_expected_ids.size(); index++)
  {
    EXPECT_FALSE(marker_vector.hasID(not_expected_ids.at(index)));
  }

  bir::Marker marker_40 = marker_vector.withID(40);
  EXPECT_NEAR(358.998, marker_40.corner.at(0).x, 0.001);
  EXPECT_NEAR(309.340, marker_40.corner.at(0).y, 0.001);
  EXPECT_NEAR(404.177, marker_40.corner.at(1).x, 0.001);
  EXPECT_NEAR(310.019, marker_40.corner.at(1).y, 0.001);
  EXPECT_NEAR(409.807, marker_40.corner.at(2).x, 0.001);
  EXPECT_NEAR(350.777, marker_40.corner.at(2).y, 0.001);
  EXPECT_NEAR(361.701, marker_40.corner.at(3).x, 0.001);
  EXPECT_NEAR(350.460, marker_40.corner.at(3).y, 0.001);

  bir::Marker marker_203 = marker_vector.withID(203);
  EXPECT_NEAR(195.202, marker_203.corner.at(0).x, 0.001);
  EXPECT_NEAR(154.422, marker_203.corner.at(0).y, 0.001);
  EXPECT_NEAR(229.843, marker_203.corner.at(1).x, 0.001);
  EXPECT_NEAR(155.568, marker_203.corner.at(1).y, 0.001);
  EXPECT_NEAR(226.706, marker_203.corner.at(2).x, 0.001);
  EXPECT_NEAR(178.677, marker_203.corner.at(2).y, 0.001);
  EXPECT_NEAR(189.896, marker_203.corner.at(3).x, 0.001);
  EXPECT_NEAR(178.294, marker_203.corner.at(3).y, 0.001);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  RESOURCE_DIR = TEST_RESOURCE_DIR;
  return RUN_ALL_TESTS();
}
