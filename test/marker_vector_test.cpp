#include <iostream>
#include <string>
#include <vector>
#include <utility>

#include <gtest/gtest.h>

#include <marker_localization/marker.h>

TEST(MarkerTest, test_marker_area)
{
  auto marker_16 = bir::Marker(
      1, std::vector<cv::Point2f>({ cv::Point2f(-2, -2), cv::Point2f(-2, 2), cv::Point2f(2, 2), cv::Point2f(2, -2) }));

  EXPECT_NEAR(16.0, marker_16.area(), 0.001);

  auto marker_1 = bir::Marker(
      1, std::vector<cv::Point2f>({ cv::Point2f(4, 4), cv::Point2f(5, 4), cv::Point2f(5, 5), cv::Point2f(4, 5) }));

  EXPECT_NEAR(1.0, marker_1.area(), 0.001);
}

TEST(MarkerTest, test_marker_equal_id)
{
  auto marker_1 = bir::Marker(
      1, std::vector<cv::Point2f>({ cv::Point2f(4, 4), cv::Point2f(5, 4), cv::Point2f(5, 5), cv::Point2f(4, 5) }));
  EXPECT_TRUE((marker_1 == 1));
  EXPECT_FALSE((marker_1 == 7));

  auto marker_47 = bir::Marker(
      47, std::vector<cv::Point2f>({ cv::Point2f(4, 4), cv::Point2f(5, 4), cv::Point2f(5, 5), cv::Point2f(4, 5) }));
  EXPECT_TRUE((marker_47 == 47));
  EXPECT_FALSE((marker_47 == 39));
}

TEST(MarkerTest, test_marker_equal_marker)
{
  auto marker_1 = bir::Marker(
      1, std::vector<cv::Point2f>({ cv::Point2f(4, 4), cv::Point2f(5, 4), cv::Point2f(5, 5), cv::Point2f(4, 5) }));

  auto marker_1_2 = bir::Marker(
      1, std::vector<cv::Point2f>({ cv::Point2f(4, 4), cv::Point2f(5, 4), cv::Point2f(5, 5), cv::Point2f(4, 5) }));

  auto marker_1_3 = bir::Marker(
      1, std::vector<cv::Point2f>({ cv::Point2f(3, 4), cv::Point2f(5, 4), cv::Point2f(5, 5), cv::Point2f(4, 5) }));

  auto marker_47_1 = bir::Marker(
      47, std::vector<cv::Point2f>({ cv::Point2f(4, 4), cv::Point2f(5, 4), cv::Point2f(5, 5), cv::Point2f(4, 5) }));
  EXPECT_TRUE((marker_1 == marker_1_2));
  EXPECT_FALSE((marker_1 == marker_1_3));
  EXPECT_FALSE((marker_1 == marker_47_1));
  EXPECT_FALSE((marker_1_3 == marker_47_1));
}

TEST(MarkerVectorTest, test_marker_vector_size)
{
  auto marker_1 = bir::Marker(
      1, std::vector<cv::Point2f>({ cv::Point2f(4, 4), cv::Point2f(5, 4), cv::Point2f(5, 5), cv::Point2f(4, 5) }));

  auto marker_1_2 = bir::Marker(
      1, std::vector<cv::Point2f>({ cv::Point2f(4, 4), cv::Point2f(5, 4), cv::Point2f(5, 5), cv::Point2f(4, 5) }));

  auto marker_1_3 = bir::Marker(
      1, std::vector<cv::Point2f>({ cv::Point2f(3, 4), cv::Point2f(5, 4), cv::Point2f(5, 5), cv::Point2f(4, 5) }));

  auto marker_47_1 = bir::Marker(
      47, std::vector<cv::Point2f>({ cv::Point2f(4, 4), cv::Point2f(5, 4), cv::Point2f(5, 5), cv::Point2f(4, 5) }));

  auto marker_vector_4 = bir::MarkerVector(std::vector<bir::Marker>({ marker_1, marker_1_2, marker_1_3, marker_47_1 }));
  auto marker_vector_3 = bir::MarkerVector(std::vector<bir::Marker>({ marker_1, marker_1_2, marker_1_3 }));
  auto marker_vector_2 = bir::MarkerVector(std::vector<bir::Marker>({ marker_1, marker_1_2 }));
  auto marker_vector_1 = bir::MarkerVector(std::vector<bir::Marker>({ marker_1 }));

  EXPECT_EQ(4, marker_vector_4.size());
  EXPECT_EQ(3, marker_vector_3.size());
  EXPECT_EQ(2, marker_vector_2.size());
  EXPECT_EQ(1, marker_vector_1.size());
}

TEST(MarkerVectorTest, test_marker_vector_empty_clear)
{
  auto marker_1 = bir::Marker(
      1, std::vector<cv::Point2f>({ cv::Point2f(4, 4), cv::Point2f(5, 4), cv::Point2f(5, 5), cv::Point2f(4, 5) }));

  auto marker_1_2 = bir::Marker(
      1, std::vector<cv::Point2f>({ cv::Point2f(4, 4), cv::Point2f(5, 4), cv::Point2f(5, 5), cv::Point2f(4, 5) }));

  auto marker_1_3 = bir::Marker(
      1, std::vector<cv::Point2f>({ cv::Point2f(3, 4), cv::Point2f(5, 4), cv::Point2f(5, 5), cv::Point2f(4, 5) }));

  auto marker_vector_not_empty = bir::MarkerVector(std::vector<bir::Marker>({ marker_1, marker_1_2, marker_1_3 }));
  auto marker_vector_empty = bir::MarkerVector(std::vector<bir::Marker>({}));

  EXPECT_FALSE(marker_vector_not_empty.isEmpty());
  EXPECT_TRUE(marker_vector_empty.isEmpty());

  marker_vector_not_empty.clear();

  EXPECT_TRUE(marker_vector_not_empty.isEmpty());
}

TEST(MarkerVectorTest, test_marker_vector_at)
{
  auto marker_1 = bir::Marker(
      1, std::vector<cv::Point2f>({ cv::Point2f(4, 4), cv::Point2f(5, 4), cv::Point2f(5, 5), cv::Point2f(4, 5) }));

  auto marker_1_2 = bir::Marker(
      1, std::vector<cv::Point2f>({ cv::Point2f(4, 4), cv::Point2f(5, 4), cv::Point2f(5, 5), cv::Point2f(4, 5) }));

  auto marker_1_3 = bir::Marker(
      1, std::vector<cv::Point2f>({ cv::Point2f(3, 4), cv::Point2f(5, 4), cv::Point2f(5, 5), cv::Point2f(4, 5) }));

  auto marker_vector = bir::MarkerVector(std::vector<bir::Marker>({ marker_1, marker_1_2, marker_1_3 }));

  EXPECT_TRUE(marker_vector.at(1) == marker_1_2);
  EXPECT_FALSE(marker_vector.at(2) == marker_1);
  EXPECT_NEAR(1.0, marker_vector.at(0).area(), 0.001);
}

TEST(MarkerVectorTest, test_marker_vector_hasid)
{
  auto marker_1 = bir::Marker(
      1, std::vector<cv::Point2f>({ cv::Point2f(4, 4), cv::Point2f(5, 4), cv::Point2f(5, 5), cv::Point2f(4, 5) }));

  auto marker_54_1 = bir::Marker(
      54, std::vector<cv::Point2f>({ cv::Point2f(4, 4), cv::Point2f(5, 4), cv::Point2f(5, 5), cv::Point2f(4, 5) }));

  auto marker_36_1 = bir::Marker(
      36, std::vector<cv::Point2f>({ cv::Point2f(3, 4), cv::Point2f(5, 4), cv::Point2f(5, 5), cv::Point2f(4, 5) }));

  bir::MarkerVector marker_vector = std::vector<bir::Marker>({ marker_1, marker_54_1, marker_36_1 });

  EXPECT_TRUE(marker_vector.hasID(1));
  EXPECT_TRUE(marker_vector.hasID(54));
  EXPECT_TRUE(marker_vector.hasID(36));

  EXPECT_FALSE(marker_vector.hasID(42));
  EXPECT_FALSE(marker_vector.hasID(31));
  EXPECT_FALSE(marker_vector.hasID(10));
}

TEST(MarkerVectorTest, test_marker_vector_withid)
{
  auto marker_1 = bir::Marker(
      1, std::vector<cv::Point2f>({ cv::Point2f(4, 4), cv::Point2f(5, 4), cv::Point2f(5, 5), cv::Point2f(4, 5) }));

  auto marker_54 = bir::Marker(
      54, std::vector<cv::Point2f>({ cv::Point2f(4, 4), cv::Point2f(5, 4), cv::Point2f(5, 5), cv::Point2f(4, 5) }));

  auto marker_36 = bir::Marker(
      36, std::vector<cv::Point2f>({ cv::Point2f(3, 4), cv::Point2f(5, 4), cv::Point2f(5, 5), cv::Point2f(3, 5) }));

  bir::MarkerVector marker_vector = std::vector<bir::Marker>({ marker_1, marker_54, marker_36 });

  EXPECT_TRUE(marker_1 == marker_vector.withID(1));
  EXPECT_TRUE(marker_54 == marker_vector.withID(54));
  EXPECT_TRUE(marker_36 == marker_vector.withID(36));
  EXPECT_NEAR(2.0, marker_vector.withID(36).area(), 0.001);
  EXPECT_NEAR(1.0, marker_vector.withID(1).area(), 0.001);
  EXPECT_THROW(marker_vector.withID(27), std::invalid_argument);
}

TEST(MarkerVectorTest, test_marker_vector_pushback)
{
  auto marker_1 = bir::Marker(
      1, std::vector<cv::Point2f>({ cv::Point2f(4, 4), cv::Point2f(5, 4), cv::Point2f(5, 5), cv::Point2f(4, 5) }));

  auto marker_54 = bir::Marker(
      54, std::vector<cv::Point2f>({ cv::Point2f(4, 4), cv::Point2f(5, 4), cv::Point2f(5, 5), cv::Point2f(4, 5) }));

  auto marker_36 = bir::Marker(
      36, std::vector<cv::Point2f>({ cv::Point2f(3, 4), cv::Point2f(5, 4), cv::Point2f(5, 5), cv::Point2f(3, 5) }));

  bir::MarkerVector marker_vector = std::vector<bir::Marker>({});

  EXPECT_TRUE(marker_vector.isEmpty());

  marker_vector.pushBack(marker_1);

  EXPECT_FALSE(marker_vector.isEmpty());
  EXPECT_TRUE(marker_vector.hasID(1));
  EXPECT_FALSE(marker_vector.hasID(54));

  marker_vector.pushBack(marker_54);

  EXPECT_FALSE(marker_vector.isEmpty());
  EXPECT_TRUE(marker_vector.hasID(54));

  marker_vector.clear();

  EXPECT_TRUE(marker_vector.isEmpty());

  marker_vector.pushBack(marker_36);

  EXPECT_FALSE(marker_vector.hasID(54));
  EXPECT_TRUE(marker_vector.hasID(36));
}

TEST(MarkerVectorTest, test_marker_vector_copy_assign)
{
  auto marker_1 = bir::Marker(
      1, std::vector<cv::Point2f>({ cv::Point2f(4, 4), cv::Point2f(5, 4), cv::Point2f(5, 5), cv::Point2f(4, 5) }));

  auto marker_54 = bir::Marker(
      54, std::vector<cv::Point2f>({ cv::Point2f(4, 4), cv::Point2f(5, 4), cv::Point2f(5, 5), cv::Point2f(4, 5) }));

  auto marker_36 = bir::Marker(
      36, std::vector<cv::Point2f>({ cv::Point2f(3, 4), cv::Point2f(5, 4), cv::Point2f(5, 5), cv::Point2f(3, 5) }));

  bir::MarkerVector marker_vector_1 = std::vector<bir::Marker>({ marker_1, marker_54, marker_36 });
  bir::MarkerVector marker_vector_2 = std::vector<bir::Marker>();

  EXPECT_FALSE(marker_vector_1.isEmpty());
  EXPECT_TRUE(marker_vector_2.isEmpty());

  marker_vector_2 = marker_vector_1;

  EXPECT_FALSE(marker_vector_1.isEmpty());
  EXPECT_FALSE(marker_vector_2.isEmpty());

  EXPECT_TRUE(marker_vector_1.hasID(36));
  EXPECT_TRUE(marker_vector_2.hasID(36));
}

TEST(MarkerVectorTest, test_marker_vector_move_assign)
{
  auto marker_1 = bir::Marker(
      1, std::vector<cv::Point2f>({ cv::Point2f(4, 4), cv::Point2f(5, 4), cv::Point2f(5, 5), cv::Point2f(4, 5) }));

  auto marker_54 = bir::Marker(
      54, std::vector<cv::Point2f>({ cv::Point2f(4, 4), cv::Point2f(5, 4), cv::Point2f(5, 5), cv::Point2f(4, 5) }));

  auto marker_36 = bir::Marker(
      36, std::vector<cv::Point2f>({ cv::Point2f(3, 4), cv::Point2f(5, 4), cv::Point2f(5, 5), cv::Point2f(3, 5) }));

  bir::MarkerVector marker_vector_1 = std::vector<bir::Marker>({ marker_1, marker_54, marker_36 });
  bir::MarkerVector marker_vector_2 = std::vector<bir::Marker>();

  EXPECT_FALSE(marker_vector_1.isEmpty());
  EXPECT_TRUE(marker_vector_2.isEmpty());

  marker_vector_2 = std::move(marker_vector_1);

  EXPECT_TRUE(marker_vector_1.isEmpty());
  EXPECT_FALSE(marker_vector_2.isEmpty());

  EXPECT_FALSE(marker_vector_1.hasID(36));
  EXPECT_TRUE(marker_vector_2.hasID(36));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
