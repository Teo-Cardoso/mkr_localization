#include <memory> // std::unique_ptr<T>

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Transform.h>
#include <sensor_msgs/Image.h>

#include <marker_localization/free_markers_localization.hpp>
#include <marker_localization/MarkerPoseArray.h>

class FreeMarkerLocalizationTest : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    transforms_received = false;
    node_.getParam("resource_path", resource_path);
    imgTransport_ = std::unique_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(node_));
    imagePublisher_ = imgTransport_->advertise("/camera/image_raw", 1);
    markersTransformsSubscriber_ =
        node_.subscribe("/detected_markers/pose", 10, &FreeMarkerLocalizationTest::markerTransformCb, this);
  }

  virtual void TearDown()
  {
  }

  void markerTransformCb(const marker_localization::MarkerPoseArray msg)
  {
    if (!transforms_received)
    {
      transforms_received = true;
      markersPoses_ = msg;
    }
  }

  ros::NodeHandle node_;
  std::unique_ptr<image_transport::ImageTransport> imgTransport_;
  std::string resource_path;
  // Subscriber / Publisher Testing
  image_transport::Publisher imagePublisher_;
  ros::Subscriber markersTransformsSubscriber_;
  bool transforms_received;

  // Pose Testing
  marker_localization::MarkerPoseArray markersPoses_;

  cv_bridge::CvImagePtr cvBridge;
};

TEST_F(FreeMarkerLocalizationTest, test_pub_sub_test)
{
  ros::Rate pub_rate(30);
  const std::string file_path = resource_path + std::string("resource_1_aruco_board_6x6.jpg");

  sensor_msgs::ImagePtr ros_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv::imread(file_path)).toImageMsg();

  EXPECT_FALSE(transforms_received);

  for (int frames = 0; ros::ok() && !transforms_received; frames++)
  {
    ros::spinOnce();

    if (frames == 15)
      FAIL() << "Transforms not received after 15 frames.";

    imagePublisher_.publish(ros_image);
    pub_rate.sleep();
  }

  EXPECT_TRUE(transforms_received);
  EXPECT_EQ(14, (int)markersPoses_.markers.size());
  
  const auto& marker_18_reference = markersPoses_.markers.back();
  EXPECT_EQ(18, marker_18_reference.marker_id);
  EXPECT_NEAR(0.3413941725982918, marker_18_reference.marker_pose.translation.x, 0.001);
  EXPECT_NEAR(0.7819027142979985, marker_18_reference.marker_pose.translation.y, 0.001);
  EXPECT_NEAR(1.1097906464981684, marker_18_reference.marker_pose.translation.z, 0.001);
  EXPECT_NEAR(0.000, marker_18_reference.marker_pose.rotation.x, 0.001);
  EXPECT_NEAR(1.000, marker_18_reference.marker_pose.rotation.y, 0.001);
  EXPECT_NEAR(0.000, marker_18_reference.marker_pose.rotation.z, 0.001);
  EXPECT_NEAR(0.000, marker_18_reference.marker_pose.rotation.w, 0.001);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "free_markers_localization_test");
  return RUN_ALL_TESTS();
}
