#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def main():
    rospy.init_node('image_publisher')
    image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=1)

    rate = rospy.Rate(30)
    
    bridge = CvBridge()
    file_path = '/home/teo/Documents/mkr_localization_ws/src/bir_marker_localization/test/resource/resource_1_aruco_board_6x6.jpg'
    cv_image = cv2.imread(file_path)
    ros_image = bridge.cv2_to_imgmsg(cv_image, "bgr8")

    while not rospy.is_shutdown():
        image_pub.publish(ros_image)
        rate.sleep()


if __name__ == "__main__":
    main()