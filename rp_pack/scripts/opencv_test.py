#!/usr/bin/env python

import rospy
import cv2
import matplotlib.pyplot as plt
from numpy import mean
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class image_converter:

    def __init__(self):
    	#Interface OpenCV from ROS
        self.bridge = CvBridge()
        #Subscribe to the robot camera image stream
        self.image_sub = rospy.Subscriber("/thorvald_001/kinect2_camera/hd/image_color_rect", Image, self.callback)

    def callback(self, data):
        cv2.namedWindow("Image window")
        cv2.namedWindow("filtered")
        #Transfer into HSV space
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv_img,(36, 0, 0),(86, 255,255))
        masked_img = cv2.bitwise_and(cv_image, cv_image, mask = mask)
        cv2.imshow("filtered", masked_img)
        cv2.waitKey(0)

if __name__ == "__main__":
	rospy.init_node('image_converter')
	ic = image_converter()
	rospy.spin()

	destroyAllWindows()
