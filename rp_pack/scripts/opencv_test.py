#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class seeker:
    def __init__(self):
        blink_rate = 0
    	#Interface OpenCV from ROS
        self.bridge = CvBridge()
        #Subscribe to the robot camera image stream
        self.image_sub = rospy.Subscriber("/thorvald_001/kinect2_camera/hd/image_color_rect", Image, self.callback)

    def callback(self, data):
        self.latest_image = data

    def examine_image():
        image = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8") #deserialize using opencv
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_RGB2HSV) #Transfer into HSV space
        mask = cv2.inRange(hsv_img,(36, 0, 0),(86, 255,255)) # filter everything but green pixels
        masked_img = cv2.bitwise_and(cv_image, cv_image, mask = mask) # apply the mask to the image

if __name__ == "__main__":
	rospy.init_node('Weed_seeker')
	harry = seeker()
    while true:
        harry.examine_image()
	rospy.spin()


	destroyAllWindows()
