#!/usr/bin/env python
import rospy
import rospkg
import cv2
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import scipy.misc
import pdb
#from sklearn.linear_model import LinearRegression
from sklearn.linear_model import LogisticRegression

class plant_discriminator():
    def __init__(self):
        self.include_harder_plants = False # trains the regression for discrimination on both the first row, and the slightly harder second row
        self.blink_rate = 5 # (Hz) Won't look at every single image published by the camera, but use this rate instead
        self.pub = rospy.Publisher('/weed_filter_image', Image, queue_size=5)

        # Initiate node
        rospy.init_node('plant_discriminator')
        # Initiate needed objects
        self.bridge = CvBridge()
        rospack = rospkg.RosPack()

        # Where to find the labeled training data:
        self.raw_image_r1 = rospack.get_path('rp_pack')+'/Supporting_Files/row1_training_image.png'
        self.labeled_image_r1 = rospack.get_path('rp_pack')+'/Supporting_Files/row1_filtered.png'
        self.raw_image_r2 = rospack.get_path('rp_pack')+'/Supporting_Files/row2_training_image.png'
        self.labeled_image_r2 = rospack.get_path('rp_pack')+'/Supporting_Files/row2_filtered.png'

        # Fetch camera infooutput
        camera_info = rospy.wait_for_message("/thorvald_001/kinect2_camera/hd/camera_info", CameraInfo)
        self.image_size = (camera_info.height, camera_info.width)
        self.image_channels = 3 #3 channels, for RGB image
        # Train logsitic regression
        self.train_regression()
        while not rospy.is_shutdown():
            latest_image = rospy.wait_for_message("/thorvald_001/kinect2_camera/hd/image_color_rect", Image)
            (x,y) = self.predict(latest_image)
            image_message = self.bridge.cv2_to_imgmsg(y, encoding="mono8")
            self.pub.publish(image_message)

    def display_single_image(self):
        """Fetch and display the latest image"""

        latest_image = rospy.wait_for_message("/thorvald_001/kinect2_camera/hd/image_color_rect", Image)
        cv_image = self.bridge.imgmsg_to_cv2(latest_image, "bgr8") #deserialize using opencv
        cv2.imshow('image',cv_image)
        cv2.waitKey(0)

    def train_regression(self):
        """Train the logistic regression using manually labeled images"""

        # Import and reshape the manually labeled training image
        raw_image_row2 = scipy.misc.imread(self.raw_image_r2)
        labels_row2 = scipy.misc.imread(self.labeled_image_r2)
        if self.include_harder_plants:
            raw_image_row1 = scipy.misc.imread(self.raw_image_r1)
            full_image = np.append(raw_image_row1,raw_image_row2,axis = 0)
            labels_row1 = scipy.misc.imread(self.labeled_image_r1)
            full_labels = np.append(labels_row1,labels_row2,axis = 0)
        else:
            full_image = raw_image_row2
            full_labels = labels_row2
        masked_gray_image = cv2.cvtColor(full_labels, cv2.COLOR_BGR2GRAY)
        (thresh, bw_image) = cv2.threshold(masked_gray_image, 127, 255, cv2.THRESH_BINARY)
        # Reshape into one long list of training examples (single pixels)
        attribs = np.reshape(full_image, (full_image.shape[0]*full_image.shape[1],self.image_channels))
        labels = np.reshape(bw_image,(bw_image.shape[0]*bw_image.shape[1],1))
        # Train regression
        ## taken from https://towardsdatascience.com/a-beginners-guide-to-linear-regression-in-python-with-scikit-learn-83a8f7ae2b4f
        self.regressor = LogisticRegression(solver='lbfgs')
        self.regressor.fit(attribs, labels.ravel())

    def predict(self, img):
        """Predict which pixels of an input image are part of a weed"""
        cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8") # deserialize
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB) # opencv uses BGR, but the regression is performed on RGB images
        inputs = np.reshape(rgb_image, (self.image_size[0]*self.image_size[1],self.image_channels))
        outputs = self.regressor.predict(inputs)
        predicted_image = np.reshape(outputs, (self.image_size[0],self.image_size[1])) # shape the outputs back into an image matrix
        return cv_image, predicted_image

    def view_prediction(self,x,y):
        """Show an input image and a prediction side by side"""
        cv2.imshow('input',x)
        cv2.imshow('prediction',y)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

if __name__ == "__main__":
    pd = plant_discriminator()
