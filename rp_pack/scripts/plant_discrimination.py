#!/usr/bin/env python
import rospy
import rospkg
import cv2
import numpy as np
import matplotlib.pyplot as plt
import sys
from sensor_msgs.msg import Image, CameraInfo
from rp_pack.msg import PixelArray, Pixel
from cv_bridge import CvBridge
import scipy.misc
from sklearn.linear_model import LogisticRegression

class Plant_discriminator():

    """
    Subscribes to the camera stream
    Classifies each RGB pixel as weed or no weed (using logistic regression fitted to training images)
    Detects connected components (clusters of weed pixels) of a certain size
    Finds the centroids of the detected clusters
    Publishes those centroid pixel positions to /weed_pixels
    """

    def __init__(self, robot_name):
        """Set-up routine"""
        rospy.init_node('plant_discriminator', anonymous=True)
        self.include_harder_plants = True # trains the regression for discrimination on both the first row, and the slightly harder second row
        self.blink_rate = rospy.Rate(5) # (Hz) Won't look at every single image published by the camera, but use this rate instead
        self.min_component_size = 250 # minimum number of connected pixels that are considered a weed
        self.max_component_size = 100000 # maximum number of connected pixels that are considered a single weed
        self.robot_id= robot_name

        # Initiate required components
        self.filter_pub = rospy.Publisher("/{}/weed_filter_image".format(self.robot_id), Image, queue_size=5)
        self.pixel_pub = rospy.Publisher("/{}/weed_pixels".format(self.robot_id), PixelArray, queue_size =5)
        self.image_pub = rospy.Publisher("/{}/weed_detection_image".format(self.robot_id), Image, queue_size=5)
        self.bridge = CvBridge()
        rospack = rospkg.RosPack()

        # Where to find the labeled training data:
        self.raw_image_r1 = rospack.get_path('rp_pack')+'/Supporting_Files/row1_training_image.png'
        self.labeled_image_r1 = rospack.get_path('rp_pack')+'/Supporting_Files/row1_filtered.png'
        self.raw_image_r2 = rospack.get_path('rp_pack')+'/Supporting_Files/row2_training_image.png'
        self.labeled_image_r2 = rospack.get_path('rp_pack')+'/Supporting_Files/row2_filtered.png'
        # Fetch camera info
        camera_info = rospy.wait_for_message("/{}/kinect2_camera/hd/camera_info".format(self.robot_id), CameraInfo)
        self.image_size = (camera_info.height, camera_info.width)
        self.image_channels = 3 #3 channels, for RGB image

        # Train logsitic regression
        self.train_regression()

        """Launch run-time routine"""
        while not rospy.is_shutdown():
            self.runtime_routine()

    def runtime_routine(self):
        latest_image = rospy.wait_for_message("/{}/kinect2_camera/hd/image_color_rect".format(self.robot_id), Image, timeout = 30)
        (x,y) = self.predict(latest_image)
        filter_message = self.bridge.cv2_to_imgmsg(y, encoding="mono8")
        self.filter_pub.publish(filter_message)
        weed_pixels = PixelArray()
        weed_pixels.array = self.component_detection(x,y)
        self.pixel_pub.publish(weed_pixels)
        self.blink_rate.sleep()

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
        (thresh, bw_image) = cv2.threshold(masked_gray_image, 127, 255, cv2.THRESH_BINARY) # threshold in the middle, to get binary labels
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
        (thresh, binary_predicted_image) = cv2.threshold(predicted_image, 127, 255, cv2.THRESH_BINARY) # threshold in case the predictions are not binary
        return cv_image, binary_predicted_image

    def component_detection(self, original, image):
        output = cv2.connectedComponentsWithStats(image, 8, cv2.CV_32S)
        # OUTPUT = [number of components, label matrix, stat matrix, centroid matrix]
        centroids = np.rint(output[3])
        centroid_pixels = centroids.astype(int)
        weed_pixels = []
        for component in range(0,output[0]):
            if self.max_component_size >= output[2][component,4] >= self.min_component_size:
                new_pixel = Pixel()
                new_pixel.coordinates = [centroid_pixels[component][0], centroid_pixels[component][1]]
                weed_pixels.append(new_pixel)
                cv2.circle(original, (centroid_pixels[component][0], centroid_pixels[component][1]), 5, (0, 0, 255), -1)
        image_message = self.bridge.cv2_to_imgmsg(original, encoding="bgr8")
        self.image_pub.publish(image_message)
        return weed_pixels


    """
    The following methods are useful for debugging
    and extracting visualisations of the detection process
    """

    def view_prediction(self,x,y):
        """Show an input image and a prediction side by side"""
        cv2.imshow('input',x)
        cv2.imshow('prediction',y)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def display_single_image(self):
        """Fetch and display the latest image"""
        latest_image = rospy.wait_for_message("/{}/kinect2_camera/hd/image_color_rect".format(self.robot_id), Image)
        cv_image = self.bridge.imgmsg_to_cv2(latest_image, "bgr8") #deserialize using opencv
        cv2.imshow('image',cv_image)
        cv2.waitKey(0)

    def display_components(self, original, image):
        output = cv2.connectedComponentsWithStats(image, 8, cv2.CV_32S)
        # OUTPUT = [number of components, label matrix, stat matrix, centroid matrix]
        centroids = np.rint(output[3])
        centroid_pixels = centroids.astype(int)
        for component in range(0,output[0]):
            if self.max_component_size >= output[2][component,4] >= self.min_component_size:
                cv2.circle(original, (centroid_pixels[component][0], centroid_pixels[component][1]), 5, (0, 0, 255), -1)
        cv2.imshow('image',original)
        cv2.imshow('mask', image)
        cv2.waitKey(0)

if __name__ == "__main__":
    pd = Plant_discriminator(sys.argv[1])
