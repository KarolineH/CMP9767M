#!/usr/bin/env python
import rospy
import numpy as np
import tf
from sensor_msgs.msg import CameraInfo
from rp_pack.msg import PixelArray, Pixel
from geometry_msgs.msg import PoseStamped, PoseArray

class Camera_converter():
    def __init__(self, robot_name):
        rospy.init_node('camera_converter', anonymous = True)
        self.tfl = tf.TransformListener()
        self.robot_id = robot_name
        self.pub = rospy.Publisher("/{}/weed_poses".format(self.robot_id), PoseArray, queue_size=5)

        camera_info = rospy.wait_for_message("/{}/kinect2_camera/hd/camera_info".format(self.robot_id), CameraInfo)
        self.rate = rospy.Rate(10.0)
        self.rate.sleep()
        initial_cam_pose = self.tfl.lookupTransform("/map","{}/kinect2_rgb_optical_frame".format(self.robot_id), rospy.Time())
        self.camera_height = initial_cam_pose[0][2] # Assuming that the camera is parallel to the ground
        self.inv_camera_matrix = np.linalg.inv(np.reshape(camera_info.K, [3,3]))
        self.sub = rospy.Subscriber("/{}/weed_pixels".format(self.robot_id), PixelArray, self.weed_callback)
        rospy.spin()

    def weed_callback(self, data):
        pixels = data.array
        world_coordinates = PoseArray()
        world_coordinates.header.frame_id = "/map"
        array = []
        for pixel in pixels:
            p = np.array([pixel.coordinates[0],pixel.coordinates[1],1])
            camera_projection = np.dot(self.inv_camera_matrix,p) # a point on the line to the ground in camera coordinates
            pose = PoseStamped()
            pose.header.frame_id = "{}/kinect2_rgb_optical_frame".format(self.robot_id)
            pose.pose.position.x = self.camera_height * camera_projection[0]
            pose.pose.position.y = self.camera_height * camera_projection[1]
            pose.pose.position.z = self.camera_height
            world_projection = self.tfl.transformPose("/map", pose)
            array.append(world_projection.pose)
        world_coordinates.poses = array
        self.pub.publish(world_coordinates)

if __name__ == "__main__":
    cc = Camera_converter('thorvald_001')
