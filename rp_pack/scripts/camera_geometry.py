#!/usr/bin/env python
import rospy
import numpy as np
import tf
import sys
from sensor_msgs.msg import CameraInfo, PointCloud
from rp_pack.msg import PixelArray, Pixel
from geometry_msgs.msg import PoseStamped, PoseArray, PointStamped

class Camera_converter():

    """
    Subscribes to the pixel positions of weeds (/weed_pixels)
    Also initialises and defines details about the camera setup (e.g. calculates the field of view parameter)
    For every new incoming set of weed pixels, translates them into world coordinates (PointCloud)
    and publishes them to /local_weed_poses
    """

    def __init__(self, robot_name, role):
        rospy.init_node('camera_converter', anonymous = True)
        self.robot_id = robot_name

        # safely fetch camera transform (see "time travel with tf")
        self.tfl = tf.TransformListener()
        now = rospy.Time(0)
        self.tfl.waitForTransform("/map","/{}/kinect2_rgb_optical_frame".format(self.robot_id),now, rospy.Duration(10.0))
        initial_cam_pose = self.tfl.lookupTransform("/map","/{}/kinect2_rgb_optical_frame".format(self.robot_id), now)
        self.camera_height = initial_cam_pose[0][2] # Assuming that the camera is parallel to the ground

        # and camera info
        camera_info = rospy.wait_for_message("/{}/kinect2_camera/hd/camera_info".format(self.robot_id), CameraInfo)
        self.inv_camera_matrix = np.linalg.inv(np.reshape(camera_info.K, [3,3]))
        self.define_fieldOfView_footprint(camera_info)

        self.sub = rospy.Subscriber("/{}/weed_pixels".format(self.robot_id), PixelArray, self.weed_callback)
        self.pub = rospy.Publisher("/{}/local_weed_poses".format(self.robot_id), PointCloud, queue_size=5)
        rospy.spin()

    def define_fieldOfView_footprint(self, cam_info):
        # find out what area of the ground is visible to the robot relative to its pose
        image_size = (cam_info.width,cam_info.height)
        image_corners = PixelArray()
        frame = "/{}/base_link".format(self.robot_id)
        image_corners.array =[Pixel(coordinates = (0, image_size[1]-1)), Pixel(coordinates = (image_size[0]-1, 0)), Pixel(coordinates = (0,0))]
        FoV_corners = self.express_pixels_in_different_frame(image_corners, frame)
        height = self.eucl_distance(FoV_corners.points[0], FoV_corners.points[2])
        width = self.eucl_distance(FoV_corners.points[1], FoV_corners.points[2])
        rospy.set_param('FoV_footprint', [float(height),float(width)])

    def weed_callback(self, data):
        frame = "/map"
        world_coord = self.express_pixels_in_different_frame(data, frame)
        self.pub.publish(world_coord)

    def express_pixels_in_different_frame(self, data, frame):
        pixels = data.array
        world_coordinates = PointCloud()
        world_coordinates.header.frame_id = "/map"
        array = []
        for pixel in pixels:
            p = np.array([pixel.coordinates[0],pixel.coordinates[1],1])
            camera_projection = np.dot(self.inv_camera_matrix,p) # a point on the line to the ground in camera coordinates
            pose = PointStamped()
            pose.header.frame_id = "/{}/kinect2_rgb_optical_frame".format(self.robot_id)
            pose.point.x = self.camera_height * camera_projection[0]
            pose.point.y = self.camera_height * camera_projection[1]
            pose.point.z = self.camera_height
            world_projection = self.tfl.transformPoint(frame, pose)
            array.append(world_projection.point)
        world_coordinates.points = array
        return world_coordinates

    def eucl_distance(self, point1, point2):
		xdiff = point1.x - point2.x
		ydiff = point1.y - point2.y
		dist = np.sqrt((xdiff ** 2) + (ydiff ** 2))
		return dist

if __name__ == "__main__":
    cc = Camera_converter(sys.argv[1],sys.argv[2])
