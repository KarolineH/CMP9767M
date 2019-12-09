#!/usr/bin/env python
import rospy
import numpy as np
import sys
from sensor_msgs.msg import PointCloud
from rp_pack.srv import PointCloudOut, PointCloudTrade

class Chaser():

	"""
	This node runs for the sprayer robot.
	It fetches a to-do list of weed positions from the explorer robot.
	It then plans a route to visit and spray all weeds.
	Once sprayed, it calls the remove_weed service to check the weed off the list.
	It then fetches an updated to-do list and re-plans the route
	"""

	def __init__(self, robot_id):
		rospy.init_node('Chaser', anonymous = True)

		self.tfl = tf.TransformListener()
		self.pub = rospy.Publisher("/{}/global_weed_poses".format(self.robot_id), PointCloud, queue_size=2)
		self.fetch_weed_positions = rospy.ServiceProxy('get_to_do_list', PointCloudOut)
		self.remove_weed_position = rospy.ServiceProxy('remove_weed', PointCloudTrade)

	def runtime_routine(self):
		rospy.wait_for_service('get_to_do_list', timeout=None)
		weed_coordinates = self.fetch_weed_positions()
		# The weed coordinates is where we want the spray to go
		# The sprayer needs to be aligned with this coordinate (z-distance does not matter)

		now = rospy.Time(0)
        self.tfl.waitForTransform("/map","/{}/kinect2_rgb_optical_frame".format(self.robot_id),now, rospy.Duration(10.0))


		# Compute distance matrix
		distance_matrix = np.zeros((len(waypoints.points), len(waypoints.points)))
		for i in range(len(waypoints.points)):
			for j in range(len(waypoints.points)):
				if i < j:
					x_diff = abs(waypoints.points[i].x - waypoints.points[j].x)
					y_diff = abs(waypoints.points[i].y - waypoints.points[j].y)

					distance_matrix[i,j] = (xdiff**2 + ydiff**2)**0.5 # euclidean distance
					distance_matrix[j,i] = distance_matrix[i,j] # since the matrix is symmetrical


if __name__ == "__main__":
	ginny = Chaser(sys.argv[1])
