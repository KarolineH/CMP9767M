#!/usr/bin/env python
import rospy
import numpy as np
import sys
import copy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud
from rp_pack.srv import PointCloudOut

class Explorer():

	"""
	This node runs for the explorer robot.
	It keeps a permanent map of weeds (topic global_weed_poses) and a to-do list of weeds that have to be sprayed.
	It automatically adds new weeds seen by the explorer.
	Also provides a service to cross weeds off the to-do list once they have been sprayed by the sprayer robot.
	"""

	def __init__(self, robot_id):
		rospy.init_node('Seeker', anonymous = True)

		#Variables
		self.robot_id = robot_id

		# Initiatialise global weed tracking (set up a weed occupancy grid map and weed point clouds)
		self.init_weed_tracking()

		# Set up connections
		self.sub = rospy.Subscriber("/{}/local_weed_poses".format(self.robot_id), PointCloud, self.add_points)
		self.weed_visual_pub = rospy.Publisher("/{}/global_weed_poses".format(self.robot_id), PointCloud, queue_size=2)
		check_service = rospy.Service('remove_weed', PointCloudOut, self.remove_point)
		to_do_service = rospy.Service('get_to_do_list', PointCloudOut, self.share_weed_locations)

		rospy.spin()

	def add_points(self, newPoints):
		# this runs for every new frame received from the plant detection nodes
		# note that this is not the original camera frame rate, but defined by the blink rate in plant_discrimination.py (default 5Hz)
		for point in newPoints.points:
			point.z = 0 # All weeds are on the ground plane
			self.global_weed_cloud.points.append(point) # All weed locations are added to the global tracker

			# Weeds are only added to the to-do list if their corresponding cell on the grid map doesn't already have a weed in it:
			# No need to spray weeds in the same cell seperately
			indx = self.Point_to_Indices(point)
			cell = self.Indices_to_Cell(indx)
			if self.weed_map.data[cell] == 0:
				self.weed_map.data[cell] = 100
				self.weed_to_do_list.points.append(point)

		self.weed_visual_pub.publish(self.global_weed_cloud) # for visualisation

	def remove_point(self, data):
		# Check a weed off the to-do list
		self.weed_to_do_list.points.pop(data.var)
		return self.weed_to_do_list

	def share_weed_locations(self, data):
		return self.weed_to_do_list

	def init_weed_tracking(self):
		# Fetch map information
		map = rospy.wait_for_message('/map', OccupancyGrid, 30)
		self.map_resolution = np.around(map.info.resolution, decimals = 3) # in m per pixel
		self.map_width = map.info.width
		self.map_height = map.info.height
		self.map_origin = map.info.origin.position

		# Create an empty copy of the occupancy map
		# This will be used to downsample the weed positions into 0.05x0.05m cells
		# The spray covers 0.1x0.1m, so multiple weeds within the same grid cell don't have to be sprayed seperately
		self.weed_map = copy.deepcopy(map)
		self.weed_map.data = np.zeros(len(map.data)).tolist()

		# Also track the weeds as points for visualisation and as nav_goals for the sprayer robot
		self.weed_to_do_list = PointCloud()
		self.global_weed_cloud = PointCloud()
		self.weed_to_do_list.header.frame_id = "/map"
		self.global_weed_cloud.header.frame_id = "/map"

	def Cell_to_Indices(self, cell):
		# Occupancy Grid Utility:
		# Translates a single cell index (row-major notation) into x,y indices (matrix notation)
		x = cell%self.map_width
		y = np.floor(cell/self.map_width)
		return [int(x),int(y)]

	def Indices_to_Cell(self, indices):
		# Occupancy Grid Utility:
		# Translates cell indices (matrix notation) into a single cell index (row-major notation)
		cell = indices[1]*self.map_width + indices[0]
		return cell

	def Point_to_Indices(self, point):
		# Occupancy Grid Utility:
		# Translates a point in the /map frame into cell indices (matrix notation)
		x = np.floor((point.x - self.map_origin.x)/self.map_resolution)
		y = np.floor((point.y - self.map_origin.y)/self.map_resolution)
		return [int(x),int(y)]

if __name__ == "__main__":
	expl = Explorer(sys.argv[1])
