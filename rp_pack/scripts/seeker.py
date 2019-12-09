#!/usr/bin/env python
import rospy
import numpy as np
import sys
import copy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud
from rp_pack.srv import PointCloudOut, PointCloudTrade
from std_srvs.srv import Empty

class Seeker():

	"""
	This node runs for the explorer robot.
	It keeps a permanent map of weeds that have to be sprayed.
	It automatically adds new weeds seen by the explorer to the map
	Provides a service to cross weeds off the map once they have been sprayed by the sprayer robot
	"""

	def __init__(self, robot_id):
		rospy.init_node('Seeker', anonymous = True)
		self.robot_id = robot_id

		# Wait for the camera_converter node to specify camera setup details
		while not rospy.has_param('/{}/FoV_footprint'.format(self.robot_id)):
			rospy.sleep(1)
		self.min_view_dimension = min(rospy.get_param('/{}/FoV_footprint'.format(self.robot_id)))

		# Initiate global weed tracking with seperate map
		self.init_map()

		# Set up connections
		self.sub = rospy.Subscriber("/{}/local_weed_poses".format(self.robot_id), PointCloud, self.add_points)
		check_service = rospy.Service('remove_weed', PointCloudTrade, self.remove_points)
		to_do_service = rospy.Service('get_to_do_list', PointCloudOut, self.share_weed_locations)
		import pdb; pdb.set_trace()
		rospy.spin()

	def add_points(self, newPoints):
		for point in newPoints.points:
			indx = self.Point_to_Indices(point)
			cell = self.Indices_to_Cell(indx)
			if self.weed_map.data[cell] == 0:
				self.weed_map.data[cell] = 100
				self.weed_cloud.points.append(point)

	def remove_points(self, checkedPoints):
		for point in checkedPoints.points:
			self.weed_cloud.points.remove(point)
		return self.weed_cloud

	def share_weed_locations(self):
		return self.weed_cloud

	def init_map(self):
		# Fetch map information
		map = rospy.wait_for_message('/map', OccupancyGrid, 30)
		self.map_resolution = np.around(map.info.resolution, decimals = 3) # in m per pixel
		self.map_width = map.info.width
		self.map_height = map.info.height
		self.map_origin = map.info.origin.position
		# Create an empty copy of the occupancy map
		self.weed_map = copy.deepcopy(map)
		self.weed_map.data = np.zeros(len(map.data)).tolist()
		# Also track the points to give as nav_goals to the sprayer robot
		self.weed_cloud = PointCloud()
		self.weed_cloud.header.frame_id = "/map"

	def Cell_to_Indices(self, cell):
	    # Translates a single cell index (row-major notation) into x,y indices (matrix notation)
	    x = cell%self.map_width
	    y = np.floor(cell/self.map_width)
	    return [int(x),int(y)]

	def Indices_to_Cell(self, indices):
	    # Translates cell indices (matrix notation) into a single cell index (row-major notation)
	    cell = indices[1]*self.map_width + indices[0]
	    return cell

	def Point_to_Indices(self, point):
	    x = np.floor((point.x - self.map_origin.x)/self.map_resolution)
	    y = np.floor((point.y - self.map_origin.y)/self.map_resolution)
	    return [int(x),int(y)]

	"""   TO DO
	set the exploration area by publishing corners to self.pub = rospy.Publisher("/clicked_point", PointStamped, queue_size=2)
	keep track of explored space
	"""

if __name__ == "__main__":
	harry = Seeker(sys.argv[1])
