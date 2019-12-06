#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid

class Mover():
	def __init__(self, robot_name):
		rospy.init_node('mover', anonymous = True)
		self.init_map()
		camera_footprint

	def init_map(self):
		# fetch map information
		map = rospy.wait_for_message('/map', OccupancyGrid, 30)
		self.map_resolution = np.around(map.info.resolution, decimals = 3) # in m per pixel
		self.map_width = map.info.width
		self.map_height = map.info.height
		self.map_origin = map.info.origin.position

		# find the list indices that are unoccupied floor space
		floor_indices = []
		for i, cell in enumerate(map.data):
			if cell == 0:
				floor_indices.append(i)

		floor_coordinates = self.indices_to_coordinates(floor_indices)

	def grid_indices_to_coordinates(self, indices):
		"""Given indices of some points of interest in the row-major OccupancyGrid map, convert those into x,y coordinates"""
		# turn into np array for elementwise operations:
		indices = np.array(indices)
		column_index = indices % self.map_width
		coordinate_x = (column_index * self.map_resolution) + self.map_resolution/2 + self.map_origin.x
		coordinate_y = (indices - column_index)/self.map_width * self.map_resolution + self.map_resolution/2 + self.map_origin.y
		coord_list = zip(coordinate_x, coordinate_y)
		import pdb; pdb.set_trace()

if __name__ == "__main__":
	mov = Mover('thorvald_001')
