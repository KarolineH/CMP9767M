#!/usr/bin/env python
import rospy
import numpy as np
import sys
from nav_msgs.msg import OccupancyGrid

class Controller():
	def __init__(self, robot_id, role):
		self.role = role
		rospy.init_node('Controller', anonymous = True)

		if self.role == "explorer":
			# Wait for the camera_converter node to specify camera setup details
			while not rospy.has_param('FoV_footprint'):
				time.sleep(1)
			min_view_dimension = min(rospy.get_param('FoV_footprint'))

		elif self.role == "sprayer":

		else:
			ROS_DEBUG("INVALID ROLE: Does the launch file specify a role for each robot?");

		rospy.wait_for_service('global_localization')
		rospy.ServiceProxy('global_localization', Empty)

if __name__ == "__main__":
	ctrl = Controller(sys.argv[1],sys.argv[2])
