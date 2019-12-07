#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid

class Controller():
	def __init__(self):
		rospy.init_node('Controller', anonymous = True)
		rospy.wait_for_service('global_localization')
		rospy.ServiceProxy('global_localization', Empty)

if __name__ == "__main__":
	mov = Controller()
