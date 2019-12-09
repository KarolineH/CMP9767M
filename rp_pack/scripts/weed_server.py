#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from rp_pack.srv import PointCloudOperation

class WeedServer():
	def __init__(self):
		rospy.init_node('WeedServer', anonymous = True)
		self.pub = rospy.Publisher('/WeedCloud', PointCloud, queue_size = 2)

		self.weed_cloud = PointCloud()
		self.weed_cloud.header.frame_id = "/map"

		push_service = rospy.Service('add_weed', PointCloudOperation, self.addPoints)
		pop_service = rospy.Service('remove_weed', PointCloudOperation, self.removePoints)

		rospy.spin()

	def addPoints(self, newPoints):
		for point in newPoints.points:
			self.weed_cloud.points.append(point)
		self.pub.publish(self.weed_cloud)

	def removePoints(self, checkedPoints):
		for point in checkedPoints.points:
			while self.weed_cloud.points.count(point) > 0: # While this point is still somewhere in the list
				self.weed_cloud.points.remove(point)# remove it
		self.pub.publish(self.weed_cloud)

if __name__ == "__main__":
	ws = WeedServer()
