#!/usr/bin/env python
import rospy
import numpy as np
import tf
import sys
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
from visualization_msgs.msg  import MarkerArray
from actionlib_msgs.msg import GoalStatusArray
from rp_pack.srv import PointCloudOut

class Sprayer():

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
        self.robot_id = robot_id

        self.move_pub = rospy.Publisher("/{}/move_base_simple/goal".format(self.robot_id), PoseStamped, queue_size=2)
        self.weed_visual_pub = rospy.Publisher("/{}/global_weed_poses".format(self.robot_id), PointCloud, queue_size=2)
        self.fetch_weed_positions = rospy.ServiceProxy('get_to_do_list', PointCloudOut)
        self.remove_weed_position = rospy.ServiceProxy('remove_weed', PointCloudOut)
        self.spray = rospy.ServiceProxy("/{}/spray".format(self.robot_id), Empty)

        exploration_topics = []
        while not exploration_topics:
            topics = rospy.get_published_topics()
            topic_names = [topic[0] for topic in topics]
            exploration_topics = filter(lambda t: "visualization_marker" in t, topic_names)
            rospy.sleep(0.5)
        rospy.wait_for_message(exploration_topics[0], MarkerArray, 30)
        # wait until the exploration robot has started moving
        while not rospy.is_shutdown():
            self.runtime_routine()

    def runtime_routine(self):
        rospy.wait_for_service('get_to_do_list', timeout=None)
        weed_coordinates_response = self.fetch_weed_positions(1)
        weed_coordinates = weed_coordinates_response.output_cloud
        if not weed_coordinates.points:
            rospy.sleep(1)
            weed_coordinates_response = self.fetch_weed_positions(1)
            weed_coordinates = weed_coordinates_response.output_cloud
        else:
            # The weed_coordinates are where we want the spray to go
            # The sprayer needs to be aligned with this coordinate (z-distance does not matter)

            now = rospy.Time(0)
            self.tfl.waitForTransform("/map","/{}/sprayer".format(self.robot_id), now, rospy.Duration(100))
            sprayer_pose = self.tfl.lookupTransform("/map","/{}/sprayer".format(self.robot_id), now)
            # Find closest next weed
            x_distances = []
            y_distances = []
            distances = []
            for i in range(0,len(weed_coordinates.points)):
                x_diff = weed_coordinates.points[i].x - sprayer_pose[0][0]
                y_diff = weed_coordinates.points[i].y - sprayer_pose[0][1]
                x_distances.append(x_diff)
                y_distances.append(y_diff)
                distances.append((x_diff**2 + y_diff**2)**0.5) # euclidean distance

            closest_weed_index = np.argmin(distances)
            closest_weed_location = weed_coordinates.points[closest_weed_index]

            movement_instruction = PoseStamped()
            movement_instruction.header.frame_id = "/{}/base_link".format(self.robot_id)
            movement_instruction.pose.position.x = x_distances[closest_weed_index]
            movement_instruction.pose.position.y = y_distances[closest_weed_index]
            movement_instruction.pose.orientation.w = 1
            self.move_pub.publish(movement_instruction)

            status = 1 # navigating to the goal
            while status < 2:
                # status of 0 or 1 mean that the movement goal is still being processed
                status_message = rospy.wait_for_message("/{}/move_base/status".format(self.robot_id), GoalStatusArray, 30)
                status = status_message.status_list[0].status
                rospy.sleep(0.5)

            if status == 3: # if the goal has been reached successfully
                #rospy.wait_for_service('spray', timeout=None)
                self.spray()
                self.remove_weed_position(int(closest_weed_index))

if __name__ == "__main__":
    spr = Sprayer(sys.argv[1])
