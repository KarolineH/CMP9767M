#!/usr/bin/env python
import rospy
import numpy as np
import tf
import sys
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
from visualization_msgs.msg  import MarkerArray
from actionlib_msgs.msg import GoalStatusArray
from rp_pack.srv import PointCloudOut

class Sprayer():

    """
    This node runs for the sprayer robot.
    First gives the explorer robot a head start, then fetches an up-to-date to-do list of weed positions from the explorer robot.
    It picks a weed that is preferably already close to the sprayer attachement, while also being far from the explorer robot,
    so the robot's don't get in each other way. Then navigates to the weed, sprays it, and calls the remove_weed service to check
    the weed off the to-do list and get an update on the to-do list.
    """

    def __init__(self, robot_id):

        rospy.init_node('Chaser', anonymous = True)

        # Variables
        self.robot_id = robot_id
        self.explorer_robot = "thorvald_001" # anually rename  if necessary
        self.min_robot_distance = 1.2 # in meters
        self.head_start = 30 # in seconds

        # Set up connections
        self.tfl = tf.TransformListener()
        self.move_pub = rospy.Publisher("/{}/move_base_simple/goal".format(self.robot_id), PoseStamped, queue_size=2)
        self.fetch_weed_positions = rospy.ServiceProxy('get_to_do_list', PointCloudOut)
        self.remove_weed_position = rospy.ServiceProxy('remove_weed', PointCloudOut)
        self.spray = rospy.ServiceProxy("{}/spray".format(self.robot_id), Empty)

        #while not rospy.is_shutdown():
            # then start spraying weeds
        self.spraying_routine()

    def spraying_routine(self):
        rospy.wait_for_service('get_to_do_list', timeout=None)
        weed_coordinates_response = self.fetch_weed_positions(1)
        weed_coordinates = weed_coordinates_response.output_cloud
        if not weed_coordinates.points:
            print("weeds empty")
            rospy.sleep(1)
            weed_coordinates_response = self.fetch_weed_positions(1)
            weed_coordinates = weed_coordinates_response.output_cloud
        else:
            rospy.loginfo("SPRAYER ROBOT: To-do list received from exploration robot")

            # Find the closest weed (z-dimension does not matter)
            now = rospy.Time(0)
            self.tfl.waitForTransform("/map","/{}/base_link".format(self.robot_id), now, rospy.Duration(100))
            current_pose = self.tfl.lookupTransform("/map","/{}/base_link".format(self.robot_id), now)
            distances = []
            for i in range(0,len(weed_coordinates.points)):
                # find the distance to the sprayer robot
                sprayer_dist_x = weed_coordinates.points[i].x - current_pose[0][0]
                sprayer_dist_y = weed_coordinates.points[i].y - current_pose[0][1]
                sprayer_eucl_dist = (sprayer_dist_x ** 2 + sprayer_dist_y ** 2) **0.5
                distances.append(sprayer_eucl_dist)

            next_weed_index = np.argmin(dinstances)
            next_weed_location = weed_coordinates.points[next_weed_index]

            #Construct a movement goal:
            movement_instruction = PoseStamped()
            movement_instruction.header.frame_id = "/{}/base_link".format(self.robot_id)
            movement_instruction.pose.position.x = next_weed_location.x
            movement_instruction.pose.position.y = next_weed_location.y
            movement_instruction.pose.orientation.w = 1
            self.move_pub.publish(movement_instruction)

            status = 1 # status = navigating to the goal
            while status < 2:
                # status of 0 or 1 mean that the movement goal is still being processed
                status_message = rospy.wait_for_message("/{}/move_base/status".format(self.robot_id), GoalStatusArray, 30)
                status = status_message.status_list[0].status
                rospy.sleep(0.5)

            self.spray()
            self.remove_weed_position(int(next_weed_index)) # check this weed off the to-do list

if __name__ == "__main__":
    spr = Sprayer(sys.argv[1])
