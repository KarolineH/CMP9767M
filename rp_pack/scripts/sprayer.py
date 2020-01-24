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
    First gives the explorer robot a head start, and moves to a safe starting position.
    It then fetches an up-to-date to-do list of weed positions from the explorer robot.
    It picks a weed that is preferably already close to the sprayer attachement, while also being far from the explorer robot,
    so the robots don't get in each other's way. Then navigates to the weed, sprays it, and calls the remove_weed service to check
    the weed off the to-do list and get an update on the to-do list.

    NOTE: Currently, the sprayer moves to the weed with his base_link frame, so the spray still misses the weed
    NOTE: If the sprayer robot is stuck, manually call the /next_weed service
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
        self.fetch_weed_positions = rospy.ServiceProxy('/get_to_do_list', PointCloudOut)
        self.remove_weed_position = rospy.ServiceProxy('/remove_weed', PointCloudOut)
        self.spray = rospy.ServiceProxy("/{}/spray".format(self.robot_id), Empty)
        # wait for the explorer robot to safely move out of the way
        # then move to the first waypoint of the exploration area:
        self.initial_coordination_routine()
        self.next_weed_service = rospy.Service('/next_weed', Empty, self.spraying_routine)

        self.spraying_routine(1)
        # then start spraying weeds
        rospy.spin()


    def initial_coordination_routine(self):
        rospy.loginfo("SPRAYER ROBOT: Ready. Waiting for explorer robot.")
        # Once the explorer starts moving, find out where it's going first:
        explorer_initial_pose = rospy.wait_for_message("/{}/move_base/current_goal".format(self.explorer_robot), PoseStamped, 1000)
        # Wait until the explorer's goal changes to the second waypoint
        explorer_current_goal = rospy.wait_for_message("/{}/move_base/current_goal".format(self.explorer_robot), PoseStamped, 1000)

        self.move_pub.publish(explorer_initial_pose) # then follow to the first pose of the exploration path
        rospy.loginfo("SPRAYER ROBOT: Navigating to safe starting posisition")
        # Wait until the goal is reached:
        status = 1 # status = navigating to the goal
        while status < 2:
            # status of 0 or 1 mean that the movement goal is still being processed
            status_message = rospy.wait_for_message("/{}/move_base/status".format(self.robot_id), GoalStatusArray, 30)
            status = status_message.status_list[0].status
            rospy.sleep(0.5)

    def spraying_routine(self, servarg):
        rospy.wait_for_service('/get_to_do_list', timeout=None)
        weed_coordinates_response = self.fetch_weed_positions(1)
        weed_coordinates = weed_coordinates_response.output_cloud
        if not weed_coordinates.points:
            print("weeds empty")
            rospy.sleep(1)
            weed_coordinates_response = self.fetch_weed_positions(1)
            weed_coordinates = weed_coordinates_response.output_cloud
        else:
            rospy.loginfo("SPRAYER ROBOT: To-do list received from exploration robot")

            # The weed_coordinates are where we want the spray to go
            # So find out where the sprayer is
            now = rospy.Time(0)
            self.tfl.waitForTransform("/map","/{}/sprayer".format(self.robot_id), now, rospy.Duration(100))
            sprayer_pose = self.tfl.lookupTransform("/map","/{}/sprayer".format(self.robot_id), now)
            explorer_pose = self.tfl.lookupTransform("/map","/{}/base_link".format(self.explorer_robot), now)

            # Find the weed that best satisfies the conditions i) far from eplorer ii) close to sprayer
            sprayer_translations_x = []
            sprayer_translations_y = []
            qs = [] # quality metric
            # for each weed
            for i in range(0,len(weed_coordinates.points)):

                # find the distance to the explorer robot
                explorer_dist_x = weed_coordinates.points[i].x - explorer_pose[0][0]
                explorer_dist_y = weed_coordinates.points[i].y - explorer_pose[0][1]
                explorer_eucl_dist = (explorer_dist_x ** 2 + explorer_dist_y ** 2) **0.5

                # find the distance to the sprayer attachment on the sprayer robot
                sprayer_dist_x = weed_coordinates.points[i].x - sprayer_pose[0][0]
                sprayer_dist_y = weed_coordinates.points[i].y - sprayer_pose[0][1]
                sprayer_eucl_dist = (sprayer_dist_x ** 2 + sprayer_dist_y ** 2) **0.5

                # calculate the quality metric
                q = explorer_eucl_dist**2 - sprayer_eucl_dist

                sprayer_translations_x.append(sprayer_dist_x) # store for easy navigation
                sprayer_translations_y.append(sprayer_dist_y) # store for easy navigation
                qs.append(q) # store quality metric

            # Find the best quality weed location
            next_weed_index = np.argmax(qs)
            next_weed_location = weed_coordinates.points[next_weed_index]

            #Construct a movement goal:
            movement_instruction = PoseStamped()
            movement_instruction.header.frame_id = "/map"
            movement_instruction.pose.position.x = next_weed_location.x
            movement_instruction.pose.position.y = next_weed_location.y
            movement_instruction.pose.orientation.w = 1
            self.move_pub.publish(movement_instruction)
            rospy.loginfo("SPRAYER ROBOT: Navigating to a weed")

            status = 1 # status = navigating to the goal
            rospy.sleep(10) #
            while status != 3:
                # status of 0 or 1 mean that the movement goal is still being processed
                status_message = rospy.wait_for_message("/{}/move_base/status".format(self.robot_id), GoalStatusArray, 30)
                status = status_message.status_list[0].status

            self.spray()
            rospy.loginfo("SPRAYER ROBOT: successfully sprayed weed")
            self.remove_weed_position(int(next_weed_index)) # check this weed off the to-do list

if __name__ == "__main__":
    spr = Sprayer(sys.argv[1])
