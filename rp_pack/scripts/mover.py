#!/usr/bin/env python
import rospy
import numpy as np

class Mover():
    def __init__(self, robot_name):
        rospy.init_node('mover', anonymous = True)

if __name__ == "__main__":
    mov = Mover('thorvald_001')
