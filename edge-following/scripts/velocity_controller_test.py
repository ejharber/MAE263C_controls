#!/usr/bin/env python3

import time
import roslib
import rospy
from robot_velocity_cmds import *

if __name__ == '__main__': 
    try:
        rospy.init_node('velocity_controller', anonymous=True)
        velocity_node = velocity_controller_node()
        rate = rospy.Rate(100) # 100hz is required by the Kinva Arm

        counter = 0

        while not rospy.is_shutdown():

            if counter < 100:

                msg = velocity_node.transform_velocity([0.1,0,0], [0,0,0])

            elif counter < 200:

                msg = velocity_node.transform_velocity([-0.1,0,0], [0,0,0])

            elif counter < 300:

                msg = velocity_node.transform_velocity([0,0.1,0], [0,0,0])

            elif counter < 400:

                msg = velocity_node.transform_velocity([0,-0.1,0], [0,0,0])

            elif counter < 500:

                msg = velocity_node.transform_velocity([0,0,0.1], [0,0,0])

            elif counter < 600:

                msg = velocity_node.transform_velocity([0,0,-0.1], [0,0,0])

            else: 
                counter = 0
                continue 

            if msg is None:
                continue 

            velocity_node.pub_velocity.publish(msg)

            counter += 1

            rate.sleep() # ROS sleep once


    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise
