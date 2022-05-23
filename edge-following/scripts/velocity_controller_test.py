#!/usr/bin/env python3

import time
import roslib
import rospy
from robot_velocity_cmds import *

if __name__ == '__main__': 
    # test veloccity commands for testing the robot velocity command node 
    try:
        # setup ros node for publishing digit data
        rospy.init_node('velocity_controller', anonymous=True)
        velocity_node = velocity_controller_node()
        rate = rospy.Rate(100) # 100hz is required by the Kinva Arm

        counter = 0

        # ROS DATA LOOP, RUN FOREVER
        while not rospy.is_shutdown():


            # publish a foward and backward linear velocity for each axis 
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

            # publish velociyt command 
            velocity_node.pub_velocity.publish(msg)

            counter += 1

            # limit ros loop to 
            rate.sleep() # ROS sleep once


    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise
