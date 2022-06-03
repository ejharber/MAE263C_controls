#!/usr/bin/env python3

import time
import roslib
import rospy
from robot_velocity_cmds import *

class position_correction(object):
    """ROS node for sending position commands to Kinova"""
    def __init__(self):

        # previous position estimate
        self.position = None

        # don't think I need a publisher to send position commands to the robot

        # listener to extract tf tree, which is a data structure with a list of transformations associated with the robot's current state
        self.listener = tf.TransformListener()

    def transform_position(self):
        # unsure if I should directly lift lines 35-37, 40-43 from robot_velocity_cmds.py to transform position

        # converting from position in the distal knuckle frame to position in the end effector frame
        rot_adjustment = Rotation.from_rotvec([0, 0, -20*np.pi/180]).as_matrix() # lifted from robot_velocity_cmds.py
        psn_adjustment = np.matmul(rot_adjustment.T, [30; 0; 0])# assuming the end effector frame is 30 mm along the z axis of the distal knuckle frame

        # extracting current position from listener
        try:
            (p, R) = self.listener.lookupTransform('root', 'j2s7s300_link_finger_tip_1', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None

        # transform position to reflect addition of DIGIT (translate frame)
        return np.array(p) + psn_adjustment

    def position_error(self):
        # calculates the error in position when using velocity commands

        # find the actual position (corrected for the addition of a DIGIT)
        # find the desired position (using velocity*time)
        # find error (psn_actual - psn_desired)

        # return position error

    def psn_callback(self, msg):
        # save previous position value as corrected value
        # convert to account for DIGIT placement
        self.position = self.transform_position(msg)


if __name__ == '__main__': 
    # implementing controller to adjust for error in velocity commands
    # test velocity commands for testing the robot velocity command node 
    try:
        # setup ros node for publishing digit data
        rospy.init_node('velocity_controller', anonymous=True)
        velocity_node = velocity_controller_node()
        rate = rospy.Rate(100) # 100hz is required by the Kinova Arm

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

            # publish velocity command 
            velocity_node.pub_velocity.publish(msg)

            # pseudocode

            # call position_error, find error in position
            # do something w error to modify the next command

            # this may be best implemented at the end of each 100 second interval
            # (-) it would correct the position in a very jerky and unnatural way
            # (+) it would be the least computationally expensive approach
            # (+) it would be structured simply; this code would be another if statement checking for the end of an interval 

            # for the sake of implementing in any general motion controller though it may be best to do it immediately after executing the velocity command


            counter += 1

            # limit ros loop to 
            rate.sleep() # ROS sleep once


    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise
