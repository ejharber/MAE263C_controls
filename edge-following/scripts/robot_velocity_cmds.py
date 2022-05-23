#!/usr/bin/env python3

import time
import roslib
import rospy
import tf2_msgs.msg
import kinova_msgs.msg
import tf 
import numpy as np
from scipy.spatial.transform import Rotation

class velocity_controller_node(object):
    """ROS node for sending velocity commands to kinova"""
    def __init__(self):

        # publisher for sending velocity commands to robot
        self.pub_velocity = rospy.Publisher("/j2s7s300_driver/in/cartesian_velocity", kinova_msgs.msg.PoseVelocity, queue_size=1)

        # listens to tf tree, which is a data structure which holds the list of transformations acociated with the robots current state
        self.listener = tf.TransformListener()

    def transform_velocity(self, v_digit, w_digit):
        # math to convert from finger tip frame to weird kinova frame 
        # for more info read this https://physics.stackexchange.com/questions/197009/transform-velocities-from-one-frame-to-an-other-within-a-rigid-body
        # Note: the kinova takes linear velocity commands in the base frame 
        # and angular velocity commands in the end effector frame 

        def screw(a):
            # convert to a screw matrix 
            return np.array([[0, -a[2], a[1]],
                    [a[2], 0, -a[0]],
                    [-a[1], a[0], 0]])

        # rotation and translation from finger tip frame to center of digit
        R_finger_sensor = Rotation.from_rotvec([0, 0, -20*np.pi/180]).as_matrix()
        p_finger_sensor = np.matmul(R_finger_sensor.T, -np.array([35, 8.19, 0])/1000)
        # p_finger_sensor = np.array([42, 8.19, 0])/1000


        try: # get current transformation from base to finger tip 
            (p, R) = self.listener.lookupTransform('root', 'j2s7s300_link_finger_tip_1', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None

        # transform the desired v and w into the base frame 
        p = np.array(p)
        R = np.array(Rotation.from_quat(R).as_matrix()) # as_matrix for python3
        p += p_finger_sensor
        R = np.matmul(R, R_finger_sensor)

        v_kinova = np.matmul(R, np.array([v_digit]).T)
        w_kinova = np.array([[0.0, 0.0, 0.0]]).T

        try: # get current transformation from the end effector to the finger tip 
            (p, R) = self.listener.lookupTransform('j2s7s300_end_effector', 'j2s7s300_link_finger_tip_1', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None

        # transform the desired v and w into the ee frame 
        p = np.array(p)
        R = np.array(Rotation.from_quat(R).as_matrix()) # as_matrix for python3
        p += p_finger_sensor
        R = np.matmul(R, R_finger_sensor)

        v_kinova += np.matmul(np.matmul(screw(p),R), np.array([w_digit]).T)
        w_kinova += np.matmul(R, np.array([w_digit]).T)

        # construct ros msg using v and w in kinova frame
        msg = kinova_msgs.msg.PoseVelocity()
        
        msg.twist_linear_x = v_kinova[0,0]
        msg.twist_linear_y = v_kinova[1,0]        
        msg.twist_linear_z = v_kinova[2,0]

        msg.twist_angular_x = w_kinova[0,0]
        msg.twist_angular_y = w_kinova[1,0]                
        msg.twist_angular_z = w_kinova[2,0]

        return msg