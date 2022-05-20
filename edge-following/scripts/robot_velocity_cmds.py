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
    """woof woof"""
    def __init__(self):

        rospy.Subscriber('/tf', tf2_msgs.msg.TFMessage, self.tf_callback, queue_size=1)

        self.pub_velocity = rospy.Publisher("/j2s7s300_driver/in/cartesian_velocity", kinova_msgs.msg.PoseVelocity, queue_size=1)

        self.listener = tf.TransformListener()

    def tf_callback(self, msg):
        pass

    def transform_velocity(self, v, w):
        def screw(a):
            return np.array([[0, -a[2], a[1]],
                    [a[2], 0, -a[0]],
                    [-a[1], a[0], 0]])

        R_finger_sensor = Rotation.from_rotvec([0, 0, -20*np.pi/180]).as_matrix()
        p_finger_sensor = np.matmul(R_finger_sensor.T, -np.array([35, 8.19, 0])/1000)
        # p_finger_sensor = np.array([42, 8.19, 0])/1000

        try: # from base to finger tip 
            (p, R) = self.listener.lookupTransform('root', 'j2s7s300_link_finger_tip_1', rospy.Time(0))
            print(p)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None

        p = np.array(p)
        R = np.array(Rotation.from_quat(R).as_matrix()) # as_matrix for python3
        p += p_finger_sensor
        R = np.matmul(R, R_finger_sensor)

        v_goal = np.matmul(R, np.array([v]).T)
        w_goal = np.array([[0.0, 0.0, 0.0]]).T

        try: # from ee to finger tip 
            (p, R) = self.listener.lookupTransform('j2s7s300_end_effector', 'j2s7s300_link_finger_tip_1', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None

        p = np.array(p)
        R = np.array(Rotation.from_quat(R).as_matrix()) # as_matrix for python3
        p += p_finger_sensor
        R = np.matmul(R, R_finger_sensor)

        v_goal += np.matmul(np.matmul(screw(p),R), np.array([w]).T)

        print(R.shape)
        print(np.array([w]).T.shape)

        w_goal += np.matmul(R, np.array([w]).T)

        msg = kinova_msgs.msg.PoseVelocity()
        
        msg.twist_linear_x = v_goal[0,0]
        msg.twist_linear_y = v_goal[1,0]        
        msg.twist_linear_z = v_goal[2,0]

        msg.twist_angular_x = w_goal[0,0]
        msg.twist_angular_y = w_goal[1,0]                
        msg.twist_angular_z = w_goal[2,0]

        return msg