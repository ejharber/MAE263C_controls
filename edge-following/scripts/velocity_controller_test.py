#!/usr/bin/env python

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

        self.pub_velocity = rospy.Publisher("/j2s7s300_driver/in/cartesian_velocity", kinova_msgs.msg.PoseVelocity)

        self.listener = tf.TransformListener()

    def tf_callback(self, msg):
        # print(msg)

        pass

    def transform_velocity(self, v, w):
        def screw(a):
            return np.array([[0, -a[2], a[1]],
                    [a[2], 0, -a[0]],
                    [-a[1], a[0], 0]])

        try:
            (p, R) = velocity_node.listener.lookupTransform('root', 'j2s7s300_link_finger_tip_1', rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None, None

        p = np.array(p)
        R = np.array(Rotation.from_quat(R).as_dcm()) # as_matrix for python3

        v_goal = np.matmul(R, np.array([v]).T)
        w_goal = np.array([[0.0, 0.0, 0.0]]).T

        try:
            (p, R) = velocity_node.listener.lookupTransform('j2s7s300_end_effector', 'j2s7s300_link_finger_tip_1', rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None, None

        p = np.array(p)
        R = np.array(Rotation.from_quat(R).as_dcm()) # as_matrix for python3

        v_goal += np.matmul(np.matmul(screw(p),R), np.array([w]).T)

        print(R.shape)
        print(np.array([w]).T.shape)

        w_goal += np.matmul(R, np.array([w]).T)

        return v_goal, w_goal

if __name__ == '__main__': 
    try:
        rospy.init_node('velocity_controller', anonymous=True)
        velocity_node = velocity_controller_node()
        rate = rospy.Rate(20) # 100hz is required by the Kinva Arm

        counter = 0

        while not rospy.is_shutdown():

            msg = kinova_msgs.msg.PoseVelocity()

            if counter < 100:

                v_goal, w_goal = velocity_node.transform_velocity([0,0,0], [0,0.5,0])
                # v_goal = np.array([[0,0,0]]).T
                # w_goal = np.array([[0,0.5,0]]).T

            elif counter < 200:

                v_goal, w_goal = velocity_node.transform_velocity([0, 0,0], [0,-0.5,0])
                # v_goal = np.array([[0,0,0]]).T
                # w_goal = np.array([[0,-0.5,0]]).T

            else: 
                counter = 0
                continue 

            if v_goal is None:
                continue 

            msg.twist_linear_x = v_goal[0,0]
            msg.twist_linear_y = v_goal[1,0]        
            msg.twist_linear_z = v_goal[2,0]

            msg.twist_angular_x = w_goal[0,0]
            msg.twist_angular_y = w_goal[1,0]                
            msg.twist_angular_z = w_goal[2,0]

            print(msg)

            velocity_node.pub_velocity.publish(msg)

            counter += 1

            rate.sleep() # ROS sleep once


    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise
