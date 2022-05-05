#!/usr/bin/env python

import time
import roslib
import rospy
import tf2_msgs.msg
import kinova_msgs.msg
import tf 

class velocity_controller_node(object):
    """woof woof"""
    def __init__(self):

        rospy.Subscriber('/tf', tf2_msgs.msg.TFMessage, self.tf_callback, queue_size=1)

        self.pub_velocity = rospy.Publisher("/j2s7s300_driver/in/cartesian_velocity", kinova_msgs.msg.PoseVelocity)

        self.listener = tf.TransformListener()

    def tf_callback(self, msg):
        # print(msg)
        pass     

if __name__ == '__main__': 
    try:
        rospy.init_node('velocity_controller', anonymous=True)
        velocity_node = velocity_controller_node()
        rate = rospy.Rate(100) # 100hz is required by the Kinva Arm

        while not rospy.is_shutdown():
            msg = kinova_msgs.msg.PoseVelocity()
            msg.twist_angular_z = -.2
            velocity_node.pub_velocity.publish(msg)

            (trans,rot) = velocity_node.listener.lookupTransform('j2s7s300_link_finger_3', 'j2s7s300_link_end_effector', rospy.Time(0))

            try:
                (trans,rot) = velocity_node.listener.lookupTransform('j2s7s300_link_finger_3', 'j2s7s300_link_end_effector', rospy.Time(0))
                print(trans)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue


            rate.sleep() # ROS sleep once


    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise
