#!/usr/bin/env python3

import time
import roslib
import rospy
from robot_velocity_cmds import *
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.spatial.transform import Rotation


class force_velocity_controller_node(velocity_controller_node):
    """force controller which inherits from the velocity controller"""
    def __init__(self):
        super().__init__()

        # subscribe to force2 value
        rospy.Subscriber('/digit_data/force2', Vector3, self.force2_callback, queue_size=1)
        # subscibe to current tool pose
        rospy.Subscriber('/j2s7s300_driver/out/tool_pose', PoseStamped, self.pose_callback, queue_size=1)
        # publish velocity
        self.pub_actual_velocity = rospy.Publisher("/controller_metrics/actual_velocity", Float32, queue_size=1)
        self.pub_actual_force = rospy.Publisher("/controller_metrics/actual_force", Float32, queue_size=1)

        # previous force estimate
        self.forces = None

        # objects associated with the moving average
        # averages the lat 15 force estimates
        self.moving_avg_history = np.zeros(5)
        self.moving_avg_i = 0
        self.moving_avg = None

        self.pose = None
        self.time = None


    def multiarray_to_numpy(self, multiarray):
        # convert a ros list msg to a numpy array 
        return np.array(multiarray.data, np.float32).reshape(9).astype(np.float64)


    def force2_callback(self, msg):
        # save previous force values
        self.forces = msg.x

        # moving average of previous force values 
        self.moving_avg_history[self.moving_avg_i] = self.forces
        self.moving_avg_i = (self.moving_avg_i + 1) % 5
        self.moving_avg = np.mean(self.moving_avg_history)


    def pose_callback(self, msg):
        self.pose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        self.pose = np.array(self.pose)
        self.time = msg.header.stamp.to_nsec()


if __name__ == '__main__': 
    try:

        # setup ros node for hybrid force/velocity controller
        rospy.init_node('force_velocity_controller', anonymous=True)
        force_velocity_controller_node = force_velocity_controller_node()
        rate = rospy.Rate(20) # 100hz is required by the Kinova Arm, change back if problems

        # ZERO SENSOR 

        n = 40
        history = np.zeros(n)
        last_pose = None
        last_time = None
        for i in range(n):
            history[i] = rospy.wait_for_message("/digit_data/force2", Vector3).x
            temp = rospy.wait_for_message('/j2s7s300_driver/out/tool_pose', PoseStamped)
            last_pose = [temp.pose.position.x, temp.pose.position.y, temp.pose.position.z]
            last_pose = np.array(last_pose)
            # print(temp.header.stamp.to_sec())
            last_time = temp.header.stamp.to_nsec()

        # sensor baseline
        zero_sensor = np.mean(history)

        # CONTROLLER

        # controller p and m term for force error
        m_inv = 1.0/10.0   # 10
        # m_inv = 1.0/3.0
        p_f = 1.0
        p_v = 1

        # ros loop 
        m_vel = 0.0
        mvright = 0.0
        while not rospy.is_shutdown():

            # calculate y velocity using pose feedback
            dt = (force_velocity_controller_node.time - last_time)*(1e-9)
            # update last time stamp
            last_time = force_velocity_controller_node.time
            # use temp variables to calculate velocity and update last pose
            mag_last = np.sqrt(last_pose.dot(last_pose))
            pose = force_velocity_controller_node.pose
            mag_current = np.sqrt(pose.dot(pose))
            if not dt == 0.0:
                m_vel = (mag_current-mag_last)/dt

            last_pose = pose

            # velocity set to be proportional to force estimated by digit
            # force
            F = force_velocity_controller_node.moving_avg - zero_sensor
            # print(force_velocity_controller_node.moving_avg ,zero_sensor)
            # desired force
            F_d = 2.0
            # force controller as velocity input
            u_vel = p_f*m_inv*(F_d - F)
            # u_vel = u_vel - p_v*m_vel
            # print("commanded velocity")
            # print(u_vel)
            # print("actual velocity")
            print(m_vel)
            # if u_vel < 1e-18:
            #     mvright = .075
            #     mvright = 0

            msg = force_velocity_controller_node.transform_velocity([0,u_vel,mvright], [0,0,0])

            # set force value to robot
            if msg is not None:
                force_velocity_controller_node.pub_velocity.publish(msg)
                pass

            msg = Float32()
            msg.data = u_vel
            force_velocity_controller_node.pub_actual_velocity.publish(msg)

            msg = Float32()
            msg.data = F
            force_velocity_controller_node.pub_actual_force.publish(msg)

            rate.sleep()

    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise


    