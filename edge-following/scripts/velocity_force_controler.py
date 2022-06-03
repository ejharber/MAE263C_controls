#!/usr/bin/env python3

import time
import roslib
import rospy
from robot_velocity_cmds import *
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseStamped

class force_velocity_controller_node(velocity_controller_node):
    """force controller which inherits from the velocity controller"""
    def __init__(self):
        super().__init__()

        # set up substriber to listen to the previous force values
        # rospy.Subscriber('/digit_data/force', Float32MultiArray, self.force_callback, queue_size=1)
        # subscribe to force2 value
        rospy.Subscriber('/digit_data/force2', Vector3, self.force2_callback, queue_size=1)
        # subscibe to current tool pose
        rospy.Subscriber('/j2s7s300_driver/out/tool_pose', PoseStamped, self.pose_callback, queue_size=1)
        # publish velocity
        self.pub_actual_velocity = rospy.Publisher("/controller_metrics/actual_velocity", Float32, queue_size=1)

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

    # def force_callback(self, msg):
    #     # save previous force values
    #     self.forces = self.multiarray_to_numpy(msg)

    #     # moving average of previous force values 
    #     self.moving_avg_history[:, self.moving_avg_i] = self.forces
    #     self.moving_avg_i = (self.moving_avg_i + 1) % self.moving_avg_history.shape[1]
    #     self.moving_avg = np.mean(self.moving_avg_history, 1)

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

        # zero sensor 
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

        zero_sensor = np.mean(history)
        # controller
        # controller p and m term for force
        # note we could have a different p term for each 9 force estimates
        # p = -.005
        m_inv = 1.0/15.0
        p = 1


        # ros loop 
        t_vel = 0.0
        mvright = 0.0
        while not rospy.is_shutdown():

            # calculate velocity using pose feedback
            dt = (force_velocity_controller_node.time - last_time)*(1e-9)
            # update last time stamp
            last_time = force_velocity_controller_node.time
            # use temp variables to calculate velocity and update last pose
            mag_last = np.sqrt(last_pose.dot(last_pose))
            pose = force_velocity_controller_node.pose
            mag_current = np.sqrt(pose.dot(pose))
            if not dt == 0.0:
                t_vel = (mag_current-mag_last)/dt

            last_pose = pose

            # velocity set to be proportional to force estimated by digit
            # force
            F = force_velocity_controller_node.moving_avg - zero_sensor
            # desired force
            F_d = 2
            # force controller as velocity input
            u_vel = p*m_inv*(F_d -  (force_velocity_controller_node.moving_avg - zero_sensor))
            # print("commanded velocity")
            # print(u_vel)
            # print("actual velocity")
            print(t_vel)
            if u_vel < 1e-18:
                mvright = .2

            msg = force_velocity_controller_node.transform_velocity([0,u_vel,mvright], [0,0,0])

            # set force value to robot
            if msg is not None:
                force_velocity_controller_node.pub_velocity.publish(msg)
                pass

            msg = Float32()
            msg.data = t_vel
            force_velocity_controller_node.pub_actual_velocity.publish(msg)

            rate.sleep()

    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise


    