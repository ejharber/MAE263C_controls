#!/usr/bin/env python3

import time
import roslib
import rospy
from robot_velocity_cmds import *
from std_msgs.msg import Float32MultiArray

class force_velocity_controller_node(velocity_controller_node):
    """docstring for ClassName"""
    def __init__(self):
        super().__init__()

        rospy.Subscriber('/digit_data/force', Float32MultiArray, self.force_callback, queue_size=1)

        self.forces = None

    def multiarray_to_numpy(self, multiarray):
        return np.array(multiarray.data, np.float32).reshape(9).astype(np.float64)

    def force_callback(self, msg):
        self.forces = self.multiarray_to_numpy(msg)

if __name__ == '__main__': 
    try:
        rospy.init_node('force_velocity_controller', anonymous=True)
        force_velocity_controller_node = force_velocity_controller_node()
        rate = rospy.Rate(20) # 100hz is required by the Kinova Arm, change back if problems

        # zero sensor 
        n = 50
        history = np.zeros((n, 9))
        for i in range(n):
            history[i, :] = force_velocity_controller_node.multiarray_to_numpy(rospy.wait_for_message("/digit_data/force", Float32MultiArray))

        zero_sensor = np.mean(history, 0)

        # controller
        p = 500
        while not rospy.is_shutdown():

            # vel = p*(force_velocity_controller_node.forces[5] - zero_sensor[5]) + .2
            vel = p*(np.sum(force_velocity_controller_node.forces - zero_sensor)) + .2

            msg = force_velocity_controller_node.transform_velocity([0,vel,0], [0,0,0])

            if msg is not None:
                force_velocity_controller_node.pub_velocity.publish(msg)

            print(vel)

            rate.sleep()

    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise


    