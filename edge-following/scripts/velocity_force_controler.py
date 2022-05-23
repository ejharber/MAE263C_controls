#!/usr/bin/env python3

import time
import roslib
import rospy
from robot_velocity_cmds import *
from std_msgs.msg import Float32MultiArray

class force_velocity_controller_node(velocity_controller_node):
    """force controller which inherits from the velocyt controller"""
    def __init__(self):
        super().__init__()

        # set up substriber to listen to the previous force values
        rospy.Subscriber('/digit_data/force', Float32MultiArray, self.force_callback, queue_size=1)

        # previous force estimate
        self.forces = None

        # objects associated with the moving average
        # averages the lat 15 force estimates
        self.moving_avg_history = np.zeros((9, 15))
        self.moving_avg_i = 0
        self.moving_avg = None

    def multiarray_to_numpy(self, multiarray):
        # convert a ros list msg to a numpy array 
        return np.array(multiarray.data, np.float32).reshape(9).astype(np.float64)

    def force_callback(self, msg):
        # save previous force values
        self.forces = self.multiarray_to_numpy(msg)

        # moving average of previous force values 
        self.moving_avg_history[:, self.moving_avg_i] = self.forces
        self.moving_avg_i = (self.moving_avg_i + 1) % self.moving_avg_history.shape[1]
        self.moving_avg = np.mean(self.moving_avg_history, 1)

if __name__ == '__main__': 
    try:

        # setup ros node for hybrid force/velocity controller
        rospy.init_node('force_velocity_controller', anonymous=True)
        force_velocity_controller_node = force_velocity_controller_node()
        rate = rospy.Rate(20) # 100hz is required by the Kinova Arm, change back if problems

        # zero sensor 
        n = 40
        history = np.zeros((n, 9))
        for i in range(n):
            history[i, :] = force_velocity_controller_node.multiarray_to_numpy(rospy.wait_for_message("/digit_data/force", Float32MultiArray))

        zero_sensor = np.mean(history, 0)

        # controller p term
        # note we could have a different p term for each 9 force estimates
        p = 40

        # ros loop 
        while not rospy.is_shutdown():

            # valocity set to be proportional to force estimated by digit
            vel = p*(np.sum(force_velocity_controller_node.moving_avg - zero_sensor)) 
            msg = force_velocity_controller_node.transform_velocity([0,vel,0], [0,0,0])

            # set force value to robot
            if msg is not None:
                force_velocity_controller_node.pub_velocity.publish(msg)
                pass

            rate.sleep()

    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise


    