#!/usr/bin/env python3

# outputs frame data

import logging
import pprint
import time
import roslib
import rospy

import cv2

from digit_interface.digit import Digit
from digit_interface.digit_handler import DigitHandler
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
	


class digit_node(object):
    """woof woof"""
    def __init__(self):

        # Connect to a Digit device with serial number with friendly name
        self.digit = Digit("D00045", "Left Gripper")
        self.digit.connect()
        self.digit.set_intensity_rgb(15,15,15)
    
        # Change DIGIT resolution to QVGA
        qvga_res = Digit.STREAMS["QVGA"]
        self.digit.set_resolution(qvga_res)

        # Change DIGIT FPS to 30fps
        fps_30 = Digit.STREAMS["QVGA"]["fps"]["30fps"]
        self.digit.set_fps(fps_30)

        self.pub_image = rospy.Publisher("/digit_data/raw", Image)

if __name__ == '__main__': 
    try:
        rospy.init_node('digit', anonymous=True)
        digit_node = digit_node()
        rate = rospy.Rate(100) # 100hz is required by the Kinva Arm
        bridge = CvBridge()

        while not rospy.is_shutdown():

            # Grab single frame from DIGIT
            frame = digit_node.digit.get_frame()
            image_message = bridge.cv2_to_imgmsg(frame, encoding="rgb8")
            # image_message.encoding = 'rgb8'
            digit_node.pub_image.publish(image_message)    
            # rate.sleep() # ROS sleep once


    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise



