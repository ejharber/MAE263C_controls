#!/usr/bin/env python3

import roslib
import rospy

from digit_interface.digit import Digit
from digit_interface.digit_handler import DigitHandler
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
	
class digit_node(object):
    """ROS node for streaming raw digit data"""
    def __init__(self):

        # Connect to a Digit device with serial number with friendly name
        self.digit = Digit("D00045", "Left Gripper")
        self.digit.connect()
        self.digit.set_intensity_rgb(15,15,15)
    
        # Change DIGIT resolution to QVGA
        qvga_res = Digit.STREAMS["QVGA"]
        self.digit.set_resolution(qvga_res)

        # Change DIGIT FPS to 30fps
        # Note 60fs causes issues with data collection 
        fps_30 = Digit.STREAMS["QVGA"]["fps"]["30fps"]
        self.digit.set_fps(fps_30)

        # setup ros topic for publishing raw digit data 
        self.pub_image = rospy.Publisher("/digit_data/raw", Image)

if __name__ == '__main__': 
    try:

        # setup ros node for publishing digit data
        rospy.init_node('digit', anonymous=True)
        digit_node = digit_node()
        rate = rospy.Rate(100)

        # CV bridged used to convert imag types
        bridge = CvBridge() 

        # ROS DATA LOOP, RUN FOREVER
        while not rospy.is_shutdown():

            # Grab single frame from DIGIT
            frame = digit_node.digit.get_frame()

            # convert frame to ros msg
            image_message = bridge.cv2_to_imgmsg(frame, encoding="rgb8")

            # publish raw digit data over ros
            digit_node.pub_image.publish(image_message) 

            # rate.sleep() # ROS sleep once DO NOT NEED FOR COLLECTING DATA FROM a real sensor

    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise



