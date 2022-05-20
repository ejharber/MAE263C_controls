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
from std_msgs.msg import Float32MultiArray
from digit_processing_fn import *
from std_msgs.msg import MultiArrayDimension

import numpy as np

# def numpy_to_multiarray(np_array):
#     multiarray = Float64MultiArray()
#     multiarray.layout.dim = [MultiArrayDimension('dim%d' % i,
#                                                  np_array.shape[i],
#                                                  np_array.shape[i] * np_array.dtype.itemsize) for i in range(np_array.ndim)];
#     multiarray.data = np_array.reshape([1, -1])[0].tolist();
#     return multiarray

# def _multiarray_to_numpy(pytype, dtype, multiarray):
#     dims = map(lambda x: x.size, multiarray.layout.dim)
#     return np.array(multiarray.data, dtype=pytype).reshape(dims).astype(dtype)

class img_processing_node(object):
    """woof woof"""
    def __init__(self):

        # Connect to a Digit device with serial number with friendly name
        self.pub_image_thresh = rospy.Publisher("/digit_data/thresh", Image, queue_size=1)
        self.pub_image_visual = rospy.Publisher("/digit_data/visual", Image, queue_size=1)
        self.pub_image_contours = rospy.Publisher("/digit_data/contours", Image, queue_size=1)
        self.pub_image_force = rospy.Publisher("/digit_data/force", Float32MultiArray)

        self.bridge = CvBridge()
        self.last_img = None

        rospy.Subscriber("/digit_data/raw", Image, self.img_callback)

    def img_callback(self, msg):
        self.last_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def numpy_to_multiarray(self, np_array):
        # print()
        multiarray = Float32MultiArray()
        multiarray.layout.dim = [MultiArrayDimension('dim%d' % i,
                                                     np_array.shape[i],
                                                     np_array.shape[i] * np_array.dtype.itemsize) for i in range(np_array.ndim)];
        multiarray.data = np_array.reshape([1, -1])[0].tolist();
        return multiarray

if __name__ == '__main__': 
    try:
        rospy.init_node('img_processing', anonymous=True)
        img_processing_node = img_processing_node()
        rate = rospy.Rate(100) # 100hz is required by the Kinva Arm

        while not rospy.is_shutdown():
            if img_processing_node.last_img is not None:
                thresh, contours, visual, normalized_areas = process_image(img_processing_node.last_img)
                thresh_msg = img_processing_node.bridge.cv2_to_imgmsg(thresh)
                visual_msg = img_processing_node.bridge.cv2_to_imgmsg(visual)
                contours_msg = img_processing_node.bridge.cv2_to_imgmsg(contours, encoding="rgb8")
                # thresh_msg = bridge.cv2_to_imgmsg(thresh, encoding="rgb8") 

                img_processing_node.pub_image_thresh.publish(thresh_msg)
                img_processing_node.pub_image_visual.publish(visual_msg) 
                img_processing_node.pub_image_contours.publish(contours_msg)               
                img_processing_node.pub_image_force.publish(img_processing_node.numpy_to_multiarray(np.array(normalized_areas).flatten()))

                img_processing_node.last_img = None
            else:
                rate.sleep()

            


    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

