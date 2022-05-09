# collecting images
# This code receives user input to begin recording DIGIT image data and saves it.

# I want this code to display the DIGIT's camera view in real time, allow users to begin and end recordings, and save the information.
import logging
import pprint
import time

import cv2

from digit_interface.digit import Digit
from digit_interface.digit_handler import DigitHandler

logging.basicConfig(level=logging.DEBUG)

# Print a list of connected DIGIT's
digits = DigitHandler.list_digits()
print("Connected DIGIT's to Host:")
pprint.pprint(digits)

# Connect to a Digit device with serial number with friendly name
digit = Digit("D00045", "Left Gripper")
digit.connect()
digit.set_intensity_rgb(15,15,15)
digit.show_view()
