# About this repository

This is a repository 

To clone this repository:
`git clone https://github.com/biorobotics/matlab_SEA.git`

Currently this repository contains:

* Plotting Tools
* Snake Pose Estimation (Complimentary Filter)
* Arm Control


# Kinova Arm

The Kinova arm control is build on top of the kinova-ros package for more information about installation and debugging look to their repo:

https://github.com/Kinovarobotics/kinova-ros/tree/1c4b2fdf05bc929fda26aac4e420276129375375

Connecting to the robot arm is established through the serial port and 

`roslaunch edge-following robot.launch`

(Remember to run 'source devel/setup.bash' in catkin_ws first)

For viewing the link frames and robot arm in rviz run:

`rviz -d rviz_setup.rviz `

# DIGIT Sensor 

The DIGIT sensor code relies on the 

To run the ROS connection to stream the digit data over ROS run

`roslaunch edge-following digit.launch`

To view the current DIGIT data live run:

`rqt_image_view`

and select the "\digit_data\raw" topic

To view the contour images:

`rqt_image_view`

and select the "\digit_data\contours" topic

To view the plots of the contour images over time:

`rqt_plot`

and select the "\digit_data\contours" topic
