# About this repository

This is a repository 

To clone this repository:
`git clone --recurse-submodules https://github.com/ejharber/MAE263C_controls.git`

Currently this repository contains:

* Velocity Controller of a Kinova arm
* Image Processing of Digit images to estimate force
* Hybrid force-velocity controller using the digit sensor and the kinova arm

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

To view the different data streams:

`rqt --perspective-file rqt_plot.perspective`

# Hybrid Force-Velocity Controller

The hybrid force-velocity controller can be run through:

`rosrun edge-following velocity_force_controler.py`
