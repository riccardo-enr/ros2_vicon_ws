#!/bin/bash

# Source the ROS 2 workspace setup file
source ./install/setup.bash

# Launch the ROS 2 launch file
ros2 launch vicon_to_px4 vicon_to_px4.launch.py