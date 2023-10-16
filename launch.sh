#!/bin/bash

# Source the ROS Foxy setup script
source /opt/ros/foxy/setup.sh

# Source your workspace setup script (change "/dff_ws" to the actual path)
source /dff_ws/install/setup.bash

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/dff_ws/src/dff_bot/models/
# which ros2
ros2 launch dff_bot nav2_launch.launch.py