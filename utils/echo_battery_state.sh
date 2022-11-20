#!/bin/bash

echo -e "\n*** Checking TB4lite (Create3) Battery with ROS2"

# only Humble
# echo "ros2 topic echo --once /battery_state"
# ros2 topic echo --once /battery_state

# in Galactic
echo "ros2 topic echo /battery_state"
ros2 topic echo /battery_state
