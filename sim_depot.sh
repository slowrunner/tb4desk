#!/bin/bash

echo      "***************************************************************"
echo -e "\n*** Starting TurtleBot 4 Lite Simulation in Ignition Gazebo ***"
echo "ros2 launch turtlebot4_ignition_bringup ignition.launch.py model:=lite"
ros2 launch turtlebot4_ignition_bringup ignition.launch.py model:=lite world:=depot

