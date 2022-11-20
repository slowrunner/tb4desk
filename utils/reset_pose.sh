#!/bin/bash

echo -e "\n*** RESET POSE ***"
echo "ros2 service call reset_pose irobot_create_msgs/srv/ResetPose"
ros2 service call reset_pose irobot_create_msgs/srv/ResetPose
