#!/bin/bash

# FILE:  clear_ros_logs.sh

echo -e "\n*** Clearing ALL ROS Logs from /home/ubuntu/.ros/log/"
rm -r -I /home/ubuntu/.ros/log/*
