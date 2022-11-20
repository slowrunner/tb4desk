#!/bin/bash

echo -e "\n*** ros2 topic echo /odom"
ros2 topic echo /odom  | grep [xy]
