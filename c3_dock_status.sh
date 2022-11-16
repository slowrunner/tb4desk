#!/bin/bash

echo -e "\n*** Fetch Create3 Docking Status"
echo '*** ros2 topic echo /dock'
ros2 topic echo /dock

# Humble
# echo '*** ros2 topic echo /dock_status'
# ros2 topic echo /dock_status
