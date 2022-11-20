#!/bin/bash

echo -e "\n*** Send Create3 Action Drive 0.5 Meters at 0.15 m/s"
echo '*** ros2 action send_goal /drive_distance irobot_create_msgs/action/DriveDistance "{distance: 0.5,max_translation_speed: 0.15}"'
ros2 action send_goal /drive_distance irobot_create_msgs/action/DriveDistance "{distance: 0.5,max_translation_speed: 0.15}"
