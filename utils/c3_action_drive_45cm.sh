#!/bin/bash

# FILE: c3_action_drive_45cm.sh
ros2 action send_goal /drive_distance irobot_create_msgs/action/DriveDistance \
"distance: 0.450 max_translation_speed: 0.1"
