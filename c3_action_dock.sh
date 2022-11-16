#!/bin/bash

echo -e "\n*** Send Create3 Dock Action"
echo '*** ros2 action send_goal /undock irobot_create_msgs/action/DockServo "{}"'
ros2 action send_goal /dock irobot_create_msgs/action/DockServo "{}"

# Humble
# echo '*** ros2 action send_goal /undock irobot_create_msgs/action/Dock "{}"'
# ros2 action send_goal /dock irobot_create_msgs/action/Dock "{}"
