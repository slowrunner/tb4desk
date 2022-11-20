#!/bin/bash

echo -e "\n*** Send Create3 Undock Action"
echo '*** ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"'
ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"
