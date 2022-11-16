#!/bin/bash

echo -e "\n*** Send Create3 Rotate 90 deg at 45 deg/sec Action"
echo '*** ros2 action send_goal /rotate_angle irobot_create_msgs/action/RotateAngle "{angle: 1.57,max_rotation_speed: 0.7854}"'
ros2 action send_goal /rotate_angle irobot_create_msgs/action/RotateAngle "{angle: 1.57,max_rotation_speed: 0.7854}"
