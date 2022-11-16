#!/bin/bash

echo -e "\n*** Send Create3 Wall Follow 60s Action"
echo '*** ros2 action send_goal /wall_follow irobot_create_msgs/action/WallFollow "{follow_side: 1, max_runtime: {sec: 60, nanosec: 0}}"'
ros2 action send_goal /wall_follow irobot_create_msgs/action/WallFollow "{follow_side: 1, max_runtime: {sec: 60, nanosec: 0}}"
