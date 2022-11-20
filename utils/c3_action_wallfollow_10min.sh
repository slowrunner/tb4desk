#!/bin/bash

echo -e "\n*** Send Create3 Wall Follow for 10 minutes Action"
echo '*** ros2 action send_goal /wall_follow irobot_create_msgs/action/WallFollow "{follow_side: 1, max_runtime: {sec: 600, nanosec: 0}}"'
ros2 action send_goal /wall_follow irobot_create_msgs/action/WallFollow "{follow_side: 1, max_runtime: {sec: 600, nanosec: 0}}"
