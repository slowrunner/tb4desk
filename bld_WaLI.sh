#!/bin/bash

# FILE: bld_WaLi.sh
# colcon build --packages-select irobot_create_msgs
colcon build --symlink-install --packages-select wali

source ~/tb4desk/install/setup.bash
