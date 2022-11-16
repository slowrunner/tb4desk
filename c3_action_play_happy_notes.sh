#!/bin/bash

echo -e "\n*** Send Create3 Action Play Happy Notes"
echo '*** ros2 topic pub /cmd_audio irobot_create_msgs/msg/AudioNoteVector "{append: false, notes: [{frequency: 392, max_runtime: {sec: 0,nanosec: 177500000}}, {frequency: 523, max_runtime: {sec: 0,nanosec: 355000000}}, {frequency: 587, max_runtime: {sec: 0,nanosec: 177500000}}, {frequency: 784, max_runtime: {sec: 0,nanosec: 533000000}}]}" -1'
ros2 topic pub /cmd_audio irobot_create_msgs/msg/AudioNoteVector "{append: false, notes: [{frequency: 392, max_runtime: {sec: 0,nanosec: 177500000}}, {frequency: 523, max_runtime: {sec: 0,nanosec: 355000000}}, {frequency: 587, max_runtime: {sec: 0,nanosec: 177500000}}, {frequency: 784, max_runtime: {sec: 0,nanosec: 533000000}}]}" -1
