#!/bin/bash

echo -e "\n*** Send Create3 LED Spinning Action"
echo '*** ros2 action send_goal /led_animation irobot_create_msgs/action/LedAnimation "{animation_type: 2,max_runtime:{sec: 10,nanosec: 0},lightring:{override_system: true, leds: [{red: 255, green: 0, blue: 0}, {red: 0, green: 255, blue: 0}, {red: 0, green: 0, blue: 255}, {red: 255, green: 255, blue: 0}, {red: 255, green: 0, blue: 255}, {red: 0, green: 255, blue: 255}]}}"'
echo '*** Note:  type 1 is blink and type 2 is spin'
ros2 action send_goal /led_animation irobot_create_msgs/action/LedAnimation "{animation_type: 2,max_runtime:{sec: 10,nanosec: 0},lightring:{override_system: true, leds: [{red: 255, green: 0, blue: 0}, {red: 0, green: 255, blue: 0}, {red: 0, green: 0, blue: 255}, {red: 255, green: 255, blue: 0}, {red: 255, green: 0, blue: 255}, {red: 0, green: 255, blue: 255}]}}"
