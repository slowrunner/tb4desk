#!/usr/bin/env python3

# FILE: wali_node.py

"""
    Make WaLI A 24/7 Autonomous Robot
    - Logs WaLI_node started to life.log
    - subscribes to Create3 /battery_state and /dock (_status)
    - publishes /undock action goal when BatteryState.percentage = 1.0
    - Logs successful undock to life.log
    - publishes /rotate_angle {angle: 1.57} (180deg) when BatteryState.percentage < 25%
    - publishes /dock action goal when BatteryState.percentage < 0.20
    - Logs successful docking to life.log

    Requires:
    - /home/ubuntu/tb4rpi/life.log with 666 permission

    Message Formats:

    - irobot_create_msgs/msg/DockStatus.msg
      std_msgs/Header header  # Header stamp is when dock info was queried.
      bool dock_visible       # Whether robot sees dock in its sensors used for docking
      bool is_docked          # Whether the robot is docked

    - sensor_msgs/msg/BatteryState
      std_msgs/Header  header
      	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
      float32 voltage          # Voltage in Volts (Mandatory)
      float32 temperature      # Temperature in Degrees Celsius (If unmeasured NaN)
      float32 current          # Negative when discharging (A)  (If unmeasured NaN)
      float32 charge           # Current charge in Ah  (If unmeasured NaN)
      float32 capacity         # Capacity in Ah (last full capacity)  (If unmeasured NaN)
      float32 design_capacity  # Capacity in Ah (design capacity)  (If unmeasured NaN)
      float32 percentage       # Charge percentage on 0 to 1 range  (If unmeasured NaN)
      ... and more

"""

import rclpy
import math
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data

import sys
from sensor_msgs.msg import BatteryState       # (percentage: 0.0 - 1.0)
# from y.msg import DockStatus                 # (docked: true,false)
import logging
import datetime as dt
from rclpy.time import Time

DEBUG = False
LIFELOGFILE = "/home/ubuntu/tb4rpi/life.log"

# Uncomment for debug prints to console
DEBUG = True

class WaLINode(Node):

  def __init__(self):
    super().__init__('wali_node')

    self.lifeLog = logging.getLogger(__name__)
    self.lifeLog.setLevel(logging.INFO)

    self.loghandler = logging.FileHandler(LIFELOGFILE)
    self.logformatter = logging.Formatter('%(asctime)s|%(filename)s.%(funcName)s %(message)s',"%Y-%m-%d %H:%M")
    self.loghandler.setFormatter(self.logformatter)
    self.lifeLog.addHandler(self.loghandler)

    printMsg = 'WaLI node started'
    print(printMsg)
    self.lifeLog.info(printMsg)


    self.sub = self.create_subscription(
      BatteryState,
      'battery_state',
      self.battery_state_sub_callback,
      # pick one from following- explicit or named profile
      # QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
      qos_profile_sensor_data)  # best effort depth 10 sensor profile
    if DEBUG:
      printMsg = '\n*** /battery_state subscriber created'

    self.battery_state = None


  def battery_state_sub_callback(self,battery_state_msg):
    self.battery_state = battery_state_msg
    if DEBUG:
      printMsg = "battery_state.percentage {:.0f} %".format(100 * self.battery_state.percentage)
      print(printMsg)
      # self.lifeLog.info(printMsg)

def main():
  rclpy.init(args=None)
  wali_node = WaLINode()
  try:
    rclpy.spin(wali_node)
  except KeyboardInterrupt:
    pass
  except ExternalShutdownException:
    sys.exit(0)
  finally:
    wali_node.destroy_node()
    try:
      rclpy.try_shutdown()
    except:
      pass


if __name__ == '__main__':
    main()
