# Setup Desktop Environment For Turtlebot4 and Create3

Ref: https://turtlebot.github.io/turtlebot4-user-manual/overview/quick_start.html
Ref: https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_packages.html
Ref: https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_packages.html#turtlebot-4-desktop
Ref: https://docs.ros.org/en/galactic/Tutorials/Intermediate/Rosdep.html





1) ==== Install TurtleBot 4 packages
```
sudo apt update
sudo apt install ros-galactic-turtlebot4-description \
ros-galactic-turtlebot4-msgs \
ros-galactic-turtlebot4-navigation \
ros-galactic-turtlebot4-node
```
2) ==== Install TurtleBot 4 Desktop
```
sudo apt install ros-galactic-turtlebot4-desktop
```

3) ==== Install TurtleBot 4 Simulator

* Install Dev Tools (probably already present)
```
sudo apt install -y \
python3-colcon-common-extensions \
python3-rosdep \
python3-vcstool
```

* Install sim packages
```
sudo apt install ros-galactic-turtlebot4-simulator ros-galactic-irobot-create-nodes
```

4) === Setup ROSDEP
```
sudo rosdep init
rosdep update
```

5) === BULDING A TURTLEBOT4 Node (My node is WaLI: Wall follower Looking for Intelligence)

make sure to include in ```<workspace>/src/<package>/package.xml```:  
```
  <depend>rclpy</depend>
  <depend>irobot_create_msgs</depend>
```
- run rosdep:  
```
rosdep install --from-paths src -y --ignore-src  
```
- do build  
```
colcon build --symlink-install --packages-select wali  
```
- add to environment  
```
source ~/tb4desk/install/setup.bash  
```
- run the node:  
```
ros2 run wali wali_node
```

This is the log of WaLi node running successfully today:
```
2022-11-23 22:03|------------ boot ------------
2022-11-24 06:58|wali_node.py| WaLI node started
2022-11-24 06:59|wali_node.py| ** WaLI Undocking: success at battery 99% **
2022-11-24 08:42|wali_node.py| ** WaLi Docking: success at battery 24% **
2022-11-24 08:54|lifelogger.dEmain| execution: 10.85
```
