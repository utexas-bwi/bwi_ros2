# ros2segway

ROS2 package to drive a segway rmp base. Establishes a connection to the segway via USB or ethernet. Accepts twist messages on the topic /cmd_vel and translates to movement commands to drive the base.
ROS2 version of original ROS1 package at https://github.com/utexas-bwi/segway_v3/tree/master/segway_ros/src/segway

## Install
```
cd catkin_ws/src
git clone https://github.com/Living-With-Robots-Lab/ros2segway.git
git clone https://github.com/utexas-bwi/segway_msgs.git
cd ..
colcon build
source install/setup.bash
```

## Usage
Ensure that the base is on, cables are connected, and ethernet settings are properly configured.
```
ros2 launch segway segway_ros.launch.py
```

To teleop:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

