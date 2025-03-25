# bwi-ros2
ROS2 workspace for building wide intelligence project, University of Texas at Austin. This workspace has all the necessary packages to run BWIBOTs v2, v4, and v5.  

## Requirements
- Ubuntu 22.04 or greater
- ROS2 humble
- serial library for ros2
- Go through the Robot startup instructions from [here](https://docs.google.com/document/d/11iZ1Vx7ReAhXJNAw5c9RzdnXDH5MWIHImsS1ARvgtaM/edit?tab=t.0) 
  
## Installation

Install ros2 humble first from [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

clone the repo:

Follow this instructions to install serial for ros2:
```
export ROS_DISTRO=humble
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
cd ~/bwi_ros2
export COLCON_WS=/path/to/your/colcon_ws/
git clone https://github.com/utexas-bwi/serial_for_ros2.git
move 'serial' out of 'serial_for_ros2' and delete 'serial_for_ros2' 
cd ~/serial
rm -rf build
mkdir build
cd build
cmake ..
make
```
** Make sure that the serial library is in _parallel to your ROS2 humble workspace_. **

Follow this instructions to download Azure-Kinect camera driver:

Follow [this](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/issues/1263#issuecomment-710698591) instructions to install Azure Kinect SDK. More details you can find over [here](https://github.com/microsoft/Azure_Kinect_ROS_Driver/blob/humble/docs/building.md).

Please set the udev-rules following [this](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/docs/usage.md#linux-device-setup).

ROS2 driver for Azure Kinect is already included in this workspace. To learn more about the driver please follow [this](https://github.com/microsoft/Azure_Kinect_ROS_Driver/tree/humble)

## Build the workspace
```
cd ~/bwi_ros2
rosdep install --from-paths src --ignore-src -r -y
colcon build
```
### Source the workspace

<code>nano ~/.bashrc </code>

Add the following line in the `.bashrc` file:
```
source /opt/ros/humble/setup.bash
```

## Run
Make sure to source the file when you open a new terminal. Example:
```
# from your [colcon ws]  
source install/setup.bash
```
**Launch segway base, sensor drivers, navigation, etc.**

<code>ros2 launch bwi_launch segbot_[BWIBOT VERSION].launch.py.</code>

BWIBOT VERSIONS: v2, v4, and v5.

Example: <code>ros2 launch bwi_launch segbot_v2.launch.py.</code>

**Teleoperation (Keyboard Control):**

ros2 run teleop_twist_keyboard teleop_twist_keyboard

**SLAM (Simultaneous Localization and Mapping):** 

<code>ros2 launch nav2_bringup slam_launch.py</code>

**Save Map:** 

<code>ros2 run nav2_map_server map_saver_cli -f my_map</code>

**Load Map with Navigation:**

Edit the ‘map_file’ param path on segbot_v2.launch.py file:
 map_file = "~/bwi_ros2/src/nav2_bringup/maps/2/occ/ahg_full_closed.yaml"
