# bwi-ros2
ROS2 workspace for building wide intelligence project, University of Texas at Austin. This workspace has all the necessary packages to run BWIBOTs V2, v4, and v5.  

## Requirements
- Ubuntu 22.04 or greater
- ROS2 humble
- serial library for ros2
- bwi-ros2 workspace
  
## Installation

Install ros2 humble first from [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

Follow this instructions to install serial for ros2:
```
cd ~/bwi-ros2
export COLCON_WS=/path/to/your/colcon_ws/
git clone https://github.com/utexas-bwi/serial_for_ros2.git
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

## Run
