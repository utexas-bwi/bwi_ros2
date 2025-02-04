from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    # Set the IP address of your robot
    arm_ip = '10.66.171.2'

    config_pkg = os.path.join(get_package_share_directory('v5_description'), "config", "ur5")
    joint_limits_yaml = os.path.join(config_pkg, "joint_limits.yaml")

    ur_driver_pkg = get_package_share_directory('ur_robot_driver')
    ur_driver_launch_file = os.path.join(ur_driver_pkg, 'launch', 'ur_control.launch.py')
    ur_control_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_driver_launch_file),
        launch_arguments={'robot_ip': arm_ip,
                          'ur_type': "ur5",
                          "launch_rviz": "false",
                          "tf_prefix":"ur5_",
                          "parent": "shoulder_base",
                          "joint_limit_params": joint_limits_yaml,
                          }.items()
    )

    gripper_control_node = Node(
        package='robotiq_gripper_driver',
        executable='gripper_driver'
    )

    # Create the launch description with all nodes
    return LaunchDescription([
        ur_control_node,
        gripper_control_node,
    ])
