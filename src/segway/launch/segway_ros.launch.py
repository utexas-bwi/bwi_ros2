from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
# from moveit_configs_utils import MoveItConfigsBuilder
import os

def generate_launch_description():

    hardware_interface = Node(
        package="segway",
        executable="segway_hardware_interface"
    )

    controller = Node(
        package="segway",
        executable="segway_controller"
    )

    return LaunchDescription(
        [
            hardware_interface,
            controller
        ]
    )