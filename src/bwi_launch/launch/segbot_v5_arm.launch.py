from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Get the package share directories
    segbot_v5_package = get_package_share_directory('bwi_launch')
    description_package = get_package_share_directory("v5_description")
    config_folder = os.path.join(get_package_share_directory('bwi_launch'), "config")
    
    # Define the robot description path
    robot_description_path = PathJoinSubstitution(
        [FindPackageShare("v5_description"), "urdf", "ur5_robotiq85_gripper.urdf.xacro"]
    )

    # Debug information
    debug_info = LogInfo(msg=["Loading URDF from: ", robot_description_path])

    # base launch file for segbot without arm
    segbot_v5_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(segbot_v5_package, 'launch', 'segbot_v5.launch.py')
        ),
        launch_arguments={"use_sim_time": "false", "publish_state": "false", "gui":"false"}.items()
    )

    # arm launch file
    v5_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            '/home/carson/v5_workspace/src/v5_arm/arm_launch/launch/v5_arm.launch.py'
        ),
        launch_arguments={"use_sim_time": "false", "robot_description": robot_description_path, "gui":"false", "stationary":"false"}.items()
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", os.path.join(config_folder, "nav_with_arm.rviz")],
    )

    return LaunchDescription([
        SetEnvironmentVariable("CUDA_VISIBLE_DEVICES", "0"),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),
        debug_info,
        segbot_v5_launch,
        v5_arm_launch,
        rviz_node
    ])
