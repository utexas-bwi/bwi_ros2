from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os

def generate_launch_description():

    config_package = get_package_share_directory("v5_moveit_config")

    # Load the robot configuration
    moveit_config = (
        MoveItConfigsBuilder(
            "v5", package_name="v5_moveit_config"
        )
        .robot_description()
        .trajectory_execution(file_path=os.path.join(config_package, "config/moveit_controllers.yaml"))
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .joint_limits(file_path=os.path.join(config_package, "config/joint_limits.yaml"))
        .robot_description_kinematics(os.path.join(config_package, "config/kinematics.yaml"))
        .to_moveit_configs()
    )

    pick_place_node = Node(
        package="pick_and_place",
        executable="pick_place_node.py",
    )

    pick_exec_task = Node(
        package="pick_and_place",
        executable="pick_place_exec_node",
        output="screen",
        parameters=[
            moveit_config.joint_limits,
            moveit_config.planning_pipelines,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    object_approximator = Node(
        package="pick_and_place",
        executable="object_approximator_node",
    )

    cloud_publisher = Node(
        package="pick_and_place",
        executable="scene_cloud_publisher_node",
    )

    tracked_objects = Node(
        package="pick_and_place",
        executable="update_tracked_objects_node",
    )

    april_tag_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("apriltag_ros"), "launch", "tag_kinect.launch.py"]),
        )
    )


    return LaunchDescription([
        #pick_place_node,
        pick_exec_task,
        object_approximator,
        cloud_publisher,
        april_tag_launch,
        tracked_objects,
    ])
