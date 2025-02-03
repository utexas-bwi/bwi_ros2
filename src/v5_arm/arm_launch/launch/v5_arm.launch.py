from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os

def generate_launch_description():

    show_gui = LaunchConfiguration("gui")
    stationary = LaunchConfiguration("stationary")

    show_gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Flag to enable GUI'
    )
    stationary_arg = DeclareLaunchArgument(
        'stationary',
        default_value='true',
        description='Flag to enable stationary mode'
    )

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
        .sensors_3d(file_path=os.path.join(config_package, "config/sensors_kinect_pointcloud.yaml"))
        .robot_description_kinematics(os.path.join(config_package, "config/kinematics.yaml"))
        .to_moveit_configs()
    )

    move_group_capabilities = {"capabilities": "move_group/ExecuteTaskSolutionCapability"}

    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict(), move_group_capabilities],
        arguments=["--debug true"],
    )

    # Get the path to the RViz configuration file
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="moveit.rviz",
        description="RViz configuration file",
    )
    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("v5_moveit_config"), "config", rviz_base]
    )

    # Launch RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(show_gui)
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_footprint"],
        condition=IfCondition(stationary)
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        arguments=["--use_tf_static", "false"],
        parameters=[moveit_config.robot_description],
        #condition=IfCondition(stationary)
    )

    pick_place_pkg = get_package_share_directory('pick_and_place')
    pick_and_place = IncludeLaunchDescription(os.path.join(pick_place_pkg, "launch/pick_place.launch.py"))

    arm_launch_pkg = get_package_share_directory('arm_launch')
    arm_launch = IncludeLaunchDescription(os.path.join(arm_launch_pkg, "launch/arm_drivers.launch.py"))

    kinect_package = get_package_share_directory('azure_kinect_ros_driver')
    kinect_driver = IncludeLaunchDescription(os.path.join(kinect_package, "launch/combined_driver.launch.py"), launch_arguments={"overwrite_robot_description": "false"}.items())
    
    return LaunchDescription(
        [
            show_gui_arg,
            stationary_arg,
            rviz_config_arg,
            rviz_node,
            static_tf,
            robot_state_publisher,
            run_move_group_node,
            #pick_and_place,
        ]
    )
