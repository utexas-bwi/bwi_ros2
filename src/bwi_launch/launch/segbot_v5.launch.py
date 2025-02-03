from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetRemap
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.conditions import IfCondition
import os
from launch.actions import TimerAction

def generate_launch_description():

    publish_state = LaunchConfiguration('publish_state')
    publish_state_arg = DeclareLaunchArgument(
        'publish_state',
        default_value='true',
        description='use robot state publisher'
    )

    gui = LaunchConfiguration('gui')
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='use gui'
    )

    description_package = get_package_share_directory("v5_description")
    default_urdf_path = os.path.join(description_package, "urdf/v5_robot.urdf.xacro")
    robot_desc = Command(['xacro ', default_urdf_path])

    config_folder = os.path.join(get_package_share_directory('bwi_launch'), "config")

    #TFs
    static_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "map", "--child-frame-id", "odom"],
    )

    static_base_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "base_footprint", "--child-frame-id", "base_link"],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_desc}],
        arguments=["--use_tf_static", "false"],
        condition=IfCondition(publish_state)
    )
    
    #segway driver
    segway_package = get_package_share_directory('segway')
    segway_controller = IncludeLaunchDescription(os.path.join(segway_package, "launch/segway_ros.launch.py"))

    #lidar
    hokuyo_package = get_package_share_directory('urg_node2')
    hokuyo_driver = GroupAction(
        actions=[
            SetRemap(src='/scan',dst='/hokuyo/scan'),
            IncludeLaunchDescription(os.path.join(hokuyo_package, "launch/urg_node2.launch.py"))
        ]
    )

    #velodyne
    velodyne_package = get_package_share_directory('velodyne')
    velodyne_driver = GroupAction(
        actions=[
            SetRemap(src='/scan',dst='/velodyne/scan'),
            IncludeLaunchDescription(os.path.join(velodyne_package, "launch/velodyne-all-nodes-VLP16-launch.py"))
        ]
    )

    #laser filter and merger
    scan_merger = IncludeLaunchDescription(os.path.join(get_package_share_directory('ros2_laser_scan_merger'), "launch/merge_2_scan.launch.py"))
    scan_filter = Node(package="scan_filter", executable="scan_filter")
    delayed_merger = TimerAction(period=5.0, actions=[scan_merger])

    #naviagtion
    nav2_package = get_package_share_directory('nav2_bringup')
    map_file = os.path.join(nav2_package, "maps/2/occ/ahg_full_closed.yaml")
    navigation = IncludeLaunchDescription(os.path.join(nav2_package, "launch/bringup_launch.py"),
                                             launch_arguments={"map": map_file}.items())
    delayed_navigation = TimerAction(period=5.0, actions=[navigation])

    #kinect
    kinect_package = get_package_share_directory('azure_kinect_ros_driver')
    kinect_driver = IncludeLaunchDescription(os.path.join(kinect_package, "launch/combined_driver.launch.py"), launch_arguments={"overwrite_robot_description": "false"}.items())
    delayed_kinect_driver = TimerAction(period=10.0, actions=[kinect_driver])

    # Launch RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", os.path.join(config_folder, "rviz_config.rviz")],
        condition=IfCondition(gui)
    )

    return LaunchDescription(
        [
            gui_arg,
            publish_state_arg,
            rviz_node,
            segway_controller,
            hokuyo_driver,
            velodyne_driver,
            delayed_merger,
            scan_filter,
            static_odom_tf,
            navigation,
            robot_state_publisher,
            delayed_kinect_driver
        ]
    )
