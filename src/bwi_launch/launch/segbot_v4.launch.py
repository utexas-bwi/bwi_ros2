from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetRemap
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
import os
from launch.actions import TimerAction

def generate_launch_description():
    # Launch arguments for publishing state and GUI
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

    # Path to the xacro file
    xacro_file = os.path.join(
        get_package_share_directory('segbot_description'),
        'robots',
        'segbot_v4.urdf.xacro'
    )

    # Process the URDF file with xacro
    robot_desc = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', xacro_file
    ])

    config_folder = os.path.join(get_package_share_directory('bwi_launch'), "config")

    # TF transformations
    static_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_odom_transform",
        output="log",
        arguments=["--frame-id", "map", "--child-frame-id", "odom"],
    )  

    static_odom_base_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_odom_base_transform",
        output="log",
        arguments=["--frame-id", "odom", "--child-frame-id", "base_footprint"],
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_desc}],
        arguments=["--use_tf_static", "false"],
        condition=IfCondition(publish_state)
    )
    
    # Robot joint publisher
    robot_joint_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
    )

    # Segway driver
    segway_package = get_package_share_directory('segway')
    segway_controller = IncludeLaunchDescription(os.path.join(segway_package, "launch/segway_ros.launch.py"))

    # hokuyo 
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
            IncludeLaunchDescription(os.path.join(velodyne_package, "launch/velodyne-all-nodes-VLP16-launch.py")),
            SetRemap(src='/scan',dst='/velodyne/scan')
        ]
    )

    # laser filter and merger
    scan_merger = IncludeLaunchDescription(os.path.join(get_package_share_directory('ros2_laser_scan_merger'), "launch/merge_2_scan.launch.py"))
    scan_filter = Node(package="scan_filter", executable="scan_filter")

    # Navigation node with map argument
    nav2_package = get_package_share_directory('nav2_bringup')
    map_file = os.path.join(nav2_package, "maps/2/occ/ahg_full_new.yaml")
    
    # Navigation
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_package, "launch", "bringup_launch.py")),
        launch_arguments={"map": map_file}.items()
    )

    # Don't use navigation and delayed navigation together 
    # delayed_naviagtion = TimerAction(period=5.0, actions=[navigation])

    #kinect
    kinect_package = get_package_share_directory('azure_kinect_ros_driver')
    kinect_driver = IncludeLaunchDescription(os.path.join(kinect_package, "launch/driver.launch.py"), launch_arguments={"overwrite_robot_description": "false"}.items())
    delayed_kinect_driver = TimerAction(period=10.0, actions=[kinect_driver])    
    
    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", os.path.join(config_folder, "rviz_config.rviz")],
        condition=IfCondition(gui)
    )

    # Return the complete launch description
    return LaunchDescription([
        gui_arg,
        publish_state_arg,
        rviz_node,
        segway_controller,
        hokuyo_driver, 
        velodyne_driver, 
        scan_merger,      
        scan_filter,
        static_odom_tf,
        static_odom_base_tf,
        navigation,
        robot_state_publisher,
        robot_joint_publisher,
        delayed_kinect_driver,
    ])