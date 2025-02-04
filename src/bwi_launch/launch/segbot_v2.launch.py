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
        'segbot_v2.urdf.xacro'
    )

    # Process the URDF file with xacro
    # robot_desc = Command(['xacro', default_urdf_path])
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

    # static_base_tf = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="static_base_transform",
    #     output="log",
    #     arguments=["--frame-id", "base_footprint", "--child-frame-id", "base_link"],
    # )   

    # static_transform_publisher_laser = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='laser_static_tf',
    #     arguments=['0', '0', '0', '0', '0', '0', 'nav_hokuyo_laser_link', 'laser']
    # )

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

    robot_joint_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
    )

    # Segway driver
    segway_package = get_package_share_directory('segway_rmp_ros2')
    segway_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(segway_package, "launch", "segway_rmp_ros2.launch.py"))
    )

    # Path to the urg_node2 launch file
    urg_node_launch = PathJoinSubstitution([
        FindPackageShare("urg_node2"),
        "launch",
        "urg_node2.launch.py"
    ])    

    # Include the urg_node2 launch file with remapping applied
    urg_node_with_remap = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(urg_node_launch),
        ),
        SetRemap(src='/scan', dst='/hokuyo/scan')
    ])

    # Scan filter node
    scan_filter = Node(
        package="scan_filter",
        executable="scan_filter",
    )

    # Navigation node with map argument
    nav2_package = get_package_share_directory('nav2_bringup')
    
    map_file = "/home/bwidemo/bwi_ros2/src/nav2_bringup/maps/2/occ/ahg_full_closed.yaml"
    # map_file = os.path.join(nav2_package, "maps/2/occ/ahg_full_closed.yaml")
    
    # navigation = IncludeLaunchDescription(os.path.join(nav2_package, "launch/bringup_launch.py"),
    #                                          launch_arguments={"map": map_file}.items())

    # delayed_naviagtion = TimerAction(period=5.0, actions=[navigation])

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_package, "launch", "bringup_launch.py")),
        launch_arguments={"map": map_file}.items()
    )
    
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
        urg_node_with_remap,
        scan_filter,
        static_odom_tf,
        static_odom_base_tf,
        # static_transform_publisher_laser,
        # static_base_tf,
        # delayed_naviagtion,
        navigation,
        robot_state_publisher,
        robot_joint_publisher,
    ])
