from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition  #用于控制某个模块是否启动
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node  #启动一个 ROS2 节点
from launch_ros.substitutions import FindPackageShare  #查找某个 ROS2 包安装后的 share 目录

#它负责把这些东西一起拉起来：MicroXRCEAgent, Livox ROS2 driver, Point-LIO, location_bridge

def generate_launch_description():
    default_livox_launch_file = PathJoinSubstitution([
        FindPackageShare("livox_ros_driver2"),
        "launch_ROS2",
        "msg_MID360_launch.py",  #默认使用 Livox 的 MID360 ROS2 驱动启动文件
    ])

    # 如果你实际用的是 MID360s，可以运行时覆盖 livox_launch_file：
    # ros2 launch uav_bringup slam_location.launch.py \
    #   livox_launch_file:=/home/jzh/uav_c/install/livox_ros_driver2/share/livox_ros_driver2/launch_ROS2/msg_MID360s_launch.py

    default_point_lio_config = PathJoinSubstitution([
        FindPackageShare("point_lio"),
        "config",
        "mid360.yaml",
    ])
    #启动开关参数
    start_dds_agent = LaunchConfiguration("start_dds_agent")
    start_livox = LaunchConfiguration("start_livox")
    start_point_lio = LaunchConfiguration("start_point_lio")
    start_location_bridge = LaunchConfiguration("start_location_bridge")

    dds_device = LaunchConfiguration("dds_device")
    dds_baudrate = LaunchConfiguration("dds_baudrate")
    livox_launch_file = LaunchConfiguration("livox_launch_file")
    point_lio_config = LaunchConfiguration("point_lio_config")
    point_lio_odom_topic = LaunchConfiguration("point_lio_odom_topic")
    visual_odometry_topic = LaunchConfiguration("visual_odometry_topic")

    declared_args = [  #默认全部是 true
        DeclareLaunchArgument("start_dds_agent", default_value="true"),
        DeclareLaunchArgument("start_livox", default_value="true"),
        DeclareLaunchArgument("start_point_lio", default_value="true"),
        DeclareLaunchArgument("start_location_bridge", default_value="true"),
        DeclareLaunchArgument(
            "dds_device",
            default_value="/dev/serial/by-path/platform-3610000.usb-usb-0:2.3:1.0-port0",
        ),
        DeclareLaunchArgument("dds_baudrate", default_value="921600"),
        DeclareLaunchArgument("livox_launch_file", default_value=default_livox_launch_file),
        DeclareLaunchArgument("point_lio_config", default_value=default_point_lio_config),
        DeclareLaunchArgument("point_lio_odom_topic", default_value="/aft_mapped_to_init"),
        DeclareLaunchArgument(
            "visual_odometry_topic",
            default_value="/fmu/in/vehicle_visual_odometry",
        ),
    ]

    dds_agent = ExecuteProcess(
        cmd=[
            "MicroXRCEAgent",
            "serial",
            "--dev",
            dds_device,
            "-b",
            dds_baudrate,
        ],
        name="micro_xrce_dds_agent",
        output="screen",
        condition=IfCondition(start_dds_agent),
    )

    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(livox_launch_file),
        condition=IfCondition(start_livox),
    )

    point_lio = Node(
        package="point_lio",
        executable="pointlio_mapping",
        name="laserMapping",
        output="screen",
        condition=IfCondition(start_point_lio),
        parameters=[
            point_lio_config,
            {
                "use_imu_as_input": False,
                "prop_at_freq_of_imu": True,
                "check_satu": True,
                "init_map_size": 10,
                "point_filter_num": 3,
                "space_down_sample": True,
                "filter_size_surf": 0.5,
                "filter_size_map": 0.5,
                "cube_side_length": 1000.0,
                "runtime_pos_log_enable": False,
            },
        ],
    )

    location_bridge = Node(
        package="uav_control_cpp",
        executable="location_bridge",
        name="location_bridge",
        output="screen",
        condition=IfCondition(start_location_bridge),
        parameters=[
            {"input_topic": point_lio_odom_topic},
            {"visual_odometry_topic": visual_odometry_topic},
            {"origin_x": 0.0},
            {"origin_y": 0.0},
            {"origin_z": 0.0},
            {"zero_initial_xy": True},
            {"zero_initial_z": True},
            {"publish_rate": 30.0},
            {"odometry_timeout": 0.5},
            {"position_variance": 0.03},
            {"orientation_variance": 0.05},
            {"velocity_variance": 0.05},
        ],
    )

    return LaunchDescription(
        declared_args + [
            dds_agent,
            livox_driver,
            point_lio,
            location_bridge,
        ]
    )
