from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    helper_description_share = FindPackageShare('helper_description')
    helper_navigation_share = FindPackageShare('helper_navigation')
    helper_perception_share = FindPackageShare('helper_perception')
    slam_toolbox_share = FindPackageShare('slam_toolbox')

    front_lidar_port = LaunchConfiguration('front_lidar_port')
    front_lidar_baudrate = LaunchConfiguration('front_lidar_baudrate')
    motor_port = LaunchConfiguration('motor_port')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    use_sim_time = LaunchConfiguration('use_sim_time')

    slam_params_file = PathJoinSubstitution([
        helper_navigation_share,
        'config',
        'helper_slam_params.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'front_lidar_port',
            default_value=EnvironmentVariable(
                'AMR_FRONT_LIDAR_PORT',
                default_value=(
                    '/dev/serial/by-id/'
                    'usb-Silicon_Labs_CP2102_USB_to_UART_'
                    'Bridge_Controller_0001-if00-port0'
                ),
            ),
            description='Serial port for the front SLAM LiDAR.',
        ),
        DeclareLaunchArgument(
            'front_lidar_baudrate',
            default_value='460800',
            description='Serial baudrate for the front SLAM LiDAR.',
        ),
        DeclareLaunchArgument(
            'motor_port',
            default_value=EnvironmentVariable(
                'AMR_MOTOR_DRIVER_PORT',
                default_value=(
                    '/dev/serial/by-id/'
                    'usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0'
                ),
            ),
            description='Serial port for the motor driver.',
        ),
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/control/cmd_vel_test',
            description='Twist topic consumed by the motor driver.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation clock if true.',
        ),
        DeclareLaunchArgument(
            'directional_safety_enabled',
            default_value='false',
            description='Experimental swept-path safety; validate footprint before enabling.',
        ),
        DeclareLaunchArgument(
            'directional_legacy_scan_filter', default_value='false',
            description='Legacy angle/range exclusions: creates a rear blind sector.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    helper_perception_share,
                    'launch',
                    'front_lidar_slam.launch.py',
                ])
            ]),
            launch_arguments={
                'front_serial_port': front_lidar_port,
                'front_serial_baudrate': front_lidar_baudrate,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    helper_description_share,
                    'launch',
                    'display.launch.py',
                ])
            ]),
        ),
        # Continuous wheel joints need joint_states for RobotModel/TF completeness.
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),
        Node(
            package='helper_control',
            executable='motor_driver',
            name='motor_driver_node',
            output='screen',
            parameters=[{
                'serial_port': ParameterValue(motor_port, value_type=str),
                'cmd_vel_topic': ParameterValue(cmd_vel_topic, value_type=str),
                'directional_safety_enabled': ParameterValue(
                    LaunchConfiguration('directional_safety_enabled'), value_type=bool),
                'directional_legacy_scan_filter': ParameterValue(
                    LaunchConfiguration('directional_legacy_scan_filter'), value_type=bool),
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    slam_toolbox_share,
                    'launch',
                    'online_async_launch.py',
                ])
            ]),
            launch_arguments={
                'slam_params_file': slam_params_file,
                'use_sim_time': use_sim_time,
            }.items(),
        ),
    ])
