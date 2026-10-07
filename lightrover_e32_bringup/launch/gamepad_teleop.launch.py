from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gamepad_launch = PathJoinSubstitution([
        FindPackageShare('lightrover_e32_bringup'),
        'launch',
        'rover_gamepad.launch.py',
    ])

    launch_arguments = {
        'device_id': LaunchConfiguration('device_id'),
        'deadzone': LaunchConfiguration('deadzone'),
        'autorepeat_rate': LaunchConfiguration('autorepeat_rate'),
        'enable_button': LaunchConfiguration('enable_button'),
        'turbo_button': LaunchConfiguration('turbo_button'),
        'slow_button': LaunchConfiguration('slow_button'),
        'invert_linear': LaunchConfiguration('invert_linear'),
        'invert_angular': LaunchConfiguration('invert_angular'),
        'linear_axis': LaunchConfiguration('linear_axis'),
        'angular_axis': LaunchConfiguration('angular_axis'),
        'dpad_linear_axis': LaunchConfiguration('dpad_linear_axis'),
        'dpad_angular_axis': LaunchConfiguration('dpad_angular_axis'),
        'max_linear_x': LaunchConfiguration('max_linear_x'),
        'max_angular_z': LaunchConfiguration('max_angular_z'),
        'publish_rate': LaunchConfiguration('publish_rate'),
        'command_timeout': LaunchConfiguration('command_timeout'),
    }

    return LaunchDescription([
        DeclareLaunchArgument('device_id', default_value='0'),
        DeclareLaunchArgument('deadzone', default_value='0.08'),
        DeclareLaunchArgument('autorepeat_rate', default_value='20.0'),
        DeclareLaunchArgument('enable_button', default_value='4'),
        DeclareLaunchArgument('turbo_button', default_value='5'),
        DeclareLaunchArgument('slow_button', default_value='6'),
        DeclareLaunchArgument('invert_linear', default_value='false'),
        DeclareLaunchArgument('invert_angular', default_value='false'),
        DeclareLaunchArgument('linear_axis', default_value='1'),
        DeclareLaunchArgument('angular_axis', default_value='3'),
        DeclareLaunchArgument('dpad_linear_axis', default_value='7'),
        DeclareLaunchArgument('dpad_angular_axis', default_value='6'),
        DeclareLaunchArgument('max_linear_x', default_value='0.10'),
        DeclareLaunchArgument('max_angular_z', default_value='1.0'),
        DeclareLaunchArgument('publish_rate', default_value='20.0'),
        DeclareLaunchArgument('command_timeout', default_value='0.5'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gamepad_launch),
            launch_arguments=launch_arguments.items(),
        ),
    ])
