from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    device_id = LaunchConfiguration('device_id')
    deadzone = LaunchConfiguration('deadzone')
    autorepeat_rate = LaunchConfiguration('autorepeat_rate')
    enable_button = LaunchConfiguration('enable_button')
    turbo_button = LaunchConfiguration('turbo_button')
    slow_button = LaunchConfiguration('slow_button')
    invert_linear = LaunchConfiguration('invert_linear')
    invert_angular = LaunchConfiguration('invert_angular')
    linear_axis = LaunchConfiguration('linear_axis')
    angular_axis = LaunchConfiguration('angular_axis')
    dpad_linear_axis = LaunchConfiguration('dpad_linear_axis')
    dpad_angular_axis = LaunchConfiguration('dpad_angular_axis')
    max_linear_x = LaunchConfiguration('max_linear_x')
    max_angular_z = LaunchConfiguration('max_angular_z')
    publish_rate = LaunchConfiguration('publish_rate')
    command_timeout = LaunchConfiguration('command_timeout')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[
            {
                'device_id': ParameterValue(device_id, value_type=int),
                'deadzone': ParameterValue(deadzone, value_type=float),
                'autorepeat_rate': ParameterValue(autorepeat_rate, value_type=float),
                'coalesce_interval': 0.001,
            }
        ],
    )

    gamepad_node = Node(
        package='lightrover_e32_bringup',
        executable='rover_gamepad',
        name='lightrover_e32_gamepad',
        output='screen',
        parameters=[
            {
                'joy_topic': '/joy',
                'output_topic': '/lightrover_e32_teleop_raw',
                'linear_axis': ParameterValue(linear_axis, value_type=int),
                'angular_axis': ParameterValue(angular_axis, value_type=int),
                'dpad_linear_axis': ParameterValue(dpad_linear_axis, value_type=int),
                'dpad_angular_axis': ParameterValue(dpad_angular_axis, value_type=int),
                'enable_button': ParameterValue(enable_button, value_type=int),
                'turbo_button': ParameterValue(turbo_button, value_type=int),
                'slow_button': ParameterValue(slow_button, value_type=int),
                'invert_linear': ParameterValue(invert_linear, value_type=bool),
                'invert_angular': ParameterValue(invert_angular, value_type=bool),
                'deadzone': ParameterValue(deadzone, value_type=float),
                'normal_scale': 1.0,
                'turbo_scale': 1.0,
                'slow_scale': 0.35,
                'publish_rate': ParameterValue(publish_rate, value_type=float),
                'command_timeout': ParameterValue(command_timeout, value_type=float),
            }
        ],
    )

    twist_limiter_node = Node(
        package='lightrover_e32_bringup',
        executable='lightrover_e32_twist_limiter',
        name='lightrover_e32_gamepad_twist_limiter',
        output='screen',
        parameters=[
            {
                'input_topic': '/lightrover_e32_teleop_raw',
                'output_topic': '/rover_twist',
                'max_linear_x': ParameterValue(max_linear_x, value_type=float),
                'max_angular_z': ParameterValue(max_angular_z, value_type=float),
                'publish_rate': ParameterValue(publish_rate, value_type=float),
                'command_timeout': ParameterValue(command_timeout, value_type=float),
            }
        ],
    )

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
        joy_node,
        gamepad_node,
        twist_limiter_node,
    ])
