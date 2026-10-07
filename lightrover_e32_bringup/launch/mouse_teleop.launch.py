from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    max_linear_x = LaunchConfiguration('max_linear_x')
    max_angular_z = LaunchConfiguration('max_angular_z')
    publish_rate = LaunchConfiguration('publish_rate')
    command_timeout = LaunchConfiguration('command_timeout')

    mouse_teleop_node = Node( 
        package='mouse_teleop', 
        executable='mouse_teleop', 
        name='mouse_teleop',
        output='screen',
        remappings=[('/mouse_vel', '/lightrover_e32_teleop_raw')]
    )

    twist_limiter_node = Node(
        package='lightrover_e32_bringup',
        executable='lightrover_e32_twist_limiter',
        name='lightrover_e32_mouse_twist_limiter',
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
        DeclareLaunchArgument('max_linear_x', default_value='0.10'),
        DeclareLaunchArgument('max_angular_z', default_value='1.0'),
        DeclareLaunchArgument('publish_rate', default_value='20.0'),
        DeclareLaunchArgument('command_timeout', default_value='0.5'),
        mouse_teleop_node,
        twist_limiter_node,
    ])
