from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    rviz_config_path = get_package_share_path('lightrover_e32_bringup') / 'rviz/laser.rviz'
    urdf_path = get_package_share_path('lightrover_e32_description') / 'urdf/lightrover_e32_robot.urdf'

    use_rviz = LaunchConfiguration('use_rviz')

    robot_description = ParameterValue(urdf_path.read_text(), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz2 with the laser view configuration.',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher_node',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='lightrover_e32_bringup',
            executable='pub_odom',
            name='pub_odom',
            parameters=[
                {
                    'odom_frame': 'odom',
                    'base_frame': 'base_link',
                    'linear_scale': 1.0,
                    'angular_scale': 1.0,
                }
            ],
        ),
        Node(
            package='lightrover_e32_bringup',
            executable='lidar_raw_to_scan',
            name='lidar_raw_to_scan',
            parameters=[
                {
                    'input_topic_0': '/rover_lidar_raw_0',
                    'input_topic_1': '/rover_lidar_raw_1',
                    'output_topic': '/scan',
                    'frame_id': 'laser_frame',
                    'scan_points': 360,
                    'chunk_points': 180,
                    'scan_time': 0.125,
                    'stamp_offset': -0.125,
                    'reverse': True,
                }
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='lightrover_e32_rviz2',
            output='screen',
            arguments=['-d', str(rviz_config_path)],
            condition=IfCondition(use_rviz),
        ),
    ])
