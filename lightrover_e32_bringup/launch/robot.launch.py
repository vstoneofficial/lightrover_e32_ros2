import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def launch_setup(context):
    robot_description_path = os.path.join(
        get_package_share_directory('lightrover_e32_description'),
        'urdf',
        'lightrover_e32_robot.urdf',
    )
    rviz_config_path = os.path.join(
        get_package_share_directory('lightrover_e32_bringup'),
        'rviz',
        'laser.rviz',
    )

    with open(robot_description_path, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher_node',
            parameters=[
                {
                    'robot_description': robot_description,
                }
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='lightrover_e32_rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
        ),
        Node(
            package='lightrover_e32_bringup',
            executable='pub_odom',
            name='pub_odom',
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
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            OpaqueFunction(
                function=launch_setup,
            )
        ]
    )
