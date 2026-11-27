"""シミュレータ起動: 実機 (micro-ROS agent + robot.launch.py) の代わりに起動します。

  ros2 launch lightrover_e32_imitation sim.launch.py                    # 四角いコース
  ros2 launch lightrover_e32_imitation sim.launch.py course:=l_shape    # L字コース
  ros2 launch lightrover_e32_imitation sim.launch.py realism:=ideal     # ノイズ・遅れなし

起動したら、別のターミナルで collect.launch.py / run_policy.launch.py を実機のときと同じように使います。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def launch_setup(context):
    share = get_package_share_directory('lightrover_e32_imitation')
    course = context.launch_configurations['course']
    realism = context.launch_configurations['realism']
    use_rviz = context.launch_configurations['rviz'].lower() == 'true'

    params_file = os.path.join(share, 'config', f'sim_{realism}.yaml')
    if not os.path.isfile(params_file):
        raise RuntimeError(f'realism:={realism} は指定できません (real または ideal)')

    urdf_path = os.path.join(get_package_share_directory('lightrover_e32_description'),
                             'urdf', 'lightrover_e32_robot.urdf')
    with open(urdf_path, encoding='utf-8') as f:
        robot_description = f.read()

    nodes = [
        Node(package='lightrover_e32_imitation', executable='il_sim', name='il_sim',
             output='screen', parameters=[params_file, {'course': course}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='il_sim_robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
    ]
    if use_rviz:
        nodes.append(Node(package='rviz2', executable='rviz2', name='il_sim_rviz2',
                          arguments=['-d', os.path.join(share, 'rviz', 'sim.rviz')]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('course', default_value='square',
                              description='square | square_wide | l_shape | obstacles | YAMLファイルのパス'),
        DeclareLaunchArgument('realism', default_value='real',
                              description='real (ノイズ・遅れあり) | ideal (理想的)'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
