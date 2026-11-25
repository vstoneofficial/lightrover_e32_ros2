"""デモ収集: 人がマウスで運転し、/scan と操作を記録する。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    base = os.path.join(get_package_share_directory('lightrover_e32_imitation'),
                        'launch', 'imitation_base.launch.py')
    # teleop:=, data_dir:= などコマンドラインの引数はそのまま base に引き継がれる
    args = {}
    args['policy'] = 'false'
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(base),
                                 launch_arguments=args.items()),
    ])
