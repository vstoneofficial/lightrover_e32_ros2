"""模倣学習デモの共通部分。collect.launch.py / run_policy.launch.py から読み込まれます。

  人の操作 (mouse / keyboard) -> /il/human_cmd ─┐
                                                 ├ il_cmd_mux -> /il/cmd -> twist_limiter -> /rover_twist
  il_policy (policy:=true のとき) -> /il/policy_cmd ┘
  il_recorder: /scan + /il/human_cmd を記録 (人が操作している瞬間だけ)

ロボット本体側 (micro-ROS agent と robot.launch.py) は別途起動しておいてください。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    teleop = LaunchConfiguration('teleop')
    policy = LaunchConfiguration('policy')
    record = LaunchConfiguration('record')
    data_dir = LaunchConfiguration('data_dir')
    model_path = LaunchConfiguration('model_path')
    max_linear_x = LaunchConfiguration('max_linear_x')
    max_angular_z = LaunchConfiguration('max_angular_z')
    mouse_stamped = LaunchConfiguration('mouse_stamped')
    # /il/human_cmd の型: Jazzy の mouse_teleop は TwistStamped、keyboard は Twist
    human_stamped = ParameterValue(
        PythonExpression(["'", teleop, "' == 'mouse' and '", mouse_stamped, "'.lower() == 'true'"]),
        value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument('teleop', default_value='mouse', description='mouse | keyboard'),
        DeclareLaunchArgument('policy', default_value='false'),
        DeclareLaunchArgument(
            'enabled', default_value='false',
            description='true で起動直後から AI 走行 (ros2 param set が使いにくい環境向け)'),
        DeclareLaunchArgument('record', default_value='true'),
        DeclareLaunchArgument('data_dir', default_value='~/il_data'),
        DeclareLaunchArgument('model_path', default_value='~/il_models/model.npz'),
        DeclareLaunchArgument('max_linear_x', default_value='0.10'),
        DeclareLaunchArgument('max_angular_z', default_value='1.0'),
        DeclareLaunchArgument(
            'mouse_stamped', default_value='true',
            description='mouse_teleop が TwistStamped を出すか (Jazzy は true、古い版は false)'),

        # --- 人の操作 ---
        Node(
            package='mouse_teleop', executable='mouse_teleop', name='il_mouse_teleop',
            output='screen',
            remappings=[('/mouse_vel', '/il/human_cmd')],
            condition=LaunchConfigurationEquals('teleop', 'mouse'),
        ),
        Node(
            package='teleop_twist_keyboard', executable='teleop_twist_keyboard',
            name='il_keyboard_teleop', output='screen',
            prefix='xterm -fa Monospace -fs 12 -e',
            remappings=[('/cmd_vel', '/il/human_cmd')],
            condition=LaunchConfigurationEquals('teleop', 'keyboard'),
        ),

        # --- 人 / AI の切り替え と 速度制限 ---
        Node(package='lightrover_e32_imitation', executable='il_cmd_mux', name='il_cmd_mux',
             output='screen', parameters=[{'human_stamped': human_stamped}]),
        Node(
            package='lightrover_e32_bringup', executable='lightrover_e32_twist_limiter',
            name='il_twist_limiter', output='screen',
            parameters=[{
                'input_topic': '/il/cmd',
                'output_topic': '/rover_twist',
                'max_linear_x': ParameterValue(max_linear_x, value_type=float),
                'max_angular_z': ParameterValue(max_angular_z, value_type=float),
                'publish_rate': 20.0,
                'command_timeout': 0.5,
            }],
        ),

        # --- 記録 ---
        Node(package='lightrover_e32_imitation', executable='il_recorder', name='il_recorder',
             output='screen', parameters=[{'data_dir': data_dir, 'human_stamped': human_stamped}],
             condition=IfCondition(record)),

        # --- 学習済み方策 ---
        Node(package='lightrover_e32_imitation', executable='il_policy', name='il_policy',
             output='screen', parameters=[{'model_path': model_path,
                          'enabled': ParameterValue(LaunchConfiguration('enabled'), value_type=bool)}],
             condition=IfCondition(policy)),
    ])
