"""学習済み方策でロボットを走らせるノード。

/scan -> 特徴量 -> MLP -> /il/policy_cmd

起動直後は停止状態 (enabled=false)。準備ができたら
  ros2 param set /il_policy enabled true
で走り出します。止めるときは false に戻すか、人がマウスで操作すれば優先されます。
"""

import os

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from .features import scan_to_sectors, sector_min
from .policy_model import NumpyPolicy


class PolicyNode(Node):
    def __init__(self):
        super().__init__('il_policy')
        model_path = os.path.expanduser(
            self.declare_parameter('model_path', '~/il_models/model.npz').value)
        self.enabled = self.declare_parameter('enabled', False).value
        self.smoothing = self.declare_parameter('smoothing', 0.4).value
        self.stop_distance = self.declare_parameter('stop_distance', 0.15).value
        self.speed_scale = self.declare_parameter('speed_scale', 1.0).value

        self.policy = NumpyPolicy(model_path)
        self.prev = np.zeros(2, dtype=np.float32)
        self.add_on_set_parameters_callback(self.on_params)
        self.pub = self.create_publisher(Twist, '/il/policy_cmd', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.get_logger().info(
            f'モデル読み込み: {model_path} (sectors={self.policy.n_sectors}, '
            f'max_range={self.policy.max_range}m)  enabled={self.enabled}')

    def on_params(self, params):
        for p in params:
            if p.name == 'enabled':
                self.enabled = bool(p.value)
                self.prev[:] = 0.0
                self.get_logger().info('AI走行 開始' if self.enabled else 'AI走行 停止')
            elif p.name == 'speed_scale':
                self.speed_scale = float(p.value)
        return SetParametersResult(successful=True)

    def on_scan(self, scan):
        if not self.enabled:
            return  # 何も出さなければ mux 側で停止扱いになる
        p = self.policy
        f = scan_to_sectors(scan.ranges, scan.angle_min, scan.angle_increment,
                            n_sectors=p.n_sectors, max_range=p.max_range,
                            laser_yaw=p.laser_yaw)
        a = p(f)

        # 出力を少しなめらかにする (1次ローパス)
        a = self.smoothing * self.prev + (1.0 - self.smoothing) * a
        self.prev = a
        lin, ang = float(a[0]) * self.speed_scale, float(a[1])

        # 安全装置: 正面 ±30° に近い物体があれば前進しない (学習結果に関係なく)
        front = sector_min(f, 0, 30, max_range=p.max_range)
        if front < self.stop_distance and lin > 0.0:
            lin = 0.0

        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = PolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():  # Humble には rclpy.try_shutdown() が無いため
            rclpy.shutdown()
