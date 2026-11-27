"""デモデータ記録ノード。

/scan を受け取るたびに、その時点の「人の操作」とペアにして保存します。
保存されるのは人が実際に操作している (ゼロでない指令を出している) 瞬間だけです。

  - デモ収集時: 人が運転した区間がそのまま学習データになる
  - AI走行時  : 人が割り込んで修正した区間だけが記録される (DAgger の追加データ)

出力: <data_dir>/demo_YYYYmmdd_HHMMSS.npz
  ranges          (N, P) 生の距離データ[m] (前処理を後から変えられるように生データで保存)
  actions         (N, 2) [linear.x, angular.z]
  stamps          (N,)   時刻[s]
  intervention    (N,)   AI走行中の割り込みなら True
  angle_min, angle_increment, laser_yaw
"""

import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from .cmd_mux import is_nonzero, subscribe_human_cmd
from .features import DEFAULT_LASER_YAW, scan_to_sectors, sector_min


class Recorder(Node):
    def __init__(self):
        super().__init__('il_recorder')
        data_dir = self.declare_parameter('data_dir', '~/il_data').value
        self.recording = self.declare_parameter('recording', True).value
        self.laser_yaw = self.declare_parameter('laser_yaw', DEFAULT_LASER_YAW).value
        self.cmd_timeout = self.declare_parameter('cmd_timeout', 0.3).value
        save_interval = self.declare_parameter('save_interval', 15.0).value

        self.data_dir = os.path.expanduser(data_dir)
        os.makedirs(self.data_dir, exist_ok=True)
        self.path = os.path.join(self.data_dir, time.strftime('demo_%Y%m%d_%H%M%S.npz'))

        self.ranges, self.actions, self.stamps, self.interv = [], [], [], []
        self.scan_meta = None
        self.human = Twist()
        self.human_t = None
        self.policy_t = None
        self.saved_n = 0

        self.add_on_set_parameters_callback(self.on_params)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        subscribe_human_cmd(self, self.on_human)
        self.create_subscription(Twist, '/il/policy_cmd', self.on_policy, 10)
        self.create_timer(save_interval, self.save)
        self.create_timer(2.0, self.status)
        self.get_logger().info(f'記録先: {self.path}')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_params(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'recording':
                self.recording = bool(p.value)
                self.get_logger().info('記録 再開' if self.recording else '記録 一時停止')
        return SetParametersResult(successful=True)

    def on_human(self, msg):
        self.human = msg
        self.human_t = self.now()

    def on_policy(self, _msg):
        self.policy_t = self.now()

    def on_scan(self, scan):
        meta = (float(scan.angle_min), float(scan.angle_increment), len(scan.ranges))
        if self.scan_meta is None:
            self.scan_meta = meta
        elif meta != self.scan_meta:
            self.get_logger().warn('スキャンの点数・角度設定が途中で変わったため、このスキャンは捨てます')
            return
        self.last_scan = scan

        t = self.now()
        human_fresh = self.human_t is not None and t - self.human_t < self.cmd_timeout
        if not (self.recording and human_fresh and is_nonzero(self.human)):
            return
        policy_alive = self.policy_t is not None and t - self.policy_t < 0.5

        self.ranges.append(np.asarray(scan.ranges, dtype=np.float32))
        self.actions.append([self.human.linear.x, self.human.angular.z])
        self.stamps.append(t)
        self.interv.append(policy_alive)

    def status(self):
        scan = getattr(self, 'last_scan', None)
        if scan is None:
            self.get_logger().warn('/scan がまだ届いていません')
            return
        f = scan_to_sectors(scan.ranges, scan.angle_min, scan.angle_increment,
                            n_sectors=36, max_range=8.0, laser_yaw=self.laser_yaw)
        # 向きの確認用: 手で前・左・右に物を置いて、値が小さくなる方向が正しいか確かめる
        self.get_logger().info(
            f'samples={len(self.actions)}  '
            f'前={sector_min(f, 0, 15, max_range=8.0):.2f}m '
            f'左={sector_min(f, 90, 15, max_range=8.0):.2f}m '
            f'右={sector_min(f, -90, 15, max_range=8.0):.2f}m')

    def save(self):
        n = len(self.actions)
        if n == 0 or n == self.saved_n:
            return
        angle_min, angle_inc, _ = self.scan_meta
        tmp = self.path + '.tmp.npz'
        np.savez_compressed(
            tmp,
            ranges=np.stack(self.ranges),
            actions=np.asarray(self.actions, dtype=np.float32),
            stamps=np.asarray(self.stamps),
            intervention=np.asarray(self.interv, dtype=bool),
            angle_min=angle_min, angle_increment=angle_inc, laser_yaw=self.laser_yaw)
        os.replace(tmp, self.path)
        self.saved_n = n
        self.get_logger().info(f'{n} サンプル保存 -> {self.path}')


def main():
    rclpy.init()
    node = Recorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():  # Humble には rclpy.try_shutdown() が無いため
            rclpy.shutdown()
