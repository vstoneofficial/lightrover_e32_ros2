"""ライトローバーE32 の簡易シミュレータノード (il_sim)。

実機 (micro-ROS agent + robot.launch.py) の代わりに起動します。

  /rover_twist (Twist)  ──>  il_sim  ──>  /scan (LaserScan: 実機と同じ形式)
                                     ──>  /odom, TF odom -> base_link
                                     ──>  /il_sim/course (壁の表示), /il_sim/path (軌跡)

実機と同じトピック・同じ形式なので、collect.launch.py / run_policy.launch.py は
そのまま使えます。シミュレータで集めたデータで学習したモデルを、実機でも走らせられます。

「現実らしさ」のパラメータ (起動中に ros2 param set で変更可):
  range_noise   LiDAR の距離ノイズ (標準偏差[m])
  dropout       LiDAR の計測抜けの割合 (0〜1)
  range_min     これより近い物は測れない[m] (E32 の LiDAR は 0.12)
  cmd_delay     指令が届いて動き出すまでの遅れ[s] (Wi-Fi 通信など)
  linear_gain   指令に対する実際の速度の比 (電池が減ると 1 より小さくなるイメージ)
  angular_gain  旋回の比 (床の滑りなど)
  motion_noise  動きのばらつき (指令に対する相対値)

サービス:
  ros2 service call /il_sim/reset std_srvs/srv/Empty   # スタート位置に戻す・統計をリセット
"""

import collections
import math
import os

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from .courses import clearance, load_course, raycast, winding_delta

# 起動中に変更できる「現実らしさ」のパラメータと既定値 (実機に近づけた値)
TUNABLE = {
    'range_noise': 0.01,
    'dropout': 0.02,
    'range_min': 0.12,
    'range_max': 8.0,
    'cmd_delay': 0.10,
    'linear_gain': 1.0,
    'angular_gain': 1.0,
    'motion_noise': 0.05,
    'cmd_timeout': 0.5,
    'robot_radius': 0.07,
}


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class SimNode(Node):
    def __init__(self):
        super().__init__('il_sim')
        share = get_package_share_directory('lightrover_e32_imitation')
        self.course_dirs = [os.path.join(share, 'courses')]

        course_name = self.declare_parameter('course', 'square').value
        self.scan_rate = self.declare_parameter('scan_rate', 8.0).value
        physics_rate = self.declare_parameter('physics_rate', 50.0).value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.laser_frame = self.declare_parameter('laser_frame', 'laser_frame').value
        seed = self.declare_parameter('seed', 0).value
        for k, v in TUNABLE.items():
            setattr(self, k, float(self.declare_parameter(k, v).value))

        self.rng = np.random.default_rng(seed if seed else None)
        self.course = load_course(course_name, self.course_dirs)
        self.cmd_buf = collections.deque()
        self.reset_state()

        self.add_on_set_parameters_callback(self.on_params)
        self.create_subscription(Twist, '/rover_twist', self.on_cmd, 10)
        self.pub_scan = self.create_publisher(LaserScan, '/scan', 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_course = self.create_publisher(MarkerArray, '~/course', 1)
        self.pub_path = self.create_publisher(Path, '~/path', 1)
        self.tf = TransformBroadcaster(self)
        self.create_service(Empty, '~/reset', self.on_reset)

        self.create_timer(1.0 / physics_rate, self.step)
        self.create_timer(1.0 / self.scan_rate, self.publish_scan)
        self.create_timer(1.0, self.publish_course)
        self.create_timer(0.5, self.publish_path)
        self.create_timer(10.0, self.log_stats)
        self.log_course()

    # ------------------------------------------------------------------ 状態
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def reset_state(self):
        self.x, self.y, self.yaw = self.course.start
        self.v = self.w = 0.0
        self.winding = 0.0
        self.laps_logged = 0
        self.collisions = 0
        self.in_contact = False
        self.distance = 0.0
        self.t_start = self.now()
        self.t_last = None
        self.cmd_buf.clear()
        self.path = []
        self.last_cmd = None       # (受信時刻, v, w): 止まったときの原因の切り分け用
        self.pose_stamp = None     # 最後にロボットの位置を更新した時刻 (スキャンの時刻に使う)

    def log_course(self):
        c = self.course
        self.get_logger().info(
            f'コース: {c.name}  {c.description}  '
            f'(スタート x={c.start[0]:.2f} y={c.start[1]:.2f} 向き={math.degrees(c.start[2]):.0f}°)')

    # ---------------------------------------------------------- コールバック
    def on_params(self, params):
        for p in params:
            if p.name == 'course':
                try:
                    course = load_course(p.value, self.course_dirs)
                except Exception as e:  # noqa: BLE001
                    return SetParametersResult(successful=False, reason=str(e))
                self.course = course
                self.reset_state()
                self.log_course()
                self.publish_course()
            elif p.name in TUNABLE:
                setattr(self, p.name, float(p.value))
                self.get_logger().info(f'{p.name} = {p.value}')
        return SetParametersResult(successful=True)

    def on_cmd(self, msg):
        t = self.now()
        self.cmd_buf.append((t, msg.linear.x, msg.angular.z))
        self.last_cmd = (t, msg.linear.x, msg.angular.z)

    def on_reset(self, _req, res):
        self.reset_state()
        self.get_logger().info('リセットしました (スタート位置に戻しました)')
        return res

    # ---------------------------------------------------------------- 物理
    def delayed_cmd(self, t):
        """cmd_delay だけ遅れて届いた指令を返す。古すぎる指令は停止扱い。"""
        t_eff = t - self.cmd_delay
        buf = self.cmd_buf
        while len(buf) > 1 and buf[1][0] <= t_eff:
            buf.popleft()
        if not buf or buf[0][0] > t_eff or t_eff - buf[0][0] > self.cmd_timeout:
            return 0.0, 0.0
        return buf[0][1], buf[0][2]

    def step(self):
        t = self.now()
        if self.t_last is None:
            self.t_last = t
            return
        dt = min(t - self.t_last, 0.1)
        self.t_last = t

        v_cmd, w_cmd = self.delayed_cmd(t)
        v = v_cmd * self.linear_gain
        w = w_cmd * self.angular_gain
        if self.motion_noise > 0.0 and (v != 0.0 or w != 0.0):
            v *= 1.0 + self.rng.normal(0.0, self.motion_noise)
            w *= 1.0 + self.rng.normal(0.0, self.motion_noise)

        yaw_mid = self.yaw + w * dt / 2.0
        nx = self.x + v * math.cos(yaw_mid) * dt
        ny = self.y + v * math.sin(yaw_mid) * dt
        walls = self.course.walls
        c_old = clearance(walls, self.x, self.y)
        c_new = clearance(walls, nx, ny)

        # 壁にめり込む動きは止める (離れる方向と、その場旋回はできる)
        blocked = c_new < self.robot_radius and c_new <= c_old
        if blocked:
            # 壁から離れるまでは 1 回の衝突として数える
            if not self.in_contact:
                self.collisions += 1
                self.get_logger().warn(f'壁に衝突しました (累計 {self.collisions} 回)')
            self.in_contact = True
            v = 0.0
        else:
            self.winding += winding_delta(self.course.lap_center, self.x, self.y, nx, ny)
            self.distance += math.hypot(nx - self.x, ny - self.y)
            self.x, self.y = nx, ny
            if c_new >= self.robot_radius + 0.01:
                self.in_contact = False
        self.yaw = (self.yaw + w * dt + math.pi) % (2.0 * math.pi) - math.pi
        self.v, self.w = v, w

        laps = int(abs(self.winding) // (2.0 * math.pi))
        if laps > self.laps_logged:
            self.laps_logged = laps
            self.get_logger().info(
                f'{laps} 周 ({t - self.t_start:.0f} 秒, 衝突 {self.collisions} 回)')

        if not self.path or math.hypot(self.x - self.path[-1][0], self.y - self.path[-1][1]) > 0.01:
            self.path.append((self.x, self.y))
            del self.path[:-3000]

        self.publish_odom()

    # ---------------------------------------------------------------- 出力
    def publish_odom(self):
        stamp = self.get_clock().now().to_msg()
        self.pose_stamp = stamp
        q = yaw_to_quat(self.yaw)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        (tf.transform.rotation.x, tf.transform.rotation.y,
         tf.transform.rotation.z, tf.transform.rotation.w) = q
        self.tf.sendTransform(tf)

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = self.odom_frame
        od.child_frame_id = self.base_frame
        od.pose.pose.position.x = self.x
        od.pose.pose.position.y = self.y
        (od.pose.pose.orientation.x, od.pose.pose.orientation.y,
         od.pose.pose.orientation.z, od.pose.pose.orientation.w) = q
        od.twist.twist.linear.x = self.v
        od.twist.twist.angular.z = self.w
        self.pub_odom.publish(od)

    def publish_scan(self):
        r = raycast(self.course.walls, self.x, self.y, self.yaw)
        n = r.shape[0]
        if self.range_noise > 0.0:
            r = r + self.rng.normal(0.0, self.range_noise, n)
        if self.dropout > 0.0:
            r[self.rng.random(n) < self.dropout] = np.inf
        r[(r < self.range_min) | (r > self.range_max)] = np.inf

        inc = 2.0 * math.pi / n
        msg = LaserScan()
        # スキャンは「最後に位置を更新した瞬間」のロボット位置から計算しているので、時刻もそれに揃える。
        # (現在時刻にすると、まだ TF が出ていない時刻を RViz が問い合わせて警告が出る)
        msg.header.stamp = self.pose_stamp or self.get_clock().now().to_msg()
        msg.header.frame_id = self.laser_frame
        msg.angle_min = 0.0
        msg.angle_max = (n - 1) * inc
        msg.angle_increment = inc
        msg.scan_time = 1.0 / self.scan_rate
        # シミュレータは全方向を同じ瞬間に測るので 0。0 以外にすると、RViz が各点を
        # 「スキャン開始から少しずつ後の時刻」で変換しようとして、未来の TF を問い合わせてしまう
        msg.time_increment = 0.0
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        msg.ranges = r.astype(np.float32).tolist()
        self.pub_scan.publish(msg)

    def publish_course(self):
        stamp = self.get_clock().now().to_msg()
        h = 0.10   # 壁の高さ (表示用)

        walls = Marker()
        walls.header.frame_id = self.odom_frame
        walls.header.stamp = stamp
        walls.ns, walls.id = 'walls', 0
        walls.type, walls.action = Marker.TRIANGLE_LIST, Marker.ADD
        walls.pose.orientation.w = 1.0
        walls.scale.x = walls.scale.y = walls.scale.z = 1.0
        walls.color.r, walls.color.g, walls.color.b, walls.color.a = 0.55, 0.45, 0.35, 0.9
        edges = Marker()
        edges.header = walls.header
        edges.ns, edges.id = 'wall_edges', 1
        edges.type, edges.action = Marker.LINE_LIST, Marker.ADD
        edges.pose.orientation.w = 1.0
        edges.scale.x = 0.01
        edges.color.r, edges.color.g, edges.color.b, edges.color.a = 0.25, 0.2, 0.15, 1.0
        for (x0, y0), (x1, y1) in self.course.walls:
            a0, a1 = Point(x=x0, y=y0, z=0.0), Point(x=x1, y=y1, z=0.0)
            b0, b1 = Point(x=x0, y=y0, z=h), Point(x=x1, y=y1, z=h)
            # 裏からも見えるように両面分の三角形を入れる
            walls.points += [a0, a1, b1, a0, b1, b0, a0, b1, a1, a0, b0, b1]
            edges.points += [b0, b1]

        sx, sy, syaw = self.course.start
        start = Marker()
        start.header = walls.header
        start.ns, start.id = 'start', 2
        start.type, start.action = Marker.ARROW, Marker.ADD
        start.pose.position.x, start.pose.position.y = sx, sy
        (start.pose.orientation.x, start.pose.orientation.y,
         start.pose.orientation.z, start.pose.orientation.w) = yaw_to_quat(syaw)
        start.scale.x, start.scale.y, start.scale.z = 0.12, 0.02, 0.02
        start.color.g, start.color.a = 0.8, 0.8

        self.pub_course.publish(MarkerArray(markers=[walls, edges, start]))

    def publish_path(self):
        msg = Path()
        msg.header.frame_id = self.odom_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.path:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y = x, y
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub_path.publish(msg)

    def log_stats(self):
        if self.last_cmd is None:
            self.get_logger().info(
                '/rover_twist の指令を待っています。別のターミナルで collect.launch.py '
                '(マウス操作) か run_policy.launch.py を起動してください')
            return
        t, v, w = self.last_cmd
        age = self.now() - t
        if age > 1.0:
            cmd = f'指令が {age:.0f}秒間 届いていません'
        else:
            cmd = f'指令 前進{v:+.2f}m/s 旋回{w:+.2f}rad/s'
        self.get_logger().info(
            f'経過 {self.now() - self.t_start:.0f}秒  周回 {abs(self.winding) / (2 * math.pi):.2f}  '
            f'衝突 {self.collisions}回  走行距離 {self.distance:.1f}m  ({cmd})')


def main():
    rclpy.init()
    node = SimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
