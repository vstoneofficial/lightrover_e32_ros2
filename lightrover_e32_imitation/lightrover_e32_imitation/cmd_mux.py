"""人の操作と AI の指令を切り替えるノード (人が優先)。

  /il/human_cmd  (人の操作: マウス・キーボード等) ─┐
                                                    ├─> /il/cmd -> twist_limiter -> /rover_twist
  /il/policy_cmd (学習した方策)  ───────────────────┘

人が操作している間 (ゼロでない指令が来ている間) は人の指令を通し、
手を離すと AI に戻ります。AI が走っている最中に人が「割り込んで修正」できるので、
修正分だけを追加データとして記録する DAgger 的な改善ループに使えます。
"""

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from std_msgs.msg import Bool


def subscribe_human_cmd(node, callback, topic='/il/human_cmd'):
    """人の操作トピックを購読する。

    Jazzy の mouse_teleop は TwistStamped を出すが、teleop_twist_keyboard は既定で Twist。
    型が違うと購読できず何も届かないので、パラメータ human_stamped で切り替え、
    どちらでも callback には Twist を渡す。
    """
    stamped = node.declare_parameter('human_stamped', False).value
    if stamped:
        node.get_logger().info(f'{topic} を TwistStamped として購読')
        return node.create_subscription(
            TwistStamped, topic, lambda m: callback(m.twist), 10)
    node.get_logger().info(f'{topic} を Twist として購読')
    return node.create_subscription(Twist, topic, callback, 10)


def is_nonzero(msg, lin_eps=0.005, ang_eps=0.02):
    return abs(msg.linear.x) > lin_eps or abs(msg.angular.z) > ang_eps


class CmdMux(Node):
    def __init__(self):
        super().__init__('il_cmd_mux')
        self.hold_time = self.declare_parameter('human_hold_time', 0.5).value
        self.policy_timeout = self.declare_parameter('policy_timeout', 0.5).value
        rate = self.declare_parameter('publish_rate', 20.0).value

        self.human = Twist()
        self.human_t = None          # 最後に「ゼロでない」人の指令を受けた時刻
        self.policy = Twist()
        self.policy_t = None
        self.last_source = None

        subscribe_human_cmd(self, self.on_human)
        self.create_subscription(Twist, '/il/policy_cmd', self.on_policy, 10)
        self.pub = self.create_publisher(Twist, '/il/cmd', 10)
        self.pub_active = self.create_publisher(Bool, '/il/human_active', 10)
        self.create_timer(1.0 / rate, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_human(self, msg):
        # 最新の指令は常に保持 (人が手を離したら、ゼロ指令で即座に止まる)
        self.human = msg
        if is_nonzero(msg):
            self.human_t = self.now()

    def on_policy(self, msg):
        self.policy = msg
        self.policy_t = self.now()

    def tick(self):
        t = self.now()
        human_active = self.human_t is not None and t - self.human_t < self.hold_time
        policy_alive = self.policy_t is not None and t - self.policy_t < self.policy_timeout

        if human_active:
            out, source = self.human, 'human'
        elif policy_alive:
            out, source = self.policy, 'policy'
        else:
            out, source = Twist(), 'stop'

        if source != self.last_source:
            self.get_logger().info(f'source -> {source}')
            self.last_source = source
        self.pub.publish(out)
        self.pub_active.publish(Bool(data=bool(human_active)))


def main():
    rclpy.init()
    node = CmdMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
