"""7. Powder Spreading Node: output 1 while control is spraying, else 0.
安全：切到手動立刻關；超過 cmd_timeout 沒收到 /spray_cmd（控制節點當掉）也自動關。
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class SprayNode(Node):
    def __init__(self):
        super().__init__('spray_node')
        self.declare_parameter('cmd_timeout', 1.0)
        self.out = False
        self.last_cmd = self.get_clock().now()
        self.pub = self.create_publisher(Bool, '/spray_out', 10)
        self.create_subscription(Bool, '/spray_cmd', self.on_cmd, 10)
        self.create_subscription(String, '/mode_cmd', self.on_mode, 10)
        self.create_timer(0.1, self.tick)

    def set_out(self, value):
        if value != self.out:
            self.out = value
            self.get_logger().info(f'spray = {int(self.out)}')
            # TODO: drive your GPIO / relay here

    def on_cmd(self, msg):
        self.last_cmd = self.get_clock().now()
        self.set_out(bool(msg.data))

    def on_mode(self, msg):
        if msg.data == 'manual':
            self.set_out(False)

    def tick(self):
        if self.out and (self.get_clock().now() - self.last_cmd).nanoseconds / 1e9 > self.get_parameter('cmd_timeout').value:
            self.get_logger().warn('spray_cmd lost -> spray off')
            self.set_out(False)
        self.pub.publish(Bool(data=self.out))


def main():
    rclpy.init()
    rclpy.spin(SprayNode())
    rclpy.shutdown()
