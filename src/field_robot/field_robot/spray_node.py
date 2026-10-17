"""7. Powder Spreading Node: output 1 while control is auto-following a map, else 0."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class SprayNode(Node):
    def __init__(self):
        super().__init__('spray_node')
        self.out = False
        self.pub = self.create_publisher(Bool, '/spray_out', 10)
        self.create_subscription(Bool, '/spray_cmd', self.on_cmd, 10)
        self.create_timer(0.1, lambda: self.pub.publish(Bool(data=self.out)))

    def on_cmd(self, msg):
        if msg.data != self.out:
            self.out = msg.data
            self.get_logger().info(f'spray = {int(self.out)}')
            # TODO: drive your GPIO / relay here


def main():
    rclpy.init()
    rclpy.spin(SprayNode())
    rclpy.shutdown()
