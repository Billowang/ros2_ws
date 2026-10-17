"""5. Data Fusion Node: wheel revolutions + MPU yaw -> Pose2D."""
import math

import rclpy
from geometry_msgs.msg import Pose2D, Twist
from rclpy.node import Node
from std_msgs.msg import Float32


class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')
        self.declare_parameter('wheel_radius', 0.10)
        self.declare_parameter('right_sign', -1.0)   # same mirroring as control_node
        self.x = self.y = 0.0
        self.yaw = None
        self.yaw0 = None
        self.prev_theta = 0.0
        self.pub = self.create_publisher(Pose2D, '/robot_pose', 10)
        self.create_subscription(Float32, '/mpu_yaw', self.on_yaw, 10)
        self.create_subscription(Twist, '/wheel_feedback', self.on_wheel, 10)

    def on_yaw(self, msg):
        if self.yaw0 is None:
            self.yaw0 = msg.data          # heading at power-up = 0 (home plate, facing +X)
        self.yaw = msg.data - self.yaw0

    def on_wheel(self, msg):
        if self.yaw is None:
            return
        c = 2.0 * math.pi * self.get_parameter('wheel_radius').value
        dl = msg.linear.x * c
        dr = msg.linear.y * c * self.get_parameter('right_sign').value
        ds = (dl + dr) / 2.0
        mid = (self.prev_theta + self.yaw) / 2.0
        self.x += ds * math.cos(mid)
        self.y += ds * math.sin(mid)
        self.prev_theta = self.yaw
        self.pub.publish(Pose2D(x=self.x, y=self.y, theta=self.yaw))


def main():
    rclpy.init()
    rclpy.spin(FusionNode())
    rclpy.shutdown()
