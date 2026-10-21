"""5. Data Fusion Node: 輪子圈數(平移) + MPU yaw(朝向) -> Pose2D。
馬達節點在原地旋轉時不送資料，所以位置 x,y 不動、朝向 theta 仍由 yaw 更新；
姿態以固定頻率發佈，確保原地旋轉時控制節點也能拿到最新的 theta。

開機當下的位置/朝向 = 原點 (0,0)、面向 +X（車子要在本壘、面向一壘開機）。
中途要重新對位：  ros2 topic pub --once /reset_pose std_msgs/msg/Empty "{}"
"""
import math

import rclpy
from geometry_msgs.msg import Pose2D, Twist
from rclpy.node import Node
from std_msgs.msg import Empty, Float32


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')
        self.declare_parameter('wheel_radius', 0.10)    # 輪徑 0.20 m -> 半徑 0.10 m
        self.declare_parameter('publish_hz', 20.0)
        self.x = self.y = 0.0
        self.raw_yaw = None
        self.yaw = None
        self.yaw0 = None
        self.pub = self.create_publisher(Pose2D, '/robot_pose', 10)
        self.create_subscription(Float32, '/mpu_yaw', self.on_yaw, 10)
        self.create_subscription(Twist, '/wheel_feedback', self.on_wheel, 10)
        self.create_subscription(Empty, '/reset_pose', self.on_reset, 10)
        self.create_timer(1.0 / self.get_parameter('publish_hz').value, self.publish_pose)

    def on_yaw(self, msg):
        self.raw_yaw = msg.data
        if self.yaw0 is None:
            self.yaw0 = msg.data          # 開機當下朝向 = 0（面向 +X）
        self.yaw = msg.data - self.yaw0

    def on_reset(self, _):
        self.x = self.y = 0.0
        if self.raw_yaw is not None:
            self.yaw0 = self.raw_yaw
            self.yaw = 0.0
        self.get_logger().info('pose reset to (0, 0, 0)')

    def on_wheel(self, msg):
        if self.yaw is None:
            return
        c = 2.0 * math.pi * self.get_parameter('wheel_radius').value   # 圈數 -> 距離
        ds = (msg.linear.x + msg.linear.y) * c / 2.0
        self.x += ds * math.cos(self.yaw)
        self.y += ds * math.sin(self.yaw)

    def publish_pose(self):
        if self.yaw is not None:
            self.pub.publish(Pose2D(x=self.x, y=self.y, theta=wrap(self.yaw)))


def main():
    rclpy.init()
    rclpy.spin(FusionNode())
    rclpy.shutdown()
