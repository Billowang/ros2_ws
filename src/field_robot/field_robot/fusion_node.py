"""5. Data Fusion Node: 輪子圈數(平移) + MPU yaw(朝向) -> Pose2D。
馬達節點在原地旋轉時不送資料，所以位置 x,y 不動、朝向 theta 仍由 yaw 更新；
姿態以固定頻率發佈，確保原地旋轉時控制節點也能拿到最新的 theta。

朝向基準：每次按下「開始工作」(/mission_cmd 的 start)，就把上一筆收到的 yaw 存成基準，
之後 yaw = 目前 yaw - 基準，也就是開始工作當下的車頭方向 = 0 度（+X）。
開機到第一次開始工作之前，先用開機第一筆 yaw 當暫時基準，讓姿態有值可用。
位置 x,y 不會因為開始工作而歸零。
中途要重新對位（位置和朝向都歸零）：  ros2 topic pub --once /reset_pose std_msgs/msg/Empty "{}"
"""
import json
import math

import rclpy
from geometry_msgs.msg import Pose2D, Twist
from rclpy.node import Node
from std_msgs.msg import Empty, Float32, String


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
        self.pub_zeroed = self.create_publisher(Empty, '/heading_zeroed', 10)
        self.create_subscription(Float32, '/mpu_yaw', self.on_yaw, 10)
        self.create_subscription(Twist, '/wheel_feedback', self.on_wheel, 10)
        self.create_subscription(Empty, '/reset_pose', self.on_reset, 10)
        self.create_subscription(String, '/mission_cmd', self.on_cmd, 10)
        self.create_timer(1.0 / self.get_parameter('publish_hz').value, self.publish_pose)

    def on_yaw(self, msg):
        self.raw_yaw = msg.data
        if self.yaw0 is None:
            self.yaw0 = msg.data          # 開機當下朝向 = 0（面向 +X）
        self.yaw = msg.data - self.yaw0

    def on_cmd(self, msg):
        # 格式同 map_node：JSON {"cmd":"start",...} 或純文字 "start"
        cmd = msg.data.strip()
        if cmd.startswith('{'):
            try:
                cmd = json.loads(cmd).get('cmd')
            except ValueError:
                return
        if cmd == 'start':
            self.zero_heading()

    def zero_heading(self):
        if self.raw_yaw is None:
            self.get_logger().warn('開始工作時還沒收到 yaw，無法歸零朝向')
        else:
            self.yaw0 = self.raw_yaw      # 上一筆 yaw 當基準，之後 = 目前 - 基準
            self.yaw = 0.0
            self.get_logger().info(f'work start: heading zeroed (baseline {math.degrees(self.yaw0):.2f} deg)')
            self.publish_pose()           # 先把歸零後的姿態送出去
        self.pub_zeroed.publish(Empty())  # 再通知 map_node 可以送第一個目標

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
