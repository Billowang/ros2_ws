"""6. MPU Node: MPU-6050 over I2C, yaw by integrating gyro Z (rad, CCW positive).
開不了 I2C（或 simulate:=true）時進入「模擬模式」：用 /motor_rpm_cmd 的左右輪差速積分出 yaw。
Gyro integration drifts; if your module outputs yaw directly, replace read_gz().
"""
import math
import time

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from std_msgs.msg import Float32

try:
    from smbus2 import SMBus
except ImportError:
    SMBus = None

ADDR, PWR, GYRO_Z = 0x68, 0x6B, 0x47
SCALE = 131.0   # LSB per deg/s at +-250 dps


class MPUNode(Node):
    def __init__(self):
        super().__init__('mpu_node')
        self.declare_parameter('bus', 1)
        self.declare_parameter('rate', 100.0)
        self.declare_parameter('simulate', False)
        self.declare_parameter('wheel_radius', 0.10)   # 模擬用，需與 control_node 一致
        self.declare_parameter('track_width', 0.50)
        self.declare_parameter('right_sign', -1.0)

        self.bus = None
        if not self.get_parameter('simulate').value:
            try:
                self.bus = SMBus(self.get_parameter('bus').value)
                self.bus.write_byte_data(ADDR, PWR, 0)
                self.get_logger().info('calibrating gyro bias, keep robot still...')
                self.bias = sum(self.read_gz() for _ in range(200)) / 200.0
            except Exception as e:
                self.bus = None
                self.get_logger().warn(f'MPU open failed ({e}) -> SIMULATION mode')
        self.sim = self.bus is None
        self.yaw = 0.0
        self.t = time.monotonic()
        self.sim_w = 0.0
        self.pub = self.create_publisher(Float32, '/mpu_yaw', 10)
        if self.sim:
            self.get_logger().info('mpu_node: simulation mode (yaw from wheel command)')
            self.create_subscription(Vector3, '/motor_rpm_cmd', self.on_cmd, 10)
        self.create_timer(1.0 / self.get_parameter('rate').value, self.tick)

    def on_cmd(self, msg):
        c = 2.0 * math.pi * self.get_parameter('wheel_radius').value / 60.0   # rpm -> m/s
        vl = msg.x * c
        vr = msg.y * c * self.get_parameter('right_sign').value
        self.sim_w = (vr - vl) / self.get_parameter('track_width').value

    def read_gz(self):
        hi, lo = self.bus.read_i2c_block_data(ADDR, GYRO_Z, 2)
        v = (hi << 8) | lo
        v = v - 65536 if v & 0x8000 else v
        time.sleep(0.002)
        return v / SCALE            # deg/s

    def tick(self):
        now = time.monotonic()
        dt, self.t = now - self.t, now
        if self.sim:
            self.yaw += self.sim_w * dt
        else:
            try:
                gz = self.read_gz() - self.bias
            except Exception as e:
                self.get_logger().warn(f'MPU read failed: {e}', throttle_duration_sec=5.0)
                return
            if abs(gz) > 0.3:       # deadband against noise
                self.yaw += math.radians(gz) * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        self.pub.publish(Float32(data=self.yaw))


def main():
    rclpy.init()
    rclpy.spin(MPUNode())
    rclpy.shutdown()
