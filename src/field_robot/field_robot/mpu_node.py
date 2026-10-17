"""6. MPU Node: MPU-6500/6050 (I2C) 陀螺儀 Z 軸積分 -> /mpu_yaw (rad, 逆時針為正)。

沿用你的做法：開機靜止校正偏移量(500 筆平均) + 噪音忽略(|角速度| < 1.5 dps 視為 0)，並加上：
  * 感測器設定：PLL 時脈、DLPF 44 Hz、±250 dps（131 LSB/dps），並讀 WHO_AM_I 確認晶片
  * 只讀 Z 軸 2 個位元組（不讀整包 14 位元組），I2C 時間更短
  * 校正時檢查是否靜止（標準差過大就重做，最多 3 次）
  * 靜止區(噪音忽略範圍內)緩慢追蹤零偏，抵銷溫漂
  * dt 保護：任務卡頓時不會一次積分暴衝
  * I2C 連續失敗自動重新初始化
開不了 I2C（或 simulate:=true）時進入模擬模式：用 /motor_rpm_cmd 的左右輪差速積分 yaw。
"""
import math
import statistics
import time

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from std_msgs.msg import Float32

try:
    from smbus2 import SMBus
except ImportError:
    SMBus = None

PWR_MGMT_1, CONFIG, GYRO_CONFIG, GYRO_ZOUT_H, WHO_AM_I = 0x6B, 0x1A, 0x1B, 0x47, 0x75
LSB_PER_DPS = 131.0          # ±250 dps


class MPUNode(Node):
    def __init__(self):
        super().__init__('mpu_node')
        for k, v in dict(bus=1, addr=0x68, rate=100.0, simulate=False,
                         deadband_dps=1.5,          # 你的噪音忽略門檻
                         calib_samples=500, calib_max_std=30.0,   # std 單位 LSB (≈0.23 dps)
                         auto_bias=True, bias_alpha=0.002,
                         wheel_radius=0.10, track_width=0.50).items():   # 後兩項只給模擬用
            self.declare_parameter(k, v)
        self.gp = lambda k: self.get_parameter(k).value

        self.bus = None
        self.bias = 0.0
        self.fail = 0
        if not self.gp('simulate'):
            try:
                self.bus = SMBus(self.gp('bus'))
                self.init_hw()
                self.calibrate()
            except Exception as e:
                self.bus = None
                self.get_logger().warn(f'MPU open failed ({e}) -> SIMULATION mode')
        self.sim = self.bus is None
        self.sim_w = 0.0
        self.yaw = 0.0
        self.t = time.monotonic()
        self.pub = self.create_publisher(Float32, '/mpu_yaw', 10)
        if self.sim:
            self.get_logger().info('mpu_node: simulation mode (yaw from wheel command)')
            self.create_subscription(Vector3, '/motor_rpm_cmd', self.on_cmd, 10)
        self.create_timer(1.0 / self.gp('rate'), self.tick)

    # ---------------- 硬體 ----------------
    def init_hw(self):
        a = self.gp('addr')
        who = self.bus.read_byte_data(a, WHO_AM_I)
        self.get_logger().info(f'WHO_AM_I = 0x{who:02X} (MPU-6500=0x70, MPU-6050=0x68)')
        self.bus.write_byte_data(a, PWR_MGMT_1, 0x80)     # reset
        time.sleep(0.1)
        self.bus.write_byte_data(a, PWR_MGMT_1, 0x01)     # 喚醒，PLL 時脈（比內部 RC 穩）
        time.sleep(0.05)
        self.bus.write_byte_data(a, CONFIG, 0x03)         # DLPF 44 Hz
        self.bus.write_byte_data(a, GYRO_CONFIG, 0x00)    # ±250 dps
        time.sleep(0.05)

    def read_raw(self):
        hi, lo = self.bus.read_i2c_block_data(self.gp('addr'), GYRO_ZOUT_H, 2)
        v = (hi << 8) | lo
        return v - 65536 if v & 0x8000 else v

    def calibrate(self):
        n = self.gp('calib_samples')
        for attempt in range(1, 4):
            self.get_logger().info(f'calibrating gyro ({attempt}/3), keep the robot STILL...')
            xs = []
            for _ in range(n):
                xs.append(self.read_raw())
                time.sleep(0.003)
            std = statistics.pstdev(xs)
            self.bias = sum(xs) / n
            if std <= self.gp('calib_max_std'):
                break
            self.get_logger().warn(f'robot not still (std={std:.1f} LSB), retrying')
        self.get_logger().info(f'gyro Z bias = {self.bias:.2f} LSB (std {std:.1f})')

    # ---------------- 模擬 ----------------
    def on_cmd(self, msg):
        c = 2.0 * math.pi * self.gp('wheel_radius') / 60.0     # rpm -> m/s
        self.sim_w = (msg.y * c - msg.x * c) / self.gp('track_width')

    # ---------------- 主迴圈 ----------------
    def tick(self):
        now = time.monotonic()
        dt, self.t = now - self.t, now
        nominal = 1.0 / self.gp('rate')
        if dt > 5 * nominal:                 # 卡頓保護
            dt = nominal

        if self.sim:
            self.yaw += self.sim_w * dt
        else:
            try:
                raw = self.read_raw()
                self.fail = 0
            except Exception as e:
                self.fail += 1
                self.get_logger().warn(f'MPU read failed: {e}', throttle_duration_sec=5.0)
                if self.fail >= 20:          # 連續失敗 -> 重新初始化（不重新校正，保留偏移量）
                    try:
                        self.init_hw()
                        self.fail = 0
                    except Exception:
                        pass
                return
            dps = (raw - self.bias) / LSB_PER_DPS
            if abs(dps) < self.gp('deadband_dps'):          # 噪音忽略
                dps = 0.0
                if self.gp('auto_bias'):                    # 靜止區內緩慢追蹤零偏
                    self.bias += self.gp('bias_alpha') * (raw - self.bias)
            self.yaw += math.radians(dps) * dt

        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        self.pub.publish(Float32(data=self.yaw))


def main():
    rclpy.init()
    rclpy.spin(MPUNode())
    rclpy.shutdown()