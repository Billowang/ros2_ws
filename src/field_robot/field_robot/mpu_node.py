"""6. MPU Node -> /mpu_yaw (rad, 逆時針為正)。三種資料來源（參數 source）：
  serial (預設) : 讀 ESP32 經 USB/UART 傳來的 yaw 封包（ESP32 已做校正 + 噪音忽略 + 積分）
  i2c           : 樹莓派直接接 MPU-6500/6050，本節點自己校正、積分（同樣有噪音忽略）
  開不了來源（或 simulate:=true）時自動進入模擬模式：用 /motor_rpm_cmd 的左右輪差速積分 yaw。

ESP32 封包 (11 bytes，小端序)：
  AA 55 | seq u8 | yaw_mdeg i32 (0.001°) | rate_cdps i16 (0.01°/s) | CRC16 Modbus(小端，範圍 seq..rate)
"""
import math
import statistics
import struct
import time

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from std_msgs.msg import Float32

try:
    import serial
except ImportError:
    serial = None
try:
    from smbus2 import SMBus
except ImportError:
    SMBus = None

PWR_MGMT_1, CONFIG, GYRO_CONFIG, GYRO_ZOUT_H, WHO_AM_I = 0x6B, 0x1A, 0x1B, 0x47, 0x75
LSB_PER_DPS = 131.0          # ±250 dps
FRAME_LEN = 11


def crc16(data) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


class MPUNode(Node):
    def __init__(self):
        super().__init__('mpu_node')
        for k, v in dict(source='serial', simulate=False,
                         port='/dev/mympu', baud=115200, stale_sec=0.5,          # serial
                         bus=1, addr=0x68, rate=100.0,                             # i2c
                         deadband_dps=0.5, calib_samples=500, calib_max_std=30.0,
                         auto_bias=True, bias_alpha=0.002,
                         wheel_radius=0.10, track_width=0.50).items():            # 模擬用
            self.declare_parameter(k, v)
        self.gp = lambda k: self.get_parameter(k).value

        self.ser = None
        self.bus = None
        self.bias = 0.0
        self.fail = 0
        if not self.gp('simulate'):
            if self.gp('source') == 'serial':
                try:
                    self.ser = serial.Serial(self.gp('port'), self.gp('baud'), timeout=0)
                    self.get_logger().info(f'reading ESP32 yaw from {self.gp("port")}')
                except Exception as e:
                    self.get_logger().warn(f'serial open failed ({e}) -> SIMULATION mode')
            else:
                try:
                    self.bus = SMBus(self.gp('bus'))
                    self.init_hw()
                    self.calibrate()
                except Exception as e:
                    self.bus = None
                    self.get_logger().warn(f'MPU open failed ({e}) -> SIMULATION mode')
        self.sim = self.ser is None and self.bus is None
        self.sim_w = 0.0
        self.yaw = 0.0
        self.rx_buf = bytearray()
        self.last_rx = time.monotonic()
        self.t = time.monotonic()
        self.pub = self.create_publisher(Float32, '/mpu_yaw', 10)

        if self.sim:
            self.get_logger().info('mpu_node: simulation mode (yaw from wheel command)')
            self.create_subscription(Vector3, '/motor_rpm_cmd', self.on_cmd, 10)
        if self.ser is not None:
            self.create_timer(0.005, self.read_serial)
            self.create_timer(0.5, self.check_stale)
        else:
            self.create_timer(1.0 / self.gp('rate'), self.tick)

    # ================= serial (ESP32) =================
    def read_serial(self):
        try:
            n = self.ser.in_waiting
            if n:
                self.rx_buf += self.ser.read(n)
        except Exception as e:
            self.get_logger().warn(f'serial read failed: {e}', throttle_duration_sec=5.0)
            return
        buf = self.rx_buf
        while len(buf) >= FRAME_LEN:
            if buf[0] == 0xAA and buf[1] == 0x55:
                f = bytes(buf[:FRAME_LEN])
                if crc16(f[2:9]) == int.from_bytes(f[9:11], 'little'):
                    mdeg = struct.unpack('<i', f[3:7])[0]
                    self.yaw = math.radians(mdeg / 1000.0)
                    self.last_rx = time.monotonic()
                    self.pub.publish(Float32(data=self.yaw))
                    del buf[:FRAME_LEN]
                    continue
            del buf[0]                    # 重新同步（開機亂碼、掉位元組都會走到這裡）

    def check_stale(self):
        if time.monotonic() - self.last_rx > self.gp('stale_sec'):
            self.get_logger().warn('no yaw data from ESP32', throttle_duration_sec=5.0)

    # ================= i2c (直接接樹莓派) =================
    def init_hw(self):
        a = self.gp('addr')
        who = self.bus.read_byte_data(a, WHO_AM_I)
        self.get_logger().info(f'WHO_AM_I = 0x{who:02X} (MPU-6500=0x70, MPU-6050=0x68)')
        self.bus.write_byte_data(a, PWR_MGMT_1, 0x80)
        time.sleep(0.1)
        self.bus.write_byte_data(a, PWR_MGMT_1, 0x01)
        time.sleep(0.05)
        self.bus.write_byte_data(a, CONFIG, 0x03)
        self.bus.write_byte_data(a, GYRO_CONFIG, 0x00)
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
        else:
            self.get_logger().error('gyro calibration unreliable: robot kept moving, yaw will drift')
        self.get_logger().info(f'gyro Z bias = {self.bias:.2f} LSB (std {std:.1f})')

    # ================= 模擬 =================
    def on_cmd(self, msg):
        c = 2.0 * math.pi * self.gp('wheel_radius') / 60.0
        self.sim_w = (msg.y * c - msg.x * c) / self.gp('track_width')

    # ================= i2c / 模擬 主迴圈 =================
    def tick(self):
        now = time.monotonic()
        dt, self.t = now - self.t, now
        nominal = 1.0 / self.gp('rate')
        if dt > 5 * nominal:
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
                if self.fail >= 20:
                    try:
                        self.init_hw()
                        self.fail = 0
                    except Exception:
                        pass
                return
            dps = (raw - self.bias) / LSB_PER_DPS
            if abs(dps) < self.gp('deadband_dps'):
                dps = 0.0
                if self.gp('auto_bias'):
                    self.bias += self.gp('bias_alpha') * (raw - self.bias)
            self.yaw += math.radians(dps) * dt

        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        self.pub.publish(Float32(data=self.yaw))


def main():
    rclpy.init()
    rclpy.spin(MPUNode())
    rclpy.shutdown()
