"""4. Motor Node: Modbus-RTU style serial driver + wheel feedback.
開不了序列埠（或 simulate:=true）時自動進入「模擬模式」：
把收到的轉速指令積分成輪子圈數回傳，方便沒接硬體時測整條鏈路。

Feedback frame (8 bytes): station(01/02) 66h index(int16) step(uint16, 0-9999) CRC16(lo,hi)
10000 steps = 1 revolution.  Station 01 = left wheel, 02 = right wheel.

/wheel_feedback (Twist):
  linear.x / linear.y   = left / right revolution delta since last publish
  angular.x / angular.y = left / right running sum of deltas
"""
import struct

import rclpy
from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Node

try:
    import serial
except ImportError:
    serial = None

STEPS = 10000
MOD = 65536 * STEPS


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def with_crc(frame: bytes) -> bytes:
    return frame + struct.pack('<H', crc16(frame))


class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')
        self.declare_parameter('port', '/dev/mymotor')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('speed_reg', 0x0000)   # TODO: your driver's target-speed register
        self.declare_parameter('left_station', 1)
        self.declare_parameter('right_station', 2)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('simulate', False)

        self.ser = None
        if not self.get_parameter('simulate').value:
            try:
                self.ser = serial.Serial(self.get_parameter('port').value,
                                         self.get_parameter('baud').value, timeout=0)
            except Exception as e:
                self.get_logger().warn(f'serial open failed ({e}) -> SIMULATION mode')
        self.sim = self.ser is None
        if self.sim:
            self.get_logger().info('motor_node: simulation mode (no hardware)')

        self.buf = bytearray()
        self.last_raw = {}
        self.delta = {1: 0.0, 2: 0.0}
        self.total = {1: 0.0, 2: 0.0}
        self.sim_rpm = {1: 0.0, 2: 0.0}
        self.last_cmd = self.get_clock().now()

        self.pub_fb = self.create_publisher(Twist, '/wheel_feedback', 10)
        self.create_subscription(Vector3, '/motor_rpm_cmd', self.on_cmd, 10)
        if self.sim:
            self.create_timer(0.05, self.sim_step)
        else:
            self.create_timer(0.005, self.read_serial)
        self.create_timer(0.05, self.publish_fb)
        self.create_timer(0.1, self.watchdog)

    # ---- command out ----
    def write_speed(self, station, rpm):
        reg = self.get_parameter('speed_reg').value
        frame = struct.pack('>BBHh', station, 0x06, reg, int(rpm))   # Modbus 06, adjust to manual
        try:
            self.ser.write(with_crc(frame))
        except Exception as e:
            self.get_logger().warn(f'serial write failed: {e}', throttle_duration_sec=5.0)

    def on_cmd(self, msg):
        self.last_cmd = self.get_clock().now()
        if self.sim:
            self.sim_rpm = {1: msg.x, 2: msg.y}
            return
        self.write_speed(self.get_parameter('left_station').value, msg.x)
        self.write_speed(self.get_parameter('right_station').value, msg.y)

    def watchdog(self):    # stop if commands go silent
        if (self.get_clock().now() - self.last_cmd).nanoseconds / 1e9 > self.get_parameter('cmd_timeout').value:
            self.last_cmd = self.get_clock().now()
            if self.sim:
                self.sim_rpm = {1: 0.0, 2: 0.0}
            else:
                self.write_speed(self.get_parameter('left_station').value, 0)
                self.write_speed(self.get_parameter('right_station').value, 0)

    # ---- feedback in ----
    def sim_step(self):
        for k in (1, 2):
            d = self.sim_rpm[k] / 60.0 * 0.05          # rev = rpm/60 * dt
            self.delta[k] += d
            self.total[k] += d

    def read_serial(self):
        try:
            n = self.ser.in_waiting
            if n:
                self.buf += self.ser.read(n)
        except Exception as e:
            self.get_logger().warn(f'serial read failed: {e}', throttle_duration_sec=5.0)
            return
        stations = (self.get_parameter('left_station').value, self.get_parameter('right_station').value)
        while len(self.buf) >= 8:
            f = bytes(self.buf[:8])
            if f[0] in stations and f[1] == 0x66 and struct.unpack('<H', f[6:8])[0] == crc16(f[:6]):
                index, step = struct.unpack('>hH', f[2:6])
                self.on_position(f[0], index * STEPS + step)
                del self.buf[:8]
            else:
                del self.buf[0]       # resync

    def on_position(self, st, raw):
        prev = self.last_raw.get(st)
        self.last_raw[st] = raw
        if prev is None:
            return
        d = ((raw - prev + MOD // 2) % MOD - MOD // 2) / STEPS     # handle int16 index wrap
        key = 1 if st == self.get_parameter('left_station').value else 2
        self.delta[key] += d
        self.total[key] += d

    def publish_fb(self):
        m = Twist()
        m.linear.x, m.linear.y = self.delta[1], self.delta[2]
        m.angular.x, m.angular.y = self.total[1], self.total[2]
        self.delta = {1: 0.0, 2: 0.0}
        self.pub_fb.publish(m)


def main():
    rclpy.init()
    rclpy.spin(MotorNode())
    rclpy.shutdown()
