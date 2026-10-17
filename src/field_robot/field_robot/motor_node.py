"""4. Motor Node: 串列埠馬達驅動 + 輪子圈數回授（沿用你實機的指令/回授格式）。

輸入  /motor_rpm_cmd (Vector3: x=左輪 rpm, y=右輪 rpm；上游一律「前進為正」，輪子裝反用 left_sign/right_sign 修)
輸出  /wheel_feedback (Twist):
        linear.x / linear.y   = 左 / 右輪「這一段時間」的圈數差
        angular.x / angular.y = 左 / 右輪累積總圈數
        angular.z             = 這一段時間長度 (s)
      左右輪圈數差「一正一負」時（原地旋轉）整筆不送出、也不累加，視為沒有移動。

開不了序列埠（或 simulate:=true）時進入模擬模式：把轉速指令積分成圈數，走同一套回授流程。
"""
import struct

import rclpy
from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Node

try:
    import serial
except ImportError:
    serial = None

STEP_PER_REV = 10000
FRAME_LEN = 8            # 站號(1) + 0x66(1) + index(2) + step(2) + CRC(2)


def crc16(data) -> bytes:
    """Modbus RTU CRC16（小端序）"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc.to_bytes(2, 'little')


def i16(v) -> bytes:
    return struct.pack('>h', max(-32768, min(32767, int(v))))


class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')
        for k, v in dict(port='/dev/mymotor', baud=115200, simulate=False,
                         left_station=1, right_station=2,
                         left_sign=1.0, right_sign=1.0,      # 輪子裝反時其中一個改 -1.0（指令與回授同時生效）
                         cmd_timeout=0.5).items():
            self.declare_parameter(k, v)
        gp = lambda k: self.get_parameter(k).value
        self.L, self.R = gp('left_station'), gp('right_station')
        self.sign = {self.L: gp('left_sign'), self.R: gp('right_sign')}

        self.ser = None
        if not gp('simulate'):
            try:
                self.ser = serial.Serial(port=gp('port'), baudrate=gp('baud'),
                                         bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                                         stopbits=serial.STOPBITS_ONE, timeout=0)
                self.get_logger().info(f'serial opened {gp("port")} ({gp("baud")}, 8N1)')
            except Exception as e:
                self.get_logger().warn(f'serial open failed ({e}) -> SIMULATION mode')
        self.sim = self.ser is None

        self.rx_buf = bytearray()
        self.positions = {}                  # 站號 -> 目前圈數(含小數)，原始方向
        self.last_pub_pos = {}
        self.total_revs = {self.L: 0.0, self.R: 0.0}
        self.last_pub_time = None
        self.sim_rpm = {self.L: 0.0, self.R: 0.0}
        self.last_cmd = self.get_clock().now()

        self.fb_pub = self.create_publisher(Twist, '/wheel_feedback', 10)
        self.create_subscription(Vector3, '/motor_rpm_cmd', self.on_cmd, 10)
        self.create_timer(0.02, self.sim_step if self.sim else self.read_serial)
        self.create_timer(0.05, self.publish_feedback)
        self.create_timer(0.1, self.watchdog)

    # ---------------- 指令 ----------------
    def on_cmd(self, msg):
        self.last_cmd = self.get_clock().now()
        self.send_rpm(msg.x * self.sign[self.L], msg.y * self.sign[self.R])

    def send_rpm(self, left, right):
        if self.sim:
            self.sim_rpm = {self.L: left, self.R: right}
            return
        base = bytearray([0x00, 0x65, 0x02, 0x02, 0x0A, 0x00, 0x00, *i16(right),
                          0x01, 0x0A, 0x00, 0x00, *i16(left)])
        try:
            self.ser.write(base + crc16(base))
        except Exception as e:
            self.get_logger().warn(f'serial write failed: {e}', throttle_duration_sec=5.0)

    def watchdog(self):          # 指令中斷就停車
        if (self.get_clock().now() - self.last_cmd).nanoseconds / 1e9 > self.get_parameter('cmd_timeout').value:
            self.last_cmd = self.get_clock().now()
            self.send_rpm(0, 0)

    # ---------------- 回授 ----------------
    def sim_step(self):
        for st in (self.L, self.R):
            self.positions[st] = self.positions.get(st, 0.0) + self.sim_rpm[st] / 60.0 * 0.02

    def read_serial(self):
        try:
            n = self.ser.in_waiting
            if n:
                self.rx_buf += self.ser.read(n)
        except Exception as e:
            self.get_logger().warn(f'serial read failed: {e}', throttle_duration_sec=5.0)
            return
        while len(self.rx_buf) >= FRAME_LEN:          # 含 CRC 驗證，自動重新同步
            f = bytes(self.rx_buf[:FRAME_LEN])
            if f[0] in (self.L, self.R) and f[1] == 0x66 and crc16(f[:-2]) == f[-2:]:
                index = struct.unpack('>h', f[2:4])[0]
                step = struct.unpack('>H', f[4:6])[0]
                if step <= STEP_PER_REV:
                    self.positions[f[0]] = index + step / STEP_PER_REV
                del self.rx_buf[:FRAME_LEN]
            else:
                del self.rx_buf[0]

    @staticmethod
    def wrap(d):                 # index 為 int16，跨越 ±32768 圈時修正
        return (d + 32768.0) % 65536.0 - 32768.0

    def publish_feedback(self):
        if self.L not in self.positions or self.R not in self.positions:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_pub_time is None:                # 第一次只記錄起點
            self.last_pub_pos = dict(self.positions)
            self.last_pub_time = now
            return
        dt = now - self.last_pub_time
        d_left = self.sign[self.L] * self.wrap(self.positions[self.L] - self.last_pub_pos[self.L])
        d_right = self.sign[self.R] * self.wrap(self.positions[self.R] - self.last_pub_pos[self.R])
        self.last_pub_pos = dict(self.positions)
        self.last_pub_time = now

        if d_left * d_right < 0:      # 一正一負 = 原地旋轉：不送、不累加
            return

        self.total_revs[self.L] += d_left
        self.total_revs[self.R] += d_right
        fb = Twist()
        fb.linear.x, fb.linear.y = d_left, d_right
        fb.angular.x, fb.angular.y = self.total_revs[self.L], self.total_revs[self.R]
        fb.angular.z = dt
        self.fb_pub.publish(fb)


def main():
    rclpy.init()
    node = MotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser is not None and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()