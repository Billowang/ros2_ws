"""MPU Node -> /mpu_yaw (rad, 逆時針為正)、/mpu_rate (rad/s)。
讀取邏輯直接使用 COM6 測試腳本（找 0xAA -> 0x55 -> 讀 9 bytes -> '<ff'），
只把 print 換成發佈 topic。
ESP32 封包 (11 bytes，小端序，Float32)：AA 55 | seq u8 | yaw_deg float32 | rate_dps float32
"""
import math
import struct
import threading

import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Float32


class MPUNode(Node):
    def __init__(self):
        super().__init__('mpu_node')
        self.declare_parameter('port', '/dev/mympu')   # 請依實際連接埠修改
        self.declare_parameter('baud', 115200)
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        self.ser = serial.Serial(port, baud, timeout=1.0)
        self.get_logger().info(f'Listening on {port} at {baud} baud (Float32 mode)...')

        self.pub_yaw = self.create_publisher(Float32, '/mpu_yaw', 10)
        self.pub_rate = self.create_publisher(Float32, '/mpu_rate', 10)

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def _reader(self):
        ser = self.ser
        while not self._stop.is_set():
            try:
                # 尋找同步標頭的第一個字節
                header_1 = ser.read(1)
                if header_1 == b"\xaa":
                    header_2 = ser.read(1)
                    if header_2 == b"\x55":
                        # 讀取後續 9 bytes: seq(1) + yaw(4) + rate(4)
                        rest = ser.read(9)
                        if len(rest) == 9:
                            seq = rest[0]
                            # '<ff' 代表小端序連續兩個 float32
                            yaw_deg, rate_dps = struct.unpack("<ff", rest[1:])
                            self.get_logger().info(
                                f"Seq: {seq:3d} | Yaw: {yaw_deg:8.3f} deg | Rate: {rate_dps:6.2f} dps",
                                throttle_duration_sec=1.0)
                            self.pub_yaw.publish(Float32(data=math.radians(yaw_deg)))
                            self.pub_rate.publish(Float32(data=math.radians(rate_dps)))
                        else:
                            self.get_logger().warn("警告: 讀取資料逾時或長度不足 (期待 9 bytes)")
            except Exception as e:
                if self._stop.is_set():
                    break
                self.get_logger().warn(f"讀取發生錯誤: {e}", throttle_duration_sec=5.0)


def main():
    rclpy.init()
    node = MPUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop.set()
        node._thread.join(timeout=1.5)
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()
