import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import struct

# ===================== 可調參數 =====================
STEP_PER_REV = 10000     # 每圈 step 數
WHEEL_DIAMETER = 0.20    # 輪徑(公尺)，請改成實際值
FRAME_LEN = 8            # 站號(1) + 0x66(1) + index(2) + step(2) + CRC(2)

LEFT_STATION = 0x01      # 請確認哪個站號是左輪
RIGHT_STATION = 0x02
LEFT_SIGN = 1.0          # 左右輪裝反時，其中一個改成 -1.0
RIGHT_SIGN = 1.0

PUBLISH_PERIOD = 0.05    # 發布週期(秒)
# ====================================================


class MotorSerialNode(Node):
    def __init__(self):
        super().__init__('motor_node')

        # 訂閱安全過濾後的速度指令
        self.subscription = self.create_subscription(
            Twist, '/speed', self.motor_callback, 10)

        # 發布輪子回授
        self.mix_data_pub = self.create_publisher(Twist, '/encoder', 10)

        # 序列埠
        self.ser = None
        try:
            self.ser = serial.Serial(
                port='/dev/mymotor',
                baudrate=115200,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.5
            )
            self.get_logger().info("✅ 序列埠已成功啟動 (115200, 8N1)")
        except Exception as e:
            self.get_logger().error(f"❌ 無法開啟序列埠: {e}")

        # 接收與位置狀態
        self.rx_buf = bytearray()
        self.positions = {}        # {站號: 目前圈數(含小數)}
        self.last_pub_pos = {}     # {站號: 上次發布時的圈數}
        self.total_revs = {LEFT_STATION: 0.0, RIGHT_STATION: 0.0}  # 累積總圈數
        self.last_pub_time = None

        self.create_timer(0.02, self.read_serial)                  # 50Hz 讀取
        self.create_timer(PUBLISH_PERIOD, self.publish_feedback)

    # ---------------- CRC ----------------
    def calculate_crc(self, data):
        """Modbus RTU CRC16（小端序回傳）"""
        crc = 0xFFFF
        for pos in data:
            crc ^= pos
            for _ in range(8):
                if crc & 1:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc.to_bytes(2, byteorder='little')

    # ---------------- 發送馬達指令 ----------------
    def motor_callback(self, msg: Twist):
        if self.ser is None or not self.ser.is_open:
            return
        try:
            v = msg.linear.x
            w = msg.angular.z
            max_rpm = -msg.linear.z

            right_rpm = int((v - (w * 0.5)) * max_rpm)
            left_rpm = int((v + (w * 0.5)) * max_rpm)

            l_hex = struct.pack('>h', left_rpm)
            r_hex = struct.pack('>h', right_rpm)

            base_cmd = bytearray([
                0x00, 0x65, 0x02, 0x02,
                0x0A,
                0x00, 0x00,
                r_hex[0], r_hex[1],
                0x01,
                0x0A,
                0x00, 0x00,
                l_hex[0], l_hex[1]
            ])

            full_cmd = base_cmd + self.calculate_crc(base_cmd)
            self.ser.write(full_cmd)   # 不清輸入緩衝區，避免誤清馬達回應

        except Exception as e:
            self.get_logger().error(f"回調執行錯誤: {e}")

    # ---------------- 讀取馬達回應 ----------------
    def read_serial(self):
        if self.ser is None or not self.ser.is_open:
            return
        try:
            n = self.ser.in_waiting
            if n:
                self.rx_buf += self.ser.read(n)
        except Exception as e:
            self.get_logger().error(f"讀取序列埠錯誤: {e}")
            return

        # 在緩衝區中找合法封包（含 CRC 驗證，自動重新同步）
        while len(self.rx_buf) >= FRAME_LEN:
            frame = bytes(self.rx_buf[:FRAME_LEN])
            header_ok = frame[0] in (LEFT_STATION, RIGHT_STATION) and frame[1] == 0x66
            crc_ok = header_ok and self.calculate_crc(frame[:-2]) == frame[-2:]
            if crc_ok:
                self.parse_position(frame)
                del self.rx_buf[:FRAME_LEN]
            else:
                del self.rx_buf[0]

    def parse_position(self, frame: bytes):
        station = frame[0]
        index = struct.unpack('>h', frame[2:4])[0]   # 有號 -32768 ~ 32767（圈數）
        step = struct.unpack('>H', frame[4:6])[0]    # 無號 0 ~ 10000

        if step > STEP_PER_REV:
            self.get_logger().warn(f"step 超出範圍: {step}")
            return

        # 保留比例：總圈數 = index + step/10000
        self.positions[station] = index + step / STEP_PER_REV

    # ---------------- 計算差值、累加、發布 ----------------
    def publish_feedback(self):
        if LEFT_STATION not in self.positions or RIGHT_STATION not in self.positions:
            return  # 兩顆馬達都收到資料後才開始

        now = self.get_clock().now().nanoseconds * 1e-9

        # 第一次：只記錄起點，不輸出差值
        if self.last_pub_time is None:
            for st in (LEFT_STATION, RIGHT_STATION):
                self.last_pub_pos[st] = self.positions[st]
            self.last_pub_time = now
            return

        dt = now - self.last_pub_time

        # 這段時間的圈數差值 = 現在圈數 - 上次圈數
        d_left = LEFT_SIGN * (self.positions[LEFT_STATION] - self.last_pub_pos[LEFT_STATION])
        d_right = RIGHT_SIGN * (self.positions[RIGHT_STATION] - self.last_pub_pos[RIGHT_STATION])

        # 差值加總
        self.total_revs[LEFT_STATION] += d_left
        self.total_revs[RIGHT_STATION] += d_right

        fb = Twist()
        fb.linear.x = d_left                           # 左輪這段時間圈數差
        fb.linear.y = d_right                          # 右輪這段時間圈數差
        fb.angular.x = self.total_revs[LEFT_STATION]   # 左輪累積總圈數
        fb.angular.y = self.total_revs[RIGHT_STATION]  # 右輪累積總圈數
        fb.angular.z = dt                              # 這段時間長度(秒)
        self.fb_pub.publish(fb)

        # 更新「上一次」
        for st in (LEFT_STATION, RIGHT_STATION):
            self.last_pub_pos[st] = self.positions[st]
        self.last_pub_time = now

        # 偵錯輸出（需要時取消註解）
        # circ = 3.14159265 * WHEEL_DIAMETER
        # self.get_logger().info(
        #     f"Δ左={d_left:+.4f}圈 Δ右={d_right:+.4f}圈 | "
        #     f"累計左={self.total_revs[LEFT_STATION]:+.3f}圈({self.total_revs[LEFT_STATION]*circ:+.3f}m) "
        #     f"累計右={self.total_revs[RIGHT_STATION]:+.3f}圈({self.total_revs[RIGHT_STATION]*circ:+.3f}m)")


def main(args=None):
    rclpy.init(args=args)
    node = MotorSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser is not None and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()