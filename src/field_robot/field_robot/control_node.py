"""3. Control Node: mode switch, PID waypoint tracking, arrival detection.

切到手動時：目標、PID、噴粉、輪速立即全部清掉，之後不再處理任何自動資料。
手動輪速需要持續收到 /manual_cmd（網頁每 0.5 s 心跳），超過 manual_timeout 自動歸零。
"""
import math

import rclpy
from geometry_msgs.msg import Pose2D, Vector3
from rclpy.node import Node
from std_msgs.msg import Bool, String


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class PID:
    def __init__(self, kp, ki, kd, limit):
        self.kp, self.ki, self.kd, self.limit = kp, ki, kd, limit
        self.i = 0.0
        self.prev = None

    def reset(self):
        self.i, self.prev = 0.0, None

    def step(self, err, dt):
        self.i = max(-self.limit, min(self.limit, self.i + err * dt))
        # 角度誤差在 ±pi 跳變時，用 wrap 取真正的變化量，避免微分尖峰
        d = 0.0 if self.prev is None else wrap(err - self.prev) / dt
        self.prev = err
        out = self.kp * err + self.ki * self.i + self.kd * d
        return max(-self.limit, min(self.limit, out))


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        for k, v in dict(wheel_radius=0.10, track_width=0.50,
                         v_max=0.6, v_min=0.15, w_max=1.2,
                         goal_tol=0.15, kp_dist=0.8,
                         turn_in=0.5, turn_out=0.35,        # 原地轉向遲滯 (rad)
                         rpm_deadzone=15.0,                 # 馬達死區補償：原地轉向時每輪最低 rpm（0 = 關閉）
                         heading_tol=0.05,                  # 前進時的角度死區 (rad)，誤差更小就不修正
                         goal_timeout=120.0,                # 單一航點逾時 (s)，逾時中止任務
                         manual_timeout=1.0,                # 手動指令逾時 (s)
                         spray_while_turning=False).items():
            self.declare_parameter(k, v)
        self.p = lambda k: self.get_parameter(k).value

        self.pub_motor = self.create_publisher(Vector3, '/motor_rpm_cmd', 10)  # x=left, y=right
        self.pub_reached = self.create_publisher(Bool, '/reached', 10)
        self.pub_trigger = self.create_publisher(Bool, '/goal_trigger', 10)
        self.pub_spray = self.create_publisher(Bool, '/spray_cmd', 10)
        self.pub_failed = self.create_publisher(Bool, '/mission_failed', 10)
        self.create_subscription(Pose2D, '/goal_pose', self.on_goal, 10)
        self.create_subscription(Pose2D, '/robot_pose', self.on_pose, 10)
        self.create_subscription(String, '/mode_cmd', self.on_mode, 10)
        self.create_subscription(Vector3, '/manual_cmd', self.on_manual, 10)

        self.mode = 'manual'
        self.goal = None
        self.goal_t0 = None
        self.pose = None
        self.turning = False
        self.manual = (0.0, 0.0)
        self.last_manual_time = self.get_clock().now()
        self.pid_w = PID(1.5, 0.0, 0.1, self.p('w_max'))
        self.dt = 0.05
        self.create_timer(self.dt, self.loop)

    def clear_auto(self):
        """清掉一切自動模式的狀態，並立刻停車、關噴粉。"""
        self.goal = None
        self.goal_t0 = None
        self.turning = False
        self.pid_w.reset()
        self.send_rpm(0.0, 0.0)
        self.pub_spray.publish(Bool(data=False))

    def on_mode(self, msg):
        if msg.data not in ('auto', 'manual') or msg.data == self.mode:
            return
        self.mode = msg.data
        self.manual = (0.0, 0.0)
        self.last_manual_time = self.get_clock().now()
        self.clear_auto()
        self.get_logger().info(f'mode -> {self.mode}')

    def on_manual(self, msg):
        self.manual = (msg.x, msg.y)
        self.last_manual_time = self.get_clock().now()

    def on_goal(self, msg):
        if self.mode == 'auto':                 # 手動模式下一律丟棄自動目標
            self.goal = msg
            self.goal_t0 = self.get_clock().now()
            self.turning = False
            self.pid_w.reset()

    def on_pose(self, msg):
        self.pose = msg

    def send_rpm(self, left, right):
        self.pub_motor.publish(Vector3(x=float(left), y=float(right)))

    def fail(self, why):
        self.get_logger().error(f'goal failed: {why} -> mission aborted')
        self.clear_auto()
        self.pub_failed.publish(Bool(data=True))

    def loop(self):
        now = self.get_clock().now()

        if self.mode == 'manual':
            if (now - self.last_manual_time).nanoseconds / 1e9 > self.p('manual_timeout'):
                self.manual = (0.0, 0.0)        # 網頁/心跳中斷 -> 停車
            self.send_rpm(*self.manual)
            self.pub_spray.publish(Bool(data=False))
            return

        # ---- auto ----
        if self.goal is not None and (now - self.goal_t0).nanoseconds / 1e9 > self.p('goal_timeout'):
            self.fail('timeout')
            return
        if self.goal is None or self.pose is None:
            self.send_rpm(0.0, 0.0)
            self.pub_spray.publish(Bool(data=False))      # 沒有目標或沒有定位時不噴
            return

        dx, dy = self.goal.x - self.pose.x, self.goal.y - self.pose.y
        dist = math.hypot(dx, dy)
        if dist < self.p('goal_tol'):
            self.send_rpm(0.0, 0.0)
            self.pub_spray.publish(Bool(data=False))
            self.goal = None
            self.pub_trigger.publish(Bool(data=True))
            self.pub_reached.publish(Bool(data=True))
            return

        err = wrap(math.atan2(dy, dx) - self.pose.theta)
        if self.turning:                                   # 遲滯：>turn_in 開始原地轉，<turn_out 才結束
            if abs(err) < self.p('turn_out'):
                self.turning = False
        elif abs(err) > self.p('turn_in'):
            self.turning = True

        if self.turning:
            v = 0.0
        else:
            v = max(self.p('v_min'), min(self.p('v_max'), self.p('kp_dist') * dist)) * math.cos(err)
        w = self.pid_w.step(err, self.dt)
        if not self.turning and abs(err) < self.p('heading_tol'):
            w = 0.0                                        # 角度死區：誤差很小就直線走，避免來回抖動

        # 只在前進時噴粉（原地轉彎不噴），要改回舊行為把 spray_while_turning 設 true
        self.pub_spray.publish(Bool(data=v > 0.0 or bool(self.p('spray_while_turning'))))

        L, r = self.p('track_width'), self.p('wheel_radius')
        to_rpm = 60.0 / (2.0 * math.pi * r)
        left = (v - w * L / 2.0) * to_rpm
        right = (v + w * L / 2.0) * to_rpm
        if self.turning:
            # 死區補償：誤差變小時 PID 輸出會低於馬達的啟動轉速，車子轉不動、卡在 turn_out 之外。
            # 原地轉向時每輪至少給 rpm_deadzone，轉進 turn_out 容許範圍後就會開始前進。
            dz = self.p('rpm_deadzone')
            sgn = 1.0 if err >= 0.0 else -1.0
            left = -sgn * max(abs(left), dz)
            right = sgn * max(abs(right), dz)
        self.send_rpm(left, right)


def main():
    rclpy.init()
    rclpy.spin(ControlNode())
    rclpy.shutdown()

