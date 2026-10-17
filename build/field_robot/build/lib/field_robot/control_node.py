"""3. Control Node: mode switch, PID waypoint tracking, arrival detection."""
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
        d = 0.0 if self.prev is None else (err - self.prev) / dt
        self.prev = err
        out = self.kp * err + self.ki * self.i + self.kd * d
        return max(-self.limit, min(self.limit, out))


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        for k, v in dict(wheel_radius=0.10, track_width=0.50, v_max=0.6, w_max=1.2,
                         goal_tol=0.15, kp_dist=0.8, spray_grace=1.5,
                         right_sign=-1.0).items():   # right_sign: right motor is mirrored
            self.declare_parameter(k, v)
        self.p = lambda k: self.get_parameter(k).value

        self.pub_motor = self.create_publisher(Vector3, '/motor_rpm_cmd', 10)  # x=left, y=right
        self.pub_reached = self.create_publisher(Bool, '/reached', 10)
        self.pub_trigger = self.create_publisher(Bool, '/goal_trigger', 10)
        self.pub_spray = self.create_publisher(Bool, '/spray_cmd', 10)
        self.create_subscription(Pose2D, '/goal_pose', self.on_goal, 10)
        self.create_subscription(Pose2D, '/robot_pose', self.on_pose, 10)
        self.create_subscription(String, '/mode_cmd', self.on_mode, 10)
        self.create_subscription(Vector3, '/manual_cmd', self.on_manual, 10)

        self.mode = 'manual'
        self.goal = None
        self.pose = None
        self.manual = (0.0, 0.0)
        self.last_goal_time = None
        self.pid_w = PID(1.5, 0.0, 0.1, self.p('w_max'))
        self.dt = 0.05
        self.create_timer(self.dt, self.loop)

    def on_mode(self, msg):
        if msg.data in ('auto', 'manual') and msg.data != self.mode:
            self.mode = msg.data
            self.goal = None
            self.pid_w.reset()
            self.manual = (0.0, 0.0)
            self.get_logger().info(f'mode -> {self.mode}')

    def on_manual(self, msg):
        self.manual = (msg.x, msg.y)

    def on_goal(self, msg):
        if self.mode == 'auto':
            self.goal = msg
            self.last_goal_time = self.get_clock().now()
            self.pid_w.reset()

    def on_pose(self, msg):
        self.pose = msg

    def send_rpm(self, left, right):
        self.pub_motor.publish(Vector3(x=float(left), y=float(right)))

    def loop(self):
        if self.mode == 'manual':
            self.send_rpm(self.manual[0], self.manual[1] * self.p('right_sign'))
            self.pub_spray.publish(Bool(data=False))
            return

        # auto: spray on while following the map (short grace between waypoints)
        active = self.goal is not None
        if not active and self.last_goal_time is not None:
            active = (self.get_clock().now() - self.last_goal_time).nanoseconds / 1e9 < self.p('spray_grace')
        self.pub_spray.publish(Bool(data=active))

        if self.goal is None or self.pose is None:
            self.send_rpm(0.0, 0.0)
            return

        dx, dy = self.goal.x - self.pose.x, self.goal.y - self.pose.y
        dist = math.hypot(dx, dy)
        if dist < self.p('goal_tol'):
            self.send_rpm(0.0, 0.0)
            self.goal = None
            self.last_goal_time = self.get_clock().now()
            self.pub_trigger.publish(Bool(data=True))
            self.pub_reached.publish(Bool(data=True))
            return

        err = wrap(math.atan2(dy, dx) - self.pose.theta)
        if abs(err) > 0.5:                      # turn in place first
            v = 0.0
        else:
            v = min(self.p('v_max'), self.p('kp_dist') * dist) * math.cos(err)
        w = self.pid_w.step(err, self.dt)

        L, r = self.p('track_width'), self.p('wheel_radius')
        to_rpm = 60.0 / (2.0 * math.pi * r)
        left = (v - w * L / 2.0) * to_rpm
        right = (v + w * L / 2.0) * to_rpm * self.p('right_sign')
        self.send_rpm(left, right)


def main():
    rclpy.init()
    rclpy.spin(ControlNode())
    rclpy.shutdown()
