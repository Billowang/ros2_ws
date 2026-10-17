"""8. Goal Arrival Trigger Node: output 1 for 20 s after each arrival, then 0."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class TriggerNode(Node):
    def __init__(self):
        super().__init__('trigger_node')
        self.declare_parameter('hold_sec', 20.0)
        self.out = False
        self.deadline = None
        self.pub = self.create_publisher(Bool, '/goal_trigger_out', 10)
        self.create_subscription(Bool, '/goal_trigger', self.on_trigger, 10)
        self.create_timer(0.1, self.tick)

    def on_trigger(self, msg):
        if msg.data:       # a new arrival restarts the 20 s window
            self.out = True
            self.deadline = self.get_clock().now().nanoseconds / 1e9 + self.get_parameter('hold_sec').value

    def tick(self):
        if self.out and self.get_clock().now().nanoseconds / 1e9 >= self.deadline:
            self.out = False
        self.pub.publish(Bool(data=self.out))


def main():
    rclpy.init()
    rclpy.spin(TriggerNode())
    rclpy.shutdown()
