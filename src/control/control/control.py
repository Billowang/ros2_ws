import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        self.web_sub = self.create_subscription(
            Twist,
            '/message',
            self.web_callback,
            10)

        self.map_sub = self.create_subscription(
            Pose2D,
            '/future',
            self.map_callback,
            10)

        self.mix_data_sub = self.create_subscription(
            Pose2D,
            '/mix_situation',
            self.mix_data_callback,
            10)

        self.motor_pub = self.create_publisher(Twist, '/speed', 10)

        self.map_pub = self.create_publisher(uint8, '/speed', 10)



    def web_callback(self, msg):

    def map_callback(self, msg):

    def mix_data_callback(self, msg):
   
def main():
    rclpy.init()
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()