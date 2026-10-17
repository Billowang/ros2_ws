import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class CmdVelFilter(Node):
    def __init__(self):
        super().__init__('cmd_vel_filter_node')


        # 1. 訂閱來自網頁的控制指令 (原始指令)
        self.raw_sub = self.create_subscription(
            Twist,
            '/cmd_vel',  # 建議將網頁發出的 Topic 改名為 raw，避免混淆
            self.cmd_callback,
            10)


        # 2. 訂閱來自雷達節點的安全狀態
        self.safety_sub = self.create_subscription(
            Bool,
            '/distance',
            self.safety_callback,
            10)


        # 3. 發布經過過濾、安全的指令給馬達驅動節點
        self.safe_pub = self.create_publisher(Twist, '/cmd_vel_safe', 10)


        # 內部狀態
        self.is_path_clear = True  # 預設為通行


    def safety_callback(self, msg):
        """ 更新目前的路径安全狀態 """
        self.is_path_clear = msg.data


    def cmd_callback(self, msg):
        """ 接收原始指令並進行安全過濾 """
        safe_msg = Twist()
       
        # 複製原本的倍率設定 (linear.z) 與 旋轉速度 (angular.z)
        # 旋轉與倍率通常不受到前方障礙物限制 (除非你想做 360 度全方位防撞)
        safe_msg.linear.z = msg.linear.z
        safe_msg.angular.z = msg.angular.z


        # --- 安全邏輯判斷 ---
        if not self.is_path_clear:
            # 如果 path_clear 是 False
            if msg.linear.x > 0:
                # 使用者想「前進」，強行攔截設為 0
                safe_msg.linear.x = 0.0
                safe_msg.angular.z = 0.0
                self.get_logger().warn("⚠️ 偵測障礙物：已過濾前進指令！")
            else:
                # 使用者想「後退」或「停止」，則允許通過
                safe_msg.linear.x = msg.linear.x
        else:
            # 路徑通暢，直接透傳
            safe_msg.linear.x = msg.linear.x


        # 發布過濾後的訊息
        self.safe_pub.publish(safe_msg)


def main():
    rclpy.init()
    node = CmdVelFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()