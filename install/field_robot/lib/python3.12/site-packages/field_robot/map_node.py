"""2. Map Node: 依「地圖型號」(field_library) 展開航點，逐點送出 Pose2D。

/map_select 內容為 JSON 字串（也接受純文字規格名稱）：
  {"preset":"成棒（國際規格）","spacing":27.43,"path":"perimeter_full"}
"""
import json
import math

import rclpy
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .field_library import get_base_coords, list_path_templates, list_presets


class MapNode(Node):
    def __init__(self):
        super().__init__('map_node')
        self.pub_goal = self.create_publisher(Pose2D, '/goal_pose', 10)
        self.pub_done = self.create_publisher(Bool, '/mission_done', 10)
        self.create_subscription(String, '/map_select', self.on_map, 10)
        self.create_subscription(String, '/mission_cmd', self.on_cmd, 10)
        self.create_subscription(Bool, '/reached', self.on_reached, 10)
        self.map_req = {}
        self.path = []
        self.idx = 0
        self.running = False

    def on_map(self, msg):
        try:
            self.map_req = json.loads(msg.data)
        except json.JSONDecodeError:
            self.map_req = {'preset': msg.data}

    def on_cmd(self, msg):
        if msg.data == 'start':
            try:
                self.path = self.build_path(self.map_req)
            except Exception as e:
                self.get_logger().error(f'bad map model {self.map_req}: {e}')
                return
            self.idx, self.running = 0, True
            self._send_goal()
        elif msg.data == 'stop':
            self.running = False

    def build_path(self, req):
        spacing = req.get('spacing')
        if spacing is None:
            spacing = list_presets()[req['preset']]['base_spacing_m']
        spacing = float(spacing)
        if not 5.0 <= spacing <= 50.0:
            raise ValueError(f'spacing {spacing} out of range')
        names = list_path_templates()[req.get('path', 'perimeter_full')]['waypoints']
        coords = get_base_coords(spacing)
        pts = [(coords[n].x, coords[n].y) for n in names]
        out = []
        for i, (x, y) in enumerate(pts):
            th = math.atan2(pts[i + 1][1] - y, pts[i + 1][0] - x) if i + 1 < len(pts) else out[-1][2]
            out.append((x, y, th))
        return out

    def on_reached(self, msg):
        if not (msg.data and self.running):
            return
        self.idx += 1
        if self.idx >= len(self.path):
            self.running = False
            self.pub_done.publish(Bool(data=True))
            self.get_logger().info('mission done')
        else:
            self._send_goal()

    def _send_goal(self):
        x, y, th = self.path[self.idx]
        self.pub_goal.publish(Pose2D(x=x, y=y, theta=th))
        self.get_logger().info(f'goal {self.idx + 1}/{len(self.path)}: ({x:.2f},{y:.2f})')


def main():
    rclpy.init()
    rclpy.spin(MapNode())
    rclpy.shutdown()
