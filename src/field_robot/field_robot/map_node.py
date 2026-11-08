"""2. Map Node: 依「地圖型號」(field_library) 展開航點，逐點送出 Pose2D。

/mission_cmd (String) 兩種格式：
  JSON（建議，地圖資料跟著指令一起到，不會有跨 topic 的先後問題）：
    {"cmd":"start","map":{"preset":"成棒（國際規格）","spacing":27.43,"path":"perimeter_full"}}
  純文字 "start" / "stop"：start 時使用最近一次 /map_select 的內容（相容舊流程）
/mode_cmd 變成 manual 時：任務、路徑、進度全部清掉，不再送任何目標。
/mission_failed：控制節點回報航點逾時 -> 任務中止。
"""
import json
import math

import rclpy
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, String

from .field_library import get_base_coords, list_path_templates, list_presets


class MapNode(Node):
    def __init__(self):
        super().__init__('map_node')
        self.pub_goal = self.create_publisher(Pose2D, '/goal_pose', 10)
        self.pub_done = self.create_publisher(Bool, '/mission_done', 10)
        self.create_subscription(String, '/map_select', self.on_map, 10)
        self.create_subscription(String, '/mission_cmd', self.on_cmd, 10)
        self.create_subscription(Bool, '/reached', self.on_reached, 10)
        self.create_subscription(Bool, '/mission_failed', self.on_failed, 10)
        self.create_subscription(String, '/mode_cmd', self.on_mode, 10)
        self.create_subscription(Empty, '/heading_zeroed', self.on_zeroed, 10)
        self.waiting_zero = False
        self.zero_timer = self.create_timer(0.5, self.on_zero_timeout)   # 等不到 fusion 時的保險
        self.zero_timer.cancel()
        self.map_req = {}
        self.path = []
        self.idx = 0
        self.running = False

    def reset_mission(self):
        self.running = False
        self.waiting_zero = False
        self.zero_timer.cancel()
        self.path = []
        self.idx = 0

    def on_map(self, msg):
        try:
            self.map_req = json.loads(msg.data)
        except json.JSONDecodeError:
            self.map_req = {'preset': msg.data}

    def on_mode(self, msg):
        if msg.data == 'manual' and (self.running or self.path):
            self.reset_mission()
            self.get_logger().info('mode -> manual: mission cleared')

    def on_failed(self, msg):
        if msg.data and self.running:
            self.reset_mission()
            self.get_logger().error('mission aborted by control_node')

    def on_cmd(self, msg):
        cmd, req = msg.data, None
        if msg.data.lstrip().startswith('{'):
            try:
                d = json.loads(msg.data)
            except json.JSONDecodeError:
                self.get_logger().error(f'bad mission_cmd JSON: {msg.data!r}')
                return
            cmd, req = d.get('cmd'), d.get('map')

        if cmd == 'start':
            req = req if req is not None else self.map_req
            try:
                self.path = self.build_path(req)
            except Exception as e:
                self.get_logger().error(f'bad map model {req}: {e}')
                return
            self.map_req = req
            self.idx, self.running = 0, True
            # 先等 fusion 把朝向歸零（/heading_zeroed）才送第一個目標，車子不會用舊座標先動
            self.waiting_zero = True
            self.zero_timer.reset()
        elif cmd == 'stop':
            self.running = False
            self.waiting_zero = False
            self.zero_timer.cancel()

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
            if i + 1 < len(pts):
                th = math.atan2(pts[i + 1][1] - y, pts[i + 1][0] - x)
            else:
                th = out[-1][2] if out else 0.0
            out.append((x, y, th))
        return out

    def on_zeroed(self, _):
        if self.waiting_zero and self.running:
            self._release_start()

    def on_zero_timeout(self):
        self.zero_timer.cancel()
        if self.waiting_zero and self.running:
            self.get_logger().warn('沒等到 /heading_zeroed，直接開始')
            self._release_start()

    def _release_start(self):
        self.waiting_zero = False
        self.zero_timer.cancel()
        self._send_goal()

    def on_reached(self, msg):
        if not (msg.data and self.running) or self.waiting_zero:
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
