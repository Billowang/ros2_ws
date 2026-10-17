"""1. Web UI Node: 靜態網頁 (HTTP 8080) + WebSocket (8765) <-> ROS 2 橋接。

瀏覽器 -> 本節點（只在事件發生時送）：
  {"type":"mode","value":"auto"|"manual"}
  {"type":"start","map":{"preset":..,"spacing":..,"path":..}}
  {"type":"stop"}
  {"type":"manual","left":rpm,"right":rpm}    變化時送；非零時每 0.5 s 心跳
  {"type":"estop"}
本節點 -> 瀏覽器：
  {"type":"hello","running":bool}   連線時 1 次
  {"type":"mission_done"}           任務完成時 1 次

ROS 2：
  發佈 /map_select(String, JSON)  /mission_cmd(String start|stop)
       /mode_cmd(String)          /manual_cmd(Vector3 x=左rpm y=右rpm)
  訂閱 /mission_done(Bool)
手動輪速只在「值改變」時才發佈到 ROS（control_node 會保持最後一筆）；
若 manual_timeout 秒內收不到網頁心跳，或網頁全部斷線，自動發佈 0。
"""
import asyncio
import functools
import json
import os
import threading
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

import rclpy
import websockets
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from std_msgs.msg import Bool, String

WEB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'web')


class QuietHandler(SimpleHTTPRequestHandler):
    def log_message(self, *args):
        pass


class WebUINode(Node):
    def __init__(self):
        super().__init__('web_ui_node')
        self.declare_parameter('ws_port', 8765)
        self.declare_parameter('http_port', 8080)
        self.declare_parameter('max_rpm', 400.0)
        self.declare_parameter('manual_timeout', 1.0)

        self.pub_map = self.create_publisher(String, '/map_select', 10)
        self.pub_mission = self.create_publisher(String, '/mission_cmd', 10)
        self.pub_mode = self.create_publisher(String, '/mode_cmd', 10)
        self.pub_manual = self.create_publisher(Vector3, '/manual_cmd', 10)
        self.create_subscription(Bool, '/mission_done', self.on_done, 10)

        self.running = False
        self.last_manual = (0.0, 0.0)
        self.last_manual_t = time.monotonic()
        self.ws_clients = set()
        self.loop = None
        self.create_timer(0.2, self.watchdog)

        threading.Thread(target=self._run_http, daemon=True).start()
        threading.Thread(target=self._run_ws, daemon=True).start()

    # ---------- 網頁 -> ROS ----------
    def publish_manual(self, left, right):
        self.last_manual = (left, right)
        self.pub_manual.publish(Vector3(x=left, y=right))

    def on_web_message(self, m):
        t = m.get('type')
        if t == 'start':
            self.pub_map.publish(String(data=json.dumps(m.get('map', {}), ensure_ascii=False)))
            self.pub_mission.publish(String(data='start'))
            self.running = True
        elif t == 'stop':
            self.pub_mission.publish(String(data='stop'))
            self.running = False
        elif t == 'mode' and m.get('value') in ('auto', 'manual'):
            self.pub_mode.publish(String(data=m['value']))
            self.last_manual = (0.0, 0.0)          # control_node 切模式時會把手動輪速歸零
            if m['value'] == 'manual':
                self.running = False
        elif t == 'manual':
            lim = self.get_parameter('max_rpm').value
            l = max(-lim, min(lim, float(m.get('left', 0))))
            r = max(-lim, min(lim, float(m.get('right', 0))))
            self.last_manual_t = time.monotonic()
            if (l, r) != self.last_manual:
                self.publish_manual(l, r)
        elif t == 'estop':
            self.pub_mission.publish(String(data='stop'))
            self.pub_mode.publish(String(data='manual'))
            self.publish_manual(0.0, 0.0)
            self.running = False

    def watchdog(self):
        if self.last_manual != (0.0, 0.0) and \
                time.monotonic() - self.last_manual_t > self.get_parameter('manual_timeout').value:
            self.get_logger().warn('web heartbeat lost -> manual rpm = 0')
            self.publish_manual(0.0, 0.0)

    # ---------- ROS -> 網頁 ----------
    def on_done(self, msg):
        if not msg.data:
            return
        self.running = False
        if self.loop:
            asyncio.run_coroutine_threadsafe(self._broadcast({'type': 'mission_done'}), self.loop)

    async def _broadcast(self, obj):
        text = json.dumps(obj)
        for ws in list(self.ws_clients):
            try:
                await ws.send(text)
            except Exception:
                self.ws_clients.discard(ws)

    # ---------- 伺服器 ----------
    async def _handler(self, ws):
        self.ws_clients.add(ws)
        try:
            await ws.send(json.dumps({'type': 'hello', 'running': self.running}))
            async for raw in ws:
                try:
                    self.on_web_message(json.loads(raw))
                except Exception as e:
                    self.get_logger().warn(f'bad message: {e}')
        except Exception:
            pass
        finally:
            self.ws_clients.discard(ws)
            if not self.ws_clients and self.last_manual != (0.0, 0.0):
                self.publish_manual(0.0, 0.0)       # 全部斷線 -> 停車

    def _run_ws(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)

        async def main():
            port = self.get_parameter('ws_port').value
            async with websockets.serve(self._handler, '0.0.0.0', port):
                self.get_logger().info(f'WebSocket ws://0.0.0.0:{port}')
                await asyncio.Future()
        self.loop.run_until_complete(main())

    def _run_http(self):
        port = self.get_parameter('http_port').value
        handler = functools.partial(QuietHandler, directory=WEB_DIR)
        httpd = ThreadingHTTPServer(('0.0.0.0', port), handler)
        self.get_logger().info(f'Web UI http://0.0.0.0:{port}')
        httpd.serve_forever()


def main():
    rclpy.init()
    rclpy.spin(WebUINode())
    rclpy.shutdown()
