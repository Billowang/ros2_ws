"""1. Web UI Node: 靜態網頁 (HTTP 8080) + WebSocket (8765) <-> ROS 2 橋接。

瀏覽器 -> 本節點（只在事件發生時送）：
  {"type":"auth","token":"..."}                      auth_token 有設定時，必須是連線後第一個訊息
  {"type":"mode","value":"auto"|"manual"}
  {"type":"start","map":{"preset":..,"spacing":..,"path":..}}
  {"type":"stop"}
  {"type":"manual","left":rpm,"right":rpm}          變化時送；非零時每 0.5 s 心跳（每筆都會轉發到 ROS）
  {"type":"estop"}
本節點 -> 瀏覽器：
  {"type":"hello","running":bool,"max_rpm":float}   連線時 1 次（max_rpm = 網頁輪速上限的唯一來源）
  {"type":"mission_done"}                           任務完成時 1 次
  {"type":"mission_failed"}                         航點逾時、任務被中止時 1 次

ROS 2：
  發佈 /mission_cmd(String, JSON {"cmd":"start","map":{...}} 或 "stop")
       /mode_cmd(String)          /manual_cmd(Vector3 x=左rpm y=右rpm)
  訂閱 /mission_done(Bool)  /mission_failed(Bool)
手動輪速每收到一筆網頁訊息就轉發一次（control_node 靠它判斷網頁還活著）；
若 manual_timeout 秒內收不到網頁心跳，或網頁全部斷線，自動發佈 0。
切到手動時一併送出 stop，確保 map_node 的任務被清掉。

安全：設定 auth_token 參數後，瀏覽器網址要帶 ?token=xxx，沒帶或不符的連線會被拒絕。
"""
import asyncio
import functools
import hmac
import json
import math
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
        self.declare_parameter('max_rpm', 100.0)       # 網頁手動輪速上限（約 1.05 m/s @ 輪徑 0.2 m）
        self.declare_parameter('manual_timeout', 1.0)
        self.declare_parameter('auth_token', '')       # 空字串 = 不驗證

        self.pub_mission = self.create_publisher(String, '/mission_cmd', 10)
        self.pub_mode = self.create_publisher(String, '/mode_cmd', 10)
        self.pub_manual = self.create_publisher(Vector3, '/manual_cmd', 10)
        self.create_subscription(Bool, '/mission_done', self.on_done, 10)
        self.create_subscription(Bool, '/mission_failed', self.on_failed, 10)

        self.running = False
        self.last_manual = (0.0, 0.0)
        self.last_manual_t = time.monotonic()
        self.ws_clients = set()
        self.loop = None
        self.create_timer(0.2, self.watchdog)

        if not self.get_parameter('auth_token').value:
            self.get_logger().warn('auth_token 未設定：同一網路的任何人都能操控機器人')

        threading.Thread(target=self._run_http, daemon=True).start()
        threading.Thread(target=self._run_ws, daemon=True).start()

    # ---------- 網頁 -> ROS ----------
    def publish_manual(self, left, right):
        self.last_manual = (left, right)
        self.pub_manual.publish(Vector3(x=left, y=right))

    def on_web_message(self, m):
        t = m.get('type')
        if t == 'start':
            payload = {'cmd': 'start', 'map': m.get('map', {})}      # 地圖跟著指令一起送
            self.pub_mission.publish(String(data=json.dumps(payload, ensure_ascii=False)))
            self.running = True
        elif t == 'stop':
            self.pub_mission.publish(String(data='stop'))
            self.running = False
        elif t == 'mode' and m.get('value') in ('auto', 'manual'):
            if m['value'] == 'manual':
                self.pub_mission.publish(String(data='stop'))
                self.running = False
            self.pub_mode.publish(String(data=m['value']))
            self.last_manual = (0.0, 0.0)          # control_node 切模式時會把手動輪速歸零
        elif t == 'manual':
            lim = self.get_parameter('max_rpm').value
            l, r = float(m.get('left', 0)), float(m.get('right', 0))
            if not (math.isfinite(l) and math.isfinite(r)):
                return
            l, r = max(-lim, min(lim, l)), max(-lim, min(lim, r))
            self.last_manual_t = time.monotonic()
            self.publish_manual(l, r)              # 每筆都轉發（含心跳）
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
    def _notify(self, obj):
        if self.loop:
            asyncio.run_coroutine_threadsafe(self._broadcast(obj), self.loop)

    def on_done(self, msg):
        if not msg.data:
            return
        self.running = False
        self._notify({'type': 'mission_done'})

    def on_failed(self, msg):
        if not msg.data:
            return
        self.running = False
        self._notify({'type': 'mission_failed'})

    async def _broadcast(self, obj):
        text = json.dumps(obj)
        for ws in list(self.ws_clients):
            try:
                await ws.send(text)
            except Exception:
                self.ws_clients.discard(ws)

    # ---------- 伺服器 ----------
    async def _authenticate(self, ws, token):
        try:
            first = json.loads(await asyncio.wait_for(ws.recv(), 3.0))
            ok = first.get('type') == 'auth' and hmac.compare_digest(
                str(first.get('token', '')).encode(), token.encode())
        except Exception:
            ok = False
        if not ok:
            self.get_logger().warn('unauthorized websocket rejected')
            try:
                await ws.close(code=1008, reason='unauthorized')
            except Exception:
                pass
        return ok

    async def _handler(self, ws):
        token = self.get_parameter('auth_token').value
        if token and not await self._authenticate(ws, token):
            return
        self.ws_clients.add(ws)
        try:
            await ws.send(json.dumps({'type': 'hello', 'running': self.running,
                                      'max_rpm': self.get_parameter('max_rpm').value}))
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
