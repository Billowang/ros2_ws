#!/usr/bin/env python3
"""
control_panel/server.py
========================
把網頁前端（index.html / app.js）跟 ROS 2 Jazzy 接起來的橋接伺服器。

架構：
  瀏覽器  <== WebSocket (ws://<pi-ip>:8765) ==>  這支程式  <== rclpy topics ==>  你的 ROS 2 節點

功能：
  1. 用 `websockets` 開一個 WebSocket 伺服器，接收前端傳來的控制指令
     {"type":"control","throttle":-100..100,"steer":-100..100,
      "lock":bool,"estop":bool,"cruise":bool,"ts":epoch_ms}
  2. 把指令轉成 ROS 2 topic 發佈出去（預設：/cmd_vel 給 Twist，
     /control/lock、/control/estop、/control/cruise 給 Bool）。
  3. 訂閱馬達轉速、電池狀態等 topic，轉成
     {"type":"telemetry", "left_rpm":.., "right_rpm":.., "battery_pct":.., ...}
     廣播回所有已連線的網頁。
  4. 用一個簡單的 HTTP 伺服器把 index.html / style.css / app.js 這個資料夾
     服務出來（預設 port 8080），方便你之前的 nginx captive portal
     直接 reverse proxy 到這個 port。

沒有安裝 rclpy 時（例如在筆電上先測試前端）會自動切換成「模擬模式」，
用你送出的 throttle 產生假的 rpm/電池資料，這樣不裝 ROS 2 也能先把
網頁滑桿、鎖定、急停等互動都測完。

安裝需求：
    pip install websockets
    # 在有 ROS 2 Jazzy 環境的機器上，rclpy 已經隨 ROS 2 安裝好了，不用額外裝

執行方式：
    source /opt/ros/jazzy/setup.bash   # 如果要接 ROS 2
    python3 server.py
"""

import asyncio
import json
import logging
import math
import os
import threading
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

import websockets

# ---------------------------------------------------------------------------
# 設定
# ---------------------------------------------------------------------------
WS_HOST = "0.0.0.0"
WS_PORT = 8765
HTTP_HOST = "0.0.0.0"
HTTP_PORT = 8080
STATIC_DIR = os.path.dirname(os.path.abspath(__file__))

CONTROL_HZ_TIMEOUT = 0.5   # 超過這麼久沒收到前端控制訊息就視為斷線，自動急停
TELEMETRY_HZ = 10          # 回傳遙測資料的頻率

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
log = logging.getLogger("bridge")

# ---------------------------------------------------------------------------
# 共用狀態（在 asyncio 執行緒跟 ROS 2 執行緒之間共享）
# ---------------------------------------------------------------------------
class SharedState:
    def __init__(self):
        self.lock = threading.Lock()
        # 最新收到的前端控制指令
        self.control = {
            "throttle": 0, "steer": 0,
            "lock": True, "estop": False, "cruise": False,
            "last_update": 0.0,
        }
        # 最新的遙測資料（由 ROS 2 subscriber 或模擬器更新）
        self.telemetry = {
            "left_rpm": 0.0, "right_rpm": 0.0,
            "battery_pct": 100.0, "voltage": 25.2, "current": 0.0,
            "batt_time_min": 240.0,
        }

    def set_control(self, msg):
        with self.lock:
            self.control.update({
                "throttle": clamp(msg.get("throttle", 0), -100, 100),
                "steer": clamp(msg.get("steer", 0), -100, 100),
                "lock": bool(msg.get("lock", True)),
                "estop": bool(msg.get("estop", False)),
                "cruise": bool(msg.get("cruise", False)),
                "last_update": time.time(),
            })

    def get_control(self):
        with self.lock:
            return dict(self.control)

    def set_telemetry(self, **kwargs):
        with self.lock:
            self.telemetry.update(kwargs)

    def get_telemetry(self):
        with self.lock:
            return dict(self.telemetry)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


state = SharedState()

# ---------------------------------------------------------------------------
# ROS 2 Jazzy 橋接（有 rclpy 才會啟用）
# ---------------------------------------------------------------------------
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Bool, Float32
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


if ROS2_AVAILABLE:

    class ControlBridgeNode(Node):
        """把網頁控制指令發佈成 ROS 2 topic，並訂閱馬達/電池 topic 回傳給網頁。"""

        def __init__(self, shared_state: SharedState, on_telemetry_update):
            super().__init__("web_control_bridge")
            self.shared = shared_state
            self.on_telemetry_update = on_telemetry_update

            # --- 發佈：把 throttle/steer 轉成 Twist 發到 /cmd_vel ---
            self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
            self.lock_pub = self.create_publisher(Bool, "/control/lock", 10)
            self.estop_pub = self.create_publisher(Bool, "/control/estop", 10)
            self.cruise_pub = self.create_publisher(Bool, "/control/cruise", 10)

            # --- 訂閱：另一個節點回報馬達轉速 / 電池狀態 ---
            self.create_subscription(Float32, "/motor/left_rpm", self._on_left_rpm, 10)
            self.create_subscription(Float32, "/motor/right_rpm", self._on_right_rpm, 10)
            self.create_subscription(Float32, "/battery/percentage", self._on_battery, 10)

            # 以固定頻率把目前的控制狀態發佈出去（就算網頁沒有新事件也要持續送出，
            # 這樣下游節點才能偵測到「斷線=無新指令」並自行觸發保護機制）
            self.create_timer(1.0 / 20.0, self._publish_control)

        def _publish_control(self):
            ctrl = self.shared.get_control()
            stale = (time.time() - ctrl["last_update"]) > CONTROL_HZ_TIMEOUT and ctrl["last_update"] > 0
            effective_estop = ctrl["estop"] or stale

            twist = Twist()
            if not ctrl["lock"] and not effective_estop:
                twist.linear.x = ctrl["throttle"] / 100.0     # -1.0 .. 1.0
                twist.angular.z = -ctrl["steer"] / 100.0       # 左正右負，依你機器人座標系調整
            self.cmd_vel_pub.publish(twist)

            self.lock_pub.publish(Bool(data=ctrl["lock"]))
            self.estop_pub.publish(Bool(data=effective_estop))
            self.cruise_pub.publish(Bool(data=ctrl["cruise"]))

        def _on_left_rpm(self, msg: Float32):
            self.shared.set_telemetry(left_rpm=msg.data)
            self.on_telemetry_update()

        def _on_right_rpm(self, msg: Float32):
            self.shared.set_telemetry(right_rpm=msg.data)
            self.on_telemetry_update()

        def _on_battery(self, msg: Float32):
            self.shared.set_telemetry(battery_pct=msg.data)
            self.on_telemetry_update()


def run_ros2_thread(shared_state: SharedState, on_telemetry_update, loop_ready_event):
    """在背景執行緒跑 rclpy 的 spin，避免擋住 asyncio 事件迴圈。"""
    rclpy.init()
    node = ControlBridgeNode(shared_state, on_telemetry_update)
    log.info("ROS 2 Jazzy 橋接節點已啟動 (/cmd_vel, /control/lock, /control/estop, /control/cruise)")
    loop_ready_event.set()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


# ---------------------------------------------------------------------------
# 沒有 ROS 2 時的模擬模式（純前端測試用）
# ---------------------------------------------------------------------------
def run_simulator_thread(shared_state: SharedState, on_telemetry_update):
    log.warning("找不到 rclpy，改用模擬模式產生假遙測資料（僅供前端測試）")
    battery = 89.0
    while True:
        ctrl = shared_state.get_control()
        target_l = ctrl["throttle"] + ctrl["steer"] * 0.3
        target_r = ctrl["throttle"] - ctrl["steer"] * 0.3
        left_rpm = clamp(target_l, -100, 100) * 3.6
        right_rpm = clamp(target_r, -100, 100) * 3.6
        battery = max(0.0, battery - 0.0007 * (abs(ctrl["throttle"]) + 1))
        shared_state.set_telemetry(
            left_rpm=left_rpm, right_rpm=right_rpm,
            battery_pct=battery,
            voltage=24.0 + battery / 100.0,
            current=1.0 + abs(ctrl["throttle"]) / 20.0,
            batt_time_min=battery * 2.5,
        )
        on_telemetry_update()
        time.sleep(1.0 / TELEMETRY_HZ)


# ---------------------------------------------------------------------------
# WebSocket 伺服器
# ---------------------------------------------------------------------------
connected_clients = set()
main_loop = None  # asyncio event loop，供背景執行緒安全地排程廣播


def schedule_broadcast():
    """由 ROS 2 執行緒 / 模擬執行緒呼叫，把「有新遙測資料」的通知丟回 asyncio 迴圈。"""
    if main_loop is not None:
        try:
            main_loop.call_soon_threadsafe(asyncio.create_task, broadcast_telemetry())
        except RuntimeError:
            pass  # loop 尚未就緒或正在關閉


async def broadcast_telemetry():
    if not connected_clients:
        return
    payload = json.dumps({"type": "telemetry", **state.get_telemetry()})
    dead = []
    for ws in connected_clients:
        try:
            await ws.send(payload)
        except websockets.ConnectionClosed:
            dead.append(ws)
    for ws in dead:
        connected_clients.discard(ws)


async def handle_client(websocket):
    connected_clients.add(websocket)
    log.info("網頁前端已連線：%s", websocket.remote_address)
    try:
        async for raw in websocket:
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                continue
            if msg.get("type") == "control":
                state.set_control(msg)
    except websockets.ConnectionClosed:
        pass
    finally:
        connected_clients.discard(websocket)
        log.info("網頁前端已斷線：%s", websocket.remote_address)


async def periodic_telemetry_push():
    """就算沒有新資料，也定期把目前狀態推給前端，確保畫面持續更新。"""
    while True:
        await broadcast_telemetry()
        await asyncio.sleep(1.0 / TELEMETRY_HZ)


# ---------------------------------------------------------------------------
# 靜態檔案伺服器（index.html / style.css / app.js）
# ---------------------------------------------------------------------------
def run_static_server():
    os.chdir(STATIC_DIR)
    handler = SimpleHTTPRequestHandler
    httpd = ThreadingHTTPServer((HTTP_HOST, HTTP_PORT), handler)
    log.info("靜態網頁伺服器啟動於 http://%s:%d （可交給 nginx reverse proxy）", HTTP_HOST, HTTP_PORT)
    httpd.serve_forever()


# ---------------------------------------------------------------------------
# 主程式
# ---------------------------------------------------------------------------
async def async_main():
    global main_loop
    main_loop = asyncio.get_running_loop()

    threading.Thread(target=run_static_server, daemon=True).start()

    if ROS2_AVAILABLE:
        ready = threading.Event()
        threading.Thread(
            target=run_ros2_thread, args=(state, schedule_broadcast, ready), daemon=True
        ).start()
        ready.wait(timeout=5.0)
    else:
        threading.Thread(
            target=run_simulator_thread, args=(state, schedule_broadcast), daemon=True
        ).start()

    async with websockets.serve(handle_client, WS_HOST, WS_PORT):
        log.info("WebSocket 伺服器啟動於 ws://%s:%d", WS_HOST, WS_PORT)
        await periodic_telemetry_push()


def main():
    """給 ros2 run 呼叫的同步進入點。"""
    try:
        asyncio.run(async_main())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
