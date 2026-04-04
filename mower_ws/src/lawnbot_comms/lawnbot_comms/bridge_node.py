#!/usr/bin/env python3
import asyncio
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, String
from geometry_msgs.msg import Twist

import websockets

WS_HOST = "0.0.0.0"
WS_PORT = 9002

class BridgeNode(Node):
    def __init__(self):
        super().__init__("lawnbot_bridge")

        # Phone -> ROS
        self.pub_heartbeat = self.create_publisher(Empty, "/app/heartbeat", 10)
        self.pub_connected = self.create_publisher(Bool, "/app/connected", 10)
        self.pub_mode      = self.create_publisher(String, "/app/mode", 10)
        self.pub_teleop    = self.create_publisher(Twist, "/app/teleop_cmd", 10)

        # ROS -> Phone
        self.sub_state     = self.create_subscription(String, "/mower/state", self._on_state, 10)
        self.sub_telemetry = self.create_subscription(String, "/mower/telemetry", self._on_telemetry, 10)

        self._last_heartbeat = 0.0
        self._client_connected = False

        # Async queue for outgoing messages
        self._send_queue = asyncio.Queue()

        self.get_logger().info(f"Listening on ws://{WS_HOST}:{WS_PORT}")

    def _on_state(self, msg: String):
        asyncio.get_event_loop().call_soon_threadsafe(
            self._send_queue.put_nowait,
            {"type": "state", "data": msg.data}
        )

    def _on_telemetry(self, msg: String):
        asyncio.get_event_loop().call_soon_threadsafe(
            self._send_queue.put_nowait,
            {"type": "telemetry", "data": msg.data}
        )

    def publish_heartbeat(self):
        self.pub_heartbeat.publish(Empty())
        self._last_heartbeat = time.time()

    def publish_mode(self, mode_value: str):
        m = String()
        m.data = mode_value
        self.pub_mode.publish(m)

    def publish_teleop(self, linear: float, angular: float):
        t = Twist()
        t.linear.x = float(linear)
        t.angular.z = float(angular)
        self.pub_teleop.publish(t)

    def heartbeat_age_ms(self) -> int:
        if self._last_heartbeat == 0.0:
            return 10**9
        return int((time.time() - self._last_heartbeat) * 1000)

async def ws_handler(websocket, node: BridgeNode):
    if node._client_connected:
        await websocket.send(json.dumps({"type":"error","msg":"another client already connected"}))
        await websocket.close()
        return

    node._client_connected = True
    node.pub_connected.publish(Bool(data=True))
    node.get_logger().info("Phone connected.")
    await websocket.send(json.dumps({"type":"ack","msg":"connected","version":1}))

    async def rx_loop():
        async for raw in websocket:
            try:
                msg = json.loads(raw)
            except Exception:
                continue

            mtype = msg.get("type", "")
            if mtype == "hello":
                await websocket.send(json.dumps({"type":"ack","hello":True,"version":1}))
            elif mtype == "heartbeat":
                node.publish_heartbeat()
            elif mtype == "mode":
                node.publish_mode(str(msg.get("value", "IDLE")))
            elif mtype == "teleop":
                node.publish_teleop(msg.get("linear", 0.0), msg.get("angular", 0.0))
            # ignore unknown

    async def tx_loop():
        while True:
            try:
                item = await asyncio.wait_for(node._send_queue.get(), timeout=0.2)
                await websocket.send(json.dumps(item))
            except asyncio.TimeoutError:
                await websocket.send(json.dumps({"type":"comms","heartbeat_age_ms": node.heartbeat_age_ms()}))
            except Exception:
                break

    try:
        await asyncio.gather(rx_loop(), tx_loop())
    finally:
        node._client_connected = False
        node.pub_connected.publish(Bool(data=False))
        node.get_logger().info("Phone disconnected.")

async def main_async(node: BridgeNode):
    server = await websockets.serve(lambda ws: ws_handler(ws, node), WS_HOST, WS_PORT)
    node.get_logger().info("WebSocket server started.")

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        await asyncio.sleep(0.01)

    server.close()
    await server.wait_closed()

def main():
    rclpy.init()
    node = BridgeNode()
    try:
        asyncio.run(main_async(node))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
