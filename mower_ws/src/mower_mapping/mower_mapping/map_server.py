#!/usr/bin/env python3
import os
import json
import math
import time
import asyncio
from dataclasses import dataclass
from typing import List, Optional, Tuple, Dict, Any

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Empty

import websockets

WS_HOST = "0.0.0.0"
WS_PORT = 8770
MAPS_DIR = os.path.expanduser("~/mower_ws/maps")

MIN_DIST_M = 0.25
MIN_HEADING_DEG = 8.0
MAX_TIME_S = 1.0
MIN_TIME_S = 0.2

NO_FIX_TIMEOUT_S = 2.0
RDP_EPS_M = 0.08
CLOSE_LOOP_DIST_M = 0.50
MIN_POINTS = 20


def deg2rad(d: float) -> float:
    return d * math.pi / 180.0


def rad2deg(r: float) -> float:
    return r * 180.0 / math.pi


def latlon_to_local_xy_m(lat: float, lon: float, lat0: float, lon0: float) -> Tuple[float, float]:
    R = 6371000.0
    x = deg2rad(lon - lon0) * R * math.cos(deg2rad(lat0))
    y = deg2rad(lat - lat0) * R
    return (x, y)


def dist_xy(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def heading_deg(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    if abs(dx) < 1e-9 and abs(dy) < 1e-9:
        return 0.0
    ang = math.atan2(dy, dx)
    hdg = rad2deg(ang)
    if hdg < 0:
        hdg += 360.0
    return hdg


def smallest_angle_diff_deg(a: float, b: float) -> float:
    d = (b - a + 180.0) % 360.0 - 180.0
    return abs(d)


def _perp_dist(p, a, b) -> float:
    ax, ay = a
    bx, by = b
    px, py = p
    vx, vy = (bx - ax, by - ay)
    wx, wy = (px - ax, py - ay)
    vv = vx * vx + vy * vy
    if vv < 1e-12:
        return math.hypot(wx, wy)
    t = (wx * vx + wy * vy) / vv
    t = max(0.0, min(1.0, t))
    projx = ax + t * vx
    projy = ay + t * vy
    return math.hypot(px - projx, py - projy)


def rdp(points: List[Tuple[float, float]], eps: float) -> List[Tuple[float, float]]:
    if len(points) < 3:
        return points[:]
    a = points[0]
    b = points[-1]
    max_d = -1.0
    idx = -1
    for i in range(1, len(points) - 1):
        d = _perp_dist(points[i], a, b)
        if d > max_d:
            max_d = d
            idx = i
    if max_d > eps:
        left = rdp(points[: idx + 1], eps)
        right = rdp(points[idx:], eps)
        return left[:-1] + right
    else:
        return [a, b]


@dataclass
class FixSample:
    lat: float
    lon: float
    ts: float
    x: float
    y: float


class MapServer(Node):
    def __init__(self):
        super().__init__("map_server")

        os.makedirs(MAPS_DIR, exist_ok=True)

        # Only subscribes to /rtk/fix (does NOT touch serial)
        self.create_subscription(NavSatFix, "/rtk/fix", self.on_fix, 10)

        # Autonomy control (event triggers)
        self.pub_autonomy_start = self.create_publisher(Empty, "/autonomy/start", 10)
        self.pub_autonomy_stop = self.create_publisher(Empty, "/autonomy/stop", 10)

        self.latest_fix: Optional[FixSample] = None
        self.state = "IDLE"  # IDLE | MAPPING | PAUSED
        self.origin_latlon: Optional[Tuple[float, float]] = None
        self.points_raw: List[FixSample] = []
        self.last_saved_time: Optional[float] = None
        self.last_saved_xy: Optional[Tuple[float, float]] = None
        self.last_heading: Optional[float] = None
        self.last_fix_rx_time: Optional[float] = None

        self.ws_clients: set = set()
        self._last_built_map: Optional[Dict[str, Any]] = None

        self.create_timer(0.2, self.on_timer)  # 5 Hz tick
        self.get_logger().info(f"MapServer ready. WS ws://{WS_HOST}:{WS_PORT}, topic /rtk/fix")

    def on_fix(self, msg: NavSatFix):
        now = time.time()
        self.last_fix_rx_time = now

        lat = float(msg.latitude)
        lon = float(msg.longitude)

        if self.origin_latlon is None:
            self.origin_latlon = (lat, lon)

        lat0, lon0 = self.origin_latlon
        x, y = latlon_to_local_xy_m(lat, lon, lat0, lon0)

        self.latest_fix = FixSample(lat=lat, lon=lon, ts=now, x=x, y=y)

        if self.state == "MAPPING":
            self.maybe_record_point(self.latest_fix)

    def on_timer(self):
        now = time.time()

        if self.state == "MAPPING":
            if self.last_fix_rx_time is None or (now - self.last_fix_rx_time) > NO_FIX_TIMEOUT_S:
                self.state = "PAUSED"
                self.get_logger().warn("No RTK updates -> PAUSED mapping")
                asyncio.create_task(self.ws_broadcast_status())

        elif self.state == "PAUSED":
            if self.last_fix_rx_time is not None and (now - self.last_fix_rx_time) <= 0.6:
                self.state = "MAPPING"
                self.get_logger().info("RTK updates returned -> resumed MAPPING")
                asyncio.create_task(self.ws_broadcast_status())

        asyncio.create_task(self.ws_broadcast_pose())
        asyncio.create_task(self.ws_broadcast_status())
        if self.state in ("MAPPING", "PAUSED") and len(self.points_raw) >= 2:
            asyncio.create_task(self.ws_broadcast_preview())

    def maybe_record_point(self, fix: FixSample):
        now = fix.ts

        if self.last_saved_time is not None and (now - self.last_saved_time) < MIN_TIME_S:
            return

        if self.last_saved_xy is None:
            self.points_raw.append(fix)
            self.last_saved_xy = (fix.x, fix.y)
            self.last_saved_time = now
            self.last_heading = None
            return

        last_xy = self.last_saved_xy
        cur_xy = (fix.x, fix.y)

        d = dist_xy(last_xy, cur_xy)

        hdg_trigger = False
        if len(self.points_raw) >= 2:
            prev_xy = (self.points_raw[-1].x, self.points_raw[-1].y)
            hdg_now = heading_deg(prev_xy, cur_xy)
            if self.last_heading is None:
                self.last_heading = hdg_now
            else:
                if smallest_angle_diff_deg(self.last_heading, hdg_now) >= MIN_HEADING_DEG:
                    hdg_trigger = True

        time_trigger = (self.last_saved_time is None) or ((now - self.last_saved_time) >= MAX_TIME_S)
        dist_trigger = d >= MIN_DIST_M

        if dist_trigger or hdg_trigger or time_trigger:
            self.points_raw.append(fix)
            self.last_saved_xy = cur_xy
            self.last_saved_time = now
            if len(self.points_raw) >= 2:
                prev_xy = (self.points_raw[-2].x, self.points_raw[-2].y)
                self.last_heading = heading_deg(prev_xy, cur_xy)

    def start_mapping(self):
        if self.latest_fix is None:
            return False, "No RTK fix received yet."
        self.state = "MAPPING"
        self.points_raw = []
        self.last_saved_time = None
        self.last_saved_xy = None
        self.last_heading = None

        self.origin_latlon = (self.latest_fix.lat, self.latest_fix.lon)
        self.get_logger().info("Mapping started.")
        return True, "Mapping started."

    def stop_mapping_and_build(self):
        if self.state not in ("MAPPING", "PAUSED"):
            return False, "Not currently mapping."
        self.state = "IDLE"

        if len(self.points_raw) < MIN_POINTS:
            return False, f"Not enough points ({len(self.points_raw)})."

        xy = [(p.x, p.y) for p in self.points_raw]

        if dist_xy(xy[0], xy[-1]) > CLOSE_LOOP_DIST_M:
            xy.append(xy[0])

        simplified = rdp(xy, RDP_EPS_M)
        if simplified[0] != simplified[-1]:
            simplified.append(simplified[0])

        result = {
            "origin_latlon": [self.origin_latlon[0], self.origin_latlon[1]],
            "raw_points_xy": xy,
            "boundary_xy": simplified,
            "raw_count": len(xy),
            "boundary_count": len(simplified),
            "created_at": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime()),
        }
        self.get_logger().info(f"Mapping stopped. raw={len(xy)} simplified={len(simplified)}")
        return True, result

    def save_map(self, name: str, map_obj: Dict[str, Any]):
        safe = "".join(c for c in name.strip().lower().replace(" ", "_") if (c.isalnum() or c in "_-"))
        if not safe:
            safe = "map"
        ts = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        map_id = f"{safe}_{ts}"
        path = os.path.join(MAPS_DIR, f"{map_id}.json")
        out = {"id": map_id, "name": name.strip(), **map_obj}
        with open(path, "w") as f:
            json.dump(out, f, indent=2)
        return map_id, path

    def list_maps(self):
        os.makedirs(MAPS_DIR, exist_ok=True)
        res = []
        for fn in sorted(os.listdir(MAPS_DIR)):
            if fn.endswith(".json"):
                try:
                    with open(os.path.join(MAPS_DIR, fn), "r") as f:
                        obj = json.load(f)
                    res.append({
                        "id": obj.get("id", fn[:-5]),
                        "name": obj.get("name", fn[:-5]),
                        "created_at": obj.get("created_at", "")
                    })
                except Exception:
                    pass
        return res

    def load_map(self, map_id: str):
        path = os.path.join(MAPS_DIR, f"{map_id}.json")
        if not os.path.exists(path):
            return None
        with open(path, "r") as f:
            return json.load(f)

    async def ws_handler(self, websocket):
        self.ws_clients.add(websocket)
        try:
            await websocket.send(json.dumps(self.make_status_msg()))
            while True:
                raw = await websocket.recv()
                msg = json.loads(raw)
                await self.handle_ws_msg(websocket, msg)
        except websockets.ConnectionClosed:
            pass
        finally:
            self.ws_clients.discard(websocket)

    async def handle_ws_msg(self, websocket, msg: Dict[str, Any]):
        # Support BOTH:
        # - mapping protocol: {"type":"map_start", ...}
        # - command protocol: {"cmd":"start_navigation"}
        cmd = msg.get("cmd", "")
        t = msg.get("type", "")

        # --- Navigation commands ---
        if cmd == "start_navigation":
            self.pub_autonomy_start.publish(Empty())
            await websocket.send(json.dumps({"ok": True, "msg": "navigation_started"}))
            return

        if cmd == "stop_navigation":
            self.pub_autonomy_stop.publish(Empty())
            await websocket.send(json.dumps({"ok": True, "msg": "navigation_stopped"}))
            return

        # --- Mapping commands ---
        if t == "map_start":
            ok, info = self.start_mapping()
            await websocket.send(json.dumps({"type": "map_start_result", "ok": ok, "info": info}))
            await self.ws_broadcast_status()

        elif t == "map_stop":
            ok, info = self.stop_mapping_and_build()
            if ok:
                self._last_built_map = info
                await websocket.send(json.dumps({"type": "map_stop_result", "ok": True, "map": info}))
            else:
                await websocket.send(json.dumps({"type": "map_stop_result", "ok": False, "info": info}))
            await self.ws_broadcast_status()

        elif t == "map_save":
            name = msg.get("name", "map")
            if self._last_built_map is None:
                await websocket.send(json.dumps({
                    "type": "map_save_result",
                    "ok": False,
                    "info": "No map to save. Stop mapping first."
                }))
                return
            map_id, path = self.save_map(name, self._last_built_map)
            await websocket.send(json.dumps({"type": "map_save_result", "ok": True, "id": map_id, "path": path}))

        elif t == "map_list":
            await websocket.send(json.dumps({"type": "map_list_result", "maps": self.list_maps()}))

        elif t == "map_load":
            map_id = msg.get("id", "")
            obj = self.load_map(map_id)
            if obj is None:
                await websocket.send(json.dumps({"type": "map_load_result", "ok": False, "info": "Map not found"}))
            else:
                await websocket.send(json.dumps({"type": "map_load_result", "ok": True, "map": obj}))

        elif t == "map_cancel":
            self.state = "IDLE"
            self.points_raw = []
            await websocket.send(json.dumps({"type": "map_cancel_result", "ok": True}))
            await self.ws_broadcast_status()

        else:
            await websocket.send(json.dumps({"type": "error", "info": f"Unknown type/cmd: type={t} cmd={cmd}"}))

    def make_status_msg(self):
        return {"type": "map_status", "state": self.state, "count": len(self.points_raw)}

    async def ws_broadcast_status(self):
        if not self.ws_clients:
            return
        msg = json.dumps(self.make_status_msg())
        await asyncio.gather(*[self.safe_send(ws, msg) for ws in list(self.ws_clients)])

    async def ws_broadcast_pose(self):
        if not self.ws_clients or self.latest_fix is None:
            return
        msg = json.dumps({
            "type": "rtk_pose",
            "lat": self.latest_fix.lat,
            "lon": self.latest_fix.lon,
            "ts": self.latest_fix.ts
        })
        await asyncio.gather(*[self.safe_send(ws, msg) for ws in list(self.ws_clients)])

    async def ws_broadcast_preview(self):
        if not self.ws_clients or len(self.points_raw) < 2:
            return
        pts = [[p.x, p.y] for p in self.points_raw]
        msg = json.dumps({"type": "map_preview", "points": pts})
        await asyncio.gather(*[self.safe_send(ws, msg) for ws in list(self.ws_clients)])

    async def safe_send(self, ws, msg: str):
        try:
            await ws.send(msg)
        except Exception:
            self.ws_clients.discard(ws)


async def main_async():
    rclpy.init()
    node = MapServer()

    ws_server = await websockets.serve(node.ws_handler, WS_HOST, WS_PORT)
    node.get_logger().info("WebSocket server started.")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            await asyncio.sleep(0.01)
    finally:
        ws_server.close()
        await ws_server.wait_closed()
        node.destroy_node()
        rclpy.shutdown()


def main():
    asyncio.run(main_async())


if __name__ == "__main__":
    main()
