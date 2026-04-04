from __future__ import annotations

import asyncio
import json
import time
import threading
import subprocess
from typing import Dict, Any, Optional

import yaml
import websockets

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

from motor_hw import build_motors_from_config, BTS7960Motor


def load_config(path: str) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def safe_float(x: Any, default: float = 0.0) -> float:
    try:
        return float(x)
    except Exception:
        return default


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def stop_all(motors: Dict[str, BTS7960Motor]) -> None:
    for m in motors.values():
        m.stop()


def apply_speeds(motors: Dict[str, BTS7960Motor], speeds: Dict[str, float]) -> None:
    for name, spd in speeds.items():
        if name in motors:
            motors[name].set_speed(spd)


def deadzone(v: float, dz: float) -> float:
    return 0.0 if abs(v) < dz else v


def ramp_towards(curr: float, target: float, dt: float, ramp_per_sec: float) -> float:
    """
    Slew-rate limiter.
    ramp_per_sec=1.5 means output can change by at most 1.5 units per second.
    """
    if ramp_per_sec <= 0:
        return target
    max_step = ramp_per_sec * dt
    if target > curr + max_step:
        return curr + max_step
    if target < curr - max_step:
        return curr - max_step
    return target


class BatterySubscriber(Node):
    def __init__(self):
        super().__init__('ws_battery_bridge')

        self.latest_percentage: Optional[int] = None
        self.latest_voltage: Optional[float] = None
        self.latest_stamp: Optional[float] = None
        self._lock = threading.Lock()

        self.subscription = self.create_subscription(
            BatteryState,
            '/battery',
            self.battery_callback,
            10
        )

        self.get_logger().info("Subscribed to /battery")

    def battery_callback(self, msg: BatteryState) -> None:
        try:
            raw_percentage = float(msg.percentage) * 100.0
            percentage = int(round(clamp(raw_percentage, 0.0, 100.0)))
        except Exception:
            percentage = None

        try:
            voltage = round(float(msg.voltage), 2)
        except Exception:
            voltage = None

        with self._lock:
            self.latest_percentage = percentage
            self.latest_voltage = voltage
            self.latest_stamp = time.time()

    def get_latest_battery(self) -> dict:
        with self._lock:
            return {
                "percentage": self.latest_percentage,
                "voltage": self.latest_voltage,
                "stamp": self.latest_stamp
            }


def ros_spin_thread(node: Node) -> None:
    rclpy.spin(node)


async def run_server(config_path: str) -> None:
    cfg = load_config(config_path)
    motors: Dict[str, BTS7960Motor] = build_motors_from_config(cfg)

    host = cfg.get("server", {}).get("host", "0.0.0.0")
    port = int(cfg.get("server", {}).get("port", 8766))
    watchdog_timeout = float(cfg.get("server", {}).get("watchdog_timeout_s", 0.4))

    # Profile used for cmd:on
    default_on = cfg.get("profiles", {}).get("default_on", {"right": 0.6, "left": 0.6, "blade": 1.0})

    # Teleop (joystick) tuning
    teleop = cfg.get("teleop", {})
    MAX_WHEEL = float(teleop.get("max_wheel", 1))
    DEADZONE = float(teleop.get("deadzone", 0.08))
    RAMP_PER_SEC = float(teleop.get("ramp_per_sec", 1.5))

    # Teleop state (smoothed outputs)
    current_left = 0.0
    current_right = 0.0
    last_update = time.time()

    last_cmd_time = time.time()

    # Obstacle avoidance process
    obstacle_process: Optional[subprocess.Popen] = None
    obstacle_script = "/home/lawnbot/mower_ws/src/obstacle_detection/obstacle_avoidance_pi.py"

    # ---------------- ROS2 battery subscriber setup ----------------
    rclpy.init(args=None)
    battery_node = BatterySubscriber()
    battery_thread = threading.Thread(target=ros_spin_thread, args=(battery_node,), daemon=True)
    battery_thread.start()
    # ---------------------------------------------------------------

    async def watchdog():
        nonlocal last_cmd_time
        if watchdog_timeout <= 0:
            return
        while True:
            await asyncio.sleep(0.05)
            if time.time() - last_cmd_time > watchdog_timeout:
                stop_all(motors)

    async def handler(ws):
        nonlocal last_cmd_time, current_left, current_right, last_update, obstacle_process

        await ws.send(json.dumps({
            "ok": True,
            "motors": list(motors.keys()),
            "commands": [
                "on",
                "stop",
                "set",
                "drive",
                "blade",
                "get_battery",
                "start_obstacle_avoidance",
                "stop_obstacle_avoidance"
            ],
            "port": port
        }))

        async for msg in ws:
            last_cmd_time = time.time()

            try:
                data = json.loads(msg)
            except Exception:
                await ws.send(json.dumps({"ok": False, "error": "invalid_json"}))
                continue

            cmd = str(data.get("cmd", "")).lower()

            # ---- STOP (stops everything) ----
            if cmd == "stop":
                stop_all(motors)
                current_left = 0.0
                current_right = 0.0
                last_update = time.time()
                await ws.send(json.dumps({"ok": True, "cmd": "stop"}))
                continue

            # ---- ON (profile: all motors) ----
            if cmd == "on":
                speeds = {k: safe_float(v, 0.0) for k, v in default_on.items()}
                apply_speeds(motors, speeds)
                await ws.send(json.dumps({"ok": True, "cmd": "on", "applied": speeds}))
                continue

            # ---- SET (optional manual override by names) ----
            if cmd == "set":
                speeds = {}
                for name in motors.keys():
                    if name in data:
                        speeds[name] = safe_float(data[name], 0.0)
                if not speeds:
                    await ws.send(json.dumps({"ok": False, "error": "no_motor_fields"}))
                    continue
                apply_speeds(motors, speeds)
                await ws.send(json.dumps({"ok": True, "cmd": "set", "applied": speeds}))
                continue

            # ---- DRIVE (joystick / teleop) ----
            # Expects: {"cmd":"drive","left":..,"right":..}
            if cmd == "drive":
                if "left" not in motors or "right" not in motors:
                    await ws.send(json.dumps({"ok": False, "error": "missing_left_or_right_motor"}))
                    continue

                print("RAW:", data.get("left"), data.get("right"), "MAX_WHEEL:", MAX_WHEEL)

                target_left = deadzone(safe_float(data.get("left", 0.0), 0.0), DEADZONE)
                target_right = deadzone(safe_float(data.get("right", 0.0), 0.0), DEADZONE)

                target_left = clamp(target_left, -MAX_WHEEL, MAX_WHEEL)
                target_right = clamp(target_right, -MAX_WHEEL, MAX_WHEEL)

                now = time.time()
                dt = max(0.0, now - last_update)
                last_update = now

                current_left = ramp_towards(current_left, target_left, dt, RAMP_PER_SEC)
                current_right = ramp_towards(current_right, target_right, dt, RAMP_PER_SEC)

                motors["left"].set_speed(current_left)
                motors["right"].set_speed(current_right)

                await ws.send(json.dumps({"ok": True, "cmd": "drive"}))
                continue

            # ---- BLADE (separate) ----
            # Expects: {"cmd":"blade","speed":0..1}
            if cmd == "blade":
                if "blade" not in motors:
                    await ws.send(json.dumps({"ok": False, "error": "missing_blade_motor"}))
                    continue
                blade_speed = safe_float(data.get("speed", 0.0), 0.0)
                blade_speed = clamp(blade_speed, 0.0, 0.1)
                motors["blade"].set_speed(blade_speed)
                await ws.send(json.dumps({"ok": True, "cmd": "blade", "speed": blade_speed}))
                continue

            # ---- GET_BATTERY ----
            # Expects: {"cmd":"get_battery"}
            if cmd == "get_battery":
                batt = battery_node.get_latest_battery()
                await ws.send(json.dumps({
                    "ok": True,
                    "cmd": "battery",
                    "percentage": batt["percentage"],
                    "voltage": batt["voltage"]
                }))
                continue

            # ---- START_OBSTACLE_AVOIDANCE ----
            # Expects: {"cmd":"start_obstacle_avoidance"}
            if cmd == "start_obstacle_avoidance":
                if obstacle_process is None or obstacle_process.poll() is not None:
                    try:
                        obstacle_process = subprocess.Popen(
                            [obstacle_script],
                            cwd="/home/lawnbot/mower_ws/src/obstacle_detection"
                        )
                        await ws.send(json.dumps({
                            "ok": True,
                            "cmd": "start_obstacle_avoidance"
                        }))
                    except Exception as e:
                        await ws.send(json.dumps({
                            "ok": False,
                            "error": "failed_to_start_obstacle_avoidance",
                            "details": str(e)
                        }))
                else:
                    await ws.send(json.dumps({
                        "ok": False,
                        "error": "obstacle_avoidance_already_running"
                    }))
                continue

            # ---- STOP_OBSTACLE_AVOIDANCE ----
            # Expects: {"cmd":"stop_obstacle_avoidance"}
            if cmd == "stop_obstacle_avoidance":
                if obstacle_process is not None and obstacle_process.poll() is None:
                    try:
                        obstacle_process.terminate()
                        obstacle_process.wait(timeout=5)
                    except Exception:
                        try:
                            obstacle_process.kill()
                        except Exception:
                            pass
                    obstacle_process = None
                    await ws.send(json.dumps({
                        "ok": True,
                        "cmd": "stop_obstacle_avoidance"
                    }))
                else:
                    obstacle_process = None
                    await ws.send(json.dumps({
                        "ok": False,
                        "error": "obstacle_avoidance_not_running"
                    }))
                continue

            await ws.send(json.dumps({"ok": False, "error": "unknown_cmd", "got": cmd}))

    wd_task = asyncio.create_task(watchdog())
    try:
        async with websockets.serve(handler, host, port, ping_interval=20, ping_timeout=20):
            print(
                f"[lawnbot_motors] ws://{host}:{port} "
                f"(ON: {{'cmd':'on'}} | STOP: {{'cmd':'stop'}} | "
                f"DRIVE: {{'cmd':'drive','left':..,'right':..}} | "
                f"GET_BATTERY: {{'cmd':'get_battery'}} | "
                f"START_OBSTACLE_AVOIDANCE: {{'cmd':'start_obstacle_avoidance'}} | "
                f"STOP_OBSTACLE_AVOIDANCE: {{'cmd':'stop_obstacle_avoidance'}})"
            )
            await asyncio.Future()
    finally:
        wd_task.cancel()

        if obstacle_process is not None and obstacle_process.poll() is None:
            try:
                obstacle_process.terminate()
                obstacle_process.wait(timeout=5)
            except Exception:
                try:
                    obstacle_process.kill()
                except Exception:
                    pass

        stop_all(motors)
        for m in motors.values():
            m.disable()

        battery_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    asyncio.run(run_server("motor_config.yaml"))
