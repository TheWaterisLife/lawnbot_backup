# ws_motor_server.py
from __future__ import annotations

import asyncio
import json
import time
from typing import Dict, Any

import yaml
import websockets

from motor_hw import build_motors_from_config, BTS7960Motor


def load_config(path: str) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def safe_float(x: Any, default: float = 0.0) -> float:
    try:
        return float(x)
    except Exception:
        return default


async def motor_server(config_path: str) -> None:
    cfg = load_config(config_path)
    motors: Dict[str, BTS7960Motor] = build_motors_from_config(cfg)

    host = cfg.get("server", {}).get("host", "0.0.0.0")
    port = int(cfg.get("server", {}).get("port", 8766))
    watchdog_timeout = float(cfg.get("server", {}).get("watchdog_timeout_s", 1.0))

    last_cmd_time = time.time()

    async def watchdog_task():
        nonlocal last_cmd_time
        while True:
            await asyncio.sleep(0.05)
            if time.time() - last_cmd_time > watchdog_timeout:
                for m in motors.values():
                    m.stop()

    async def handler(ws):
        nonlocal last_cmd_time
        await ws.send(json.dumps({"ok": True, "motors": list(motors.keys())}))

        async for msg in ws:
            last_cmd_time = time.time()

            try:
                data = json.loads(msg)
            except Exception:
                await ws.send(json.dumps({"ok": False, "error": "invalid_json"}))
                continue

            # Stop all
            if data.get("stop") is True:
                for m in motors.values():
                    m.stop()
                await ws.send(json.dumps({"ok": True, "stopped": True}))
                continue

            # Mode 1: {"motor":"right","speed":0.6}
            if "motor" in data and "speed" in data:
                name = str(data["motor"])
                if name not in motors:
                    await ws.send(json.dumps({"ok": False, "error": f"unknown_motor:{name}"}))
                    continue
                motors[name].set_speed(safe_float(data["speed"], 0.0))
                await ws.send(json.dumps({"ok": True, "motor": name}))
                continue

            # Mode 2: {"right":0.6,"left":0.6,"blade":1.0}
            updated = []
            for name, val in data.items():
                if name in motors:
                    motors[name].set_speed(safe_float(val, 0.0))
                    updated.append(name)

            if updated:
                await ws.send(json.dumps({"ok": True, "updated": updated}))
            else:
                await ws.send(json.dumps({"ok": False, "error": "no_valid_motor_fields"}))

    # Start watchdog
    wd = asyncio.create_task(watchdog_task())

    try:
        async with websockets.serve(handler, host, port, ping_interval=20, ping_timeout=20):
            print(f"[ws_motor_server] listening on ws://{host}:{port}")
            await asyncio.Future()  # run forever
    finally:
        wd.cancel()
        for m in motors.values():
            m.disable()


if __name__ == "__main__":
    # default path
    asyncio.run(motor_server("motor_config.yaml"))

