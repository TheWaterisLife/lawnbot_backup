#!/usr/bin/env python3
import asyncio
import json
import time
from dataclasses import dataclass
from typing import Optional

import websockets


@dataclass
class MotorWsConfig:
    host: str = "127.0.0.1"
    port: int = 8766
    connect_timeout_s: float = 2.0
    reconnect_delay_s: float = 0.5
    send_min_interval_s: float = 0.05   # 20 Hz max
    send_retry_once: bool = True        # if send fails, reconnect and retry once


class MotorWsClient:
    """
    Simple, robust websocket client for ws_motor_server.py
    Commands sent:
      {"cmd":"drive","left":X,"right":Y}
      {"cmd":"blade","speed":S}
      {"cmd":"stop"}
    """

    def __init__(self, cfg: MotorWsConfig = MotorWsConfig()):
        self.cfg = cfg
        self._ws: Optional[websockets.WebSocketClientProtocol] = None
        self._last_send_t = 0.0

    @property
    def uri(self) -> str:
        return f"ws://{self.cfg.host}:{self.cfg.port}"

    def is_connected(self) -> bool:
        return self._ws is not None and not self._ws.closed

    async def connect(self) -> bool:
        if self.is_connected():
            return True
        try:
            self._ws = await asyncio.wait_for(
                websockets.connect(self.uri, ping_interval=10, ping_timeout=10),
                timeout=self.cfg.connect_timeout_s
            )
            return True
        except Exception:
            self._ws = None
            return False

    async def ensure_connected(self) -> bool:
        while True:
            ok = await self.connect()
            if ok:
                return True
            await asyncio.sleep(self.cfg.reconnect_delay_s)

    async def close(self):
        if self._ws is not None:
            try:
                await self._ws.close()
            except Exception:
                pass
        self._ws = None

    async def _throttle(self):
        now = time.time()
        dt = now - self._last_send_t
        if dt < self.cfg.send_min_interval_s:
            await asyncio.sleep(self.cfg.send_min_interval_s - dt)

    async def _send_once(self, payload: dict) -> bool:
        await self._throttle()
        try:
            if not self.is_connected():
                return False
            await self._ws.send(json.dumps(payload))
            self._last_send_t = time.time()
            return True
        except Exception:
            await self.close()
            return False

    async def _send(self, payload: dict) -> bool:
        """
        Send once, if fails optionally reconnect and retry once.
        """
        ok = await self._send_once(payload)
        if ok:
            return True

        if not self.cfg.send_retry_once:
            return False

        # Retry once after reconnect
        await self.ensure_connected()
        ok2 = await self._send_once(payload)
        return ok2

    async def drive(self, left: float, right: float) -> bool:
        return await self._send({"cmd": "drive", "left": float(left), "right": float(right)})

    async def blade(self, speed: float = 1.0) -> bool:
        return await self._send({"cmd": "blade", "speed": float(speed)})

    async def stop(self) -> bool:
        return await self._send({"cmd": "stop"})


# quick manual test
if __name__ == "__main__":
    async def main():
        c = MotorWsClient(MotorWsConfig(host="127.0.0.1", port=8766))
        print("Connecting to", c.uri)
        await c.ensure_connected()

        print("Blade ON")
        await c.blade(1.0)
        await asyncio.sleep(1.0)

        print("Pivot")
        await c.drive(0.2, -0.2)
        await asyncio.sleep(1.0)

        print("STOP")
        await c.stop()
        await asyncio.sleep(0.2)

        await c.close()

    asyncio.run(main())
