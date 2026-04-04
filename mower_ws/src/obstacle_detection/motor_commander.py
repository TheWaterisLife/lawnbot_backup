"""Translates obstacle avoidance decisions into motor WebSocket commands.

Manages a persistent WebSocket connection to the already-running
ws_motor_server.py on port 8766, sends drive/stop commands, and reads
responses to keep the channel healthy.
"""

import asyncio
import json
import logging
import time

import websockets

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Tuneable speed constants (0.0 – 1.0 PWM duty cycle)
# ---------------------------------------------------------------------------
SPEED_FORWARD = 0.40    # Straight-line cruising
SPEED_SLOW = 0.30       # Minimum forward speed (never below this when moving)
SPEED_TURN_OUTER = 0.45 # Outer wheel during arc turn
SPEED_TURN_INNER = -0.25 # Inner wheel reverses hard for aggressive turn
SPEED_PIVOT = 0.45      # Both wheels during in-place pivot
PIVOT_180_DURATION = 2.0

WS_URI = "ws://127.0.0.1:8766"
RECONNECT_DELAY = 0.3


def _ws_is_open(ws) -> bool:
    """Check if a websocket connection is open (compatible with all versions)."""
    if ws is None:
        return False
    try:
        return not ws.closed
    except AttributeError:
        pass
    try:
        from websockets.protocol import State
        return ws.state == State.OPEN
    except Exception:
        pass
    try:
        return ws.open
    except AttributeError:
        return False


class MotorCommander:
    """Sends drive/stop commands directly to ws_motor_server.py."""

    def __init__(self):
        self._ws = None
        self._pivot_180_start: float = 0.0
        self._in_pivot_180: bool = False

    # ------------------------------------------------------------------
    # Connection management
    # ------------------------------------------------------------------

    async def connect(self) -> bool:
        if _ws_is_open(self._ws):
            return True
        try:
            self._ws = await asyncio.wait_for(
                websockets.connect(WS_URI, ping_interval=20, ping_timeout=20),
                timeout=3.0,
            )
            welcome = await asyncio.wait_for(self._ws.recv(), timeout=2.0)
            logger.info("Motor server connected: %s", welcome)
            return True
        except Exception as e:
            logger.warning("Motor connect failed: %s", e)
            self._ws = None
            return False

    async def ensure_connected(self) -> None:
        while not await self.connect():
            await asyncio.sleep(RECONNECT_DELAY)

    async def _send(self, payload: dict) -> bool:
        for attempt in range(2):
            try:
                if not _ws_is_open(self._ws):
                    ok = await self.connect()
                    if not ok:
                        return False
                await self._ws.send(json.dumps(payload))
                resp = await asyncio.wait_for(self._ws.recv(), timeout=1.0)
                data = json.loads(resp)
                return data.get("ok", False)
            except Exception as e:
                if attempt == 0:
                    logger.debug("Motor send retry after: %s", e)
                    try:
                        if self._ws:
                            await self._ws.close()
                    except Exception:
                        pass
                    self._ws = None
                else:
                    logger.warning("Motor send failed: %s", e)
                    self._ws = None
                    return False
        return False

    async def close(self) -> None:
        if self._ws:
            try:
                await self._ws.close()
            except Exception:
                pass
            self._ws = None

    # ------------------------------------------------------------------
    # Motor actions
    # ------------------------------------------------------------------

    async def drive(self, left: float, right: float, label: str = "") -> bool:
        ok = await self._send({"cmd": "drive", "left": float(left), "right": float(right)})
        logger.debug("MOTOR >> %s  L=%.2f R=%.2f (ok=%s)", label or "DRIVE", left, right, ok)
        return ok

    async def stop(self, label: str = "STOP") -> bool:
        ok = await self._send({"cmd": "stop"})
        logger.info("MOTOR >> %s (ok=%s)", label, ok)
        return ok

    async def execute(self, action_name: str, speed_factor: float = 1.0) -> None:
        """Execute a named avoidance action.

        Args:
            action_name: One of: none, slow_down, stop, turn_left, turn_right,
                         pivot_left, pivot_right, pivot_180
            speed_factor: 0.0–1.0 from obstacle handler (scales forward speed).
                         Clamped so the mower never gets a value too low to move.
        """
        speed = max(SPEED_SLOW, SPEED_FORWARD * speed_factor)

        if action_name == "none":
            self._in_pivot_180 = False
            await self.drive(speed, speed, "FORWARD")

        elif action_name == "slow_down":
            self._in_pivot_180 = False
            await self.drive(SPEED_SLOW, SPEED_SLOW, "SLOW")

        elif action_name == "stop":
            self._in_pivot_180 = False
            await self.stop()

        elif action_name == "turn_left":
            self._in_pivot_180 = False
            await self.drive(SPEED_TURN_OUTER, SPEED_TURN_INNER, "TURN_LEFT")

        elif action_name == "turn_right":
            self._in_pivot_180 = False
            await self.drive(SPEED_TURN_INNER, SPEED_TURN_OUTER, "TURN_RIGHT")

        elif action_name == "pivot_left":
            self._in_pivot_180 = False
            await self.drive(SPEED_PIVOT, -SPEED_PIVOT, "PIVOT_LEFT")

        elif action_name == "pivot_right":
            self._in_pivot_180 = False
            await self.drive(-SPEED_PIVOT, SPEED_PIVOT, "PIVOT_RIGHT")

        elif action_name == "pivot_180":
            if not self._in_pivot_180:
                self._in_pivot_180 = True
                self._pivot_180_start = time.time()
                logger.info("Starting 180 pivot")
            if time.time() - self._pivot_180_start < PIVOT_180_DURATION:
                await self.drive(-SPEED_PIVOT, SPEED_PIVOT, "PIVOT_180")
            else:
                self._in_pivot_180 = False
                await self.stop("PIVOT_180_DONE")

        else:
            logger.warning("Unknown action %s — stopping", action_name)
            await self.stop("UNKNOWN")

    async def emergency_stop(self) -> None:
        self._in_pivot_180 = False
        await self.stop("EMERGENCY_STOP")
