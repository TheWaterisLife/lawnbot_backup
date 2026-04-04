"""
Motor Controller Module (WebSocket Client).

Story 4.1: PWM Motor Control
Story 4.2: Differential Drive Control
Story 4.3: Safety Limits

Sends drive commands to the lawnbot_motors WebSocket server
instead of controlling GPIO directly.
"""

from dataclasses import dataclass
from typing import Tuple, Optional, Callable
import asyncio
import json
import time
import threading
import logging

logger = logging.getLogger(__name__)


# =============================================================================
# Configuration
# =============================================================================

@dataclass
class MotorConfig:
    """Motor controller configuration (WebSocket client to lawnbot_motors)."""

    # WebSocket connection to lawnbot_motors server
    ws_host: str = "localhost"
    ws_port: int = 8766

    # Speed limits
    max_speed: float = 1.0       # Maximum speed (0-1)

    # Safety
    watchdog_timeout: float = 0.5  # Stop if no command for this long


# =============================================================================
# Motor Controller (WebSocket Client)
# =============================================================================

class MotorController:
    """
    Motor controller that delegates to lawnbot_motors via WebSocket.

    Connects to the lawnbot_motors WebSocket server and sends
    drive/stop commands. All GPIO control, ramping, deadzone, and
    watchdog logic is handled by the lawnbot_motors server.

    Usage:
        config = MotorConfig()
        controller = MotorController(config)
        controller.start()

        # Set speeds
        controller.set_speeds(left=0.5, right=0.5)  # Forward
        controller.set_speeds(left=-0.5, right=0.5)  # Turn left

        # Or use velocity interface
        controller.set_velocity(vx=0.5, wz=0.0)  # Forward
        controller.set_velocity(vx=0.0, wz=0.5)  # Rotate

        controller.stop()

    Requires lawnbot_motors server running:
        cd ~/mower_ws/src/lawnbot_motors
        python3 ws_motor_server.py
    """

    def __init__(
        self,
        config: Optional[MotorConfig] = None,
        track_width: float = 0.24,  # For velocity conversion
    ):
        """
        Initialize motor controller.

        Args:
            config: Motor configuration
            track_width: Distance between tracks in meters
        """
        self._config = config or MotorConfig()
        self._track_width = track_width

        # Current state
        self._left_speed = 0.0   # -1 to 1
        self._right_speed = 0.0  # -1 to 1
        self._target_left = 0.0
        self._target_right = 0.0

        # Connection state
        self._running = False
        self._connected = False
        self._ws = None
        self._thread: Optional[threading.Thread] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._lock = threading.Lock()

        # Watchdog
        self._last_command_time = 0.0
        self._watchdog_triggered = False

        # Emergency stop callback
        self._emergency_stop_callback: Optional[Callable[[], None]] = None

    def set_emergency_stop_callback(self, callback: Callable[[], None]) -> None:
        """Set callback for emergency stop events."""
        self._emergency_stop_callback = callback

    def start(self) -> bool:
        """
        Start motor controller and connect to lawnbot_motors server.

        Returns:
            True if started successfully
        """
        if self._running:
            return True

        try:
            self._running = True
            self._last_command_time = time.time()

            # Start the async event loop in a background thread
            self._thread = threading.Thread(target=self._run_event_loop, daemon=True)
            self._thread.start()

            # Wait briefly for connection
            timeout = 3.0
            start = time.time()
            while not self._connected and time.time() - start < timeout:
                time.sleep(0.1)

            if self._connected:
                logger.info(
                    f"Motor controller connected to ws://"
                    f"{self._config.ws_host}:{self._config.ws_port}"
                )
                return True
            else:
                logger.warning(
                    f"Motor controller started but not yet connected to "
                    f"ws://{self._config.ws_host}:{self._config.ws_port} "
                    f"(will keep retrying in background)"
                )
                # Still return True — we'll auto-reconnect
                return True

        except Exception as e:
            logger.error(f"Failed to start motor controller: {e}")
            self._running = False
            return False

    def stop(self) -> None:
        """Stop motor controller and disconnect."""
        self._running = False

        # Send stop command before disconnecting
        self._send_command({"cmd": "stop"})

        # Reset speeds
        with self._lock:
            self._left_speed = 0.0
            self._right_speed = 0.0
            self._target_left = 0.0
            self._target_right = 0.0

        # Stop the event loop
        if self._loop and self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)

        # Wait for thread
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None

        self._connected = False
        logger.info("Motor controller stopped")

    def set_speeds(self, left: float, right: float) -> None:
        """
        Set target motor speeds.

        Args:
            left: Left motor speed (-1 to 1, negative = backward)
            right: Right motor speed (-1 to 1, negative = backward)
        """
        with self._lock:
            self._target_left = max(-1.0, min(1.0, left))
            self._target_right = max(-1.0, min(1.0, right))
            self._left_speed = self._target_left
            self._right_speed = self._target_right
            self._last_command_time = time.time()
            self._watchdog_triggered = False

        # Send drive command via WebSocket
        self._send_command({
            "cmd": "drive",
            "left": self._target_left,
            "right": self._target_right,
        })

    def set_velocity(self, vx: float, wz: float) -> None:
        """
        Set robot velocity (linear and angular).

        Converts to differential drive wheel speeds.

        Args:
            vx: Linear velocity in m/s (positive = forward)
            wz: Angular velocity in rad/s (positive = counter-clockwise)
        """
        # Convert to wheel speeds (m/s)
        half_track = self._track_width / 2.0
        v_left = vx - wz * half_track
        v_right = vx + wz * half_track

        # Normalize to -1 to 1 range
        max_wheel_speed = 1.0  # m/s
        left = v_left / max_wheel_speed
        right = v_right / max_wheel_speed

        self.set_speeds(left, right)

    def emergency_stop(self) -> None:
        """
        Immediately stop all motors.

        Sends stop command to lawnbot_motors server.
        """
        with self._lock:
            self._target_left = 0.0
            self._target_right = 0.0
            self._left_speed = 0.0
            self._right_speed = 0.0

        self._send_command({"cmd": "stop"})
        logger.warning("EMERGENCY STOP activated")

        if self._emergency_stop_callback:
            self._emergency_stop_callback()

    def get_speeds(self) -> Tuple[float, float]:
        """Get current motor speeds."""
        with self._lock:
            return (self._left_speed, self._right_speed)

    def get_target_speeds(self) -> Tuple[float, float]:
        """Get target motor speeds."""
        with self._lock:
            return (self._target_left, self._target_right)

    @property
    def is_moving(self) -> bool:
        """Check if motors are moving."""
        with self._lock:
            return abs(self._left_speed) > 0.01 or abs(self._right_speed) > 0.01

    @property
    def is_connected(self) -> bool:
        """Check if connected to lawnbot_motors server."""
        return self._connected

    # =========================================================================
    # Internal: WebSocket Communication
    # =========================================================================

    def _run_event_loop(self) -> None:
        """Run the asyncio event loop in a background thread."""
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)

        try:
            self._loop.run_until_complete(self._ws_connection_loop())
        except Exception as e:
            if self._running:
                logger.error(f"Event loop error: {e}")
        finally:
            self._loop.close()

    async def _ws_connection_loop(self) -> None:
        """Maintain WebSocket connection with auto-reconnect."""
        import websockets

        uri = f"ws://{self._config.ws_host}:{self._config.ws_port}"

        while self._running:
            try:
                async with websockets.connect(
                    uri,
                    ping_interval=10,
                    ping_timeout=10,
                    close_timeout=2,
                ) as ws:
                    self._ws = ws
                    self._connected = True
                    logger.info(f"Connected to lawnbot_motors at {uri}")

                    # Read the welcome message
                    try:
                        welcome = await asyncio.wait_for(ws.recv(), timeout=2.0)
                        welcome_data = json.loads(welcome)
                        logger.info(
                            f"lawnbot_motors server: motors={welcome_data.get('motors')}, "
                            f"commands={welcome_data.get('commands')}"
                        )
                    except Exception:
                        pass

                    # Keep connection alive — just wait for it to close
                    try:
                        async for msg in ws:
                            # Process any responses from the server (ACKs)
                            pass
                    except websockets.exceptions.ConnectionClosed:
                        pass

            except (OSError, ConnectionRefusedError) as e:
                if self._running:
                    logger.warning(
                        f"Cannot connect to lawnbot_motors at {uri}: {e}. "
                        f"Retrying in 2s..."
                    )
            except Exception as e:
                if self._running:
                    logger.error(f"WebSocket error: {e}. Retrying in 2s...")
            finally:
                self._ws = None
                self._connected = False

            if self._running:
                await asyncio.sleep(2.0)

    def _send_command(self, command: dict) -> bool:
        """
        Send a JSON command to lawnbot_motors server.

        Args:
            command: Command dict (e.g. {"cmd": "drive", "left": 0.5, "right": 0.5})

        Returns:
            True if sent successfully
        """
        if not self._connected or self._ws is None or self._loop is None:
            return False

        try:
            msg = json.dumps(command)
            future = asyncio.run_coroutine_threadsafe(
                self._ws.send(msg), self._loop
            )
            # Don't block waiting for result — fire and forget for speed
            future.add_done_callback(self._on_send_done)
            return True
        except Exception as e:
            logger.debug(f"Send error: {e}")
            return False

    @staticmethod
    def _on_send_done(future) -> None:
        """Callback for send completion — log errors."""
        try:
            future.result()
        except Exception as e:
            logger.debug(f"WebSocket send failed: {e}")


# =============================================================================
# Velocity Controller
# =============================================================================

class VelocityController:
    """
    High-level velocity controller.

    Wraps MotorController with additional features:
    - Velocity limits
    """

    def __init__(
        self,
        motor_controller: MotorController,
        max_linear_velocity: float = 0.8,   # m/s
        max_angular_velocity: float = 1.5,  # rad/s
    ):
        """
        Initialize velocity controller.

        Args:
            motor_controller: Motor controller instance
            max_linear_velocity: Maximum linear velocity in m/s
            max_angular_velocity: Maximum angular velocity in rad/s
        """
        self._motor = motor_controller
        self._max_vx = max_linear_velocity
        self._max_wz = max_angular_velocity

    def set_velocity(self, vx: float, wz: float) -> None:
        """
        Set robot velocity with limits.

        Args:
            vx: Linear velocity in m/s
            wz: Angular velocity in rad/s
        """
        # Apply limits
        vx = max(-self._max_vx, min(self._max_vx, vx))
        wz = max(-self._max_wz, min(self._max_wz, wz))

        self._motor.set_velocity(vx, wz)

    def stop(self) -> None:
        """Stop the robot."""
        self._motor.set_velocity(0.0, 0.0)

    def emergency_stop(self) -> None:
        """Emergency stop."""
        self._motor.emergency_stop()
