"""
Wheel Encoder Driver and Differential Drive Kinematics.

Story 3.1: GPIO Encoder Reader
Story 3.2: Differential Drive Kinematics

Uses gpiozero for Pi 5 compatibility.
"""

from dataclasses import dataclass
from typing import Tuple, Optional
import threading
import time
import math
import logging

logger = logging.getLogger(__name__)


# =============================================================================
# Configuration
# =============================================================================

@dataclass
class OdometryConfig:
    """Configuration for differential drive odometry."""
    
    # Physical parameters
    track_width: float = 0.24       # Distance between tracks (meters)
    wheel_radius: float = 0.05     # Wheel/sprocket radius (meters)
    
    # Encoder parameters - calibrated from testing: 3232 ticks per revolution
    encoder_cpr: int = 3232         # Counts per revolution
    
    # GPIO pins (BCM numbering) - avoid motor driver pins (5,6,12,13,18,19,23,24)
    left_encoder_a: int = 17
    left_encoder_b: int = 27
    right_encoder_a: int = 22
    right_encoder_b: int = 4
    
    # Computation parameters
    velocity_smoothing: float = 0.1
    min_velocity_threshold: float = 0.01


# =============================================================================
# Encoder Reader (gpiozero)
# =============================================================================

class EncoderReader:
    """
    Quadrature encoder reader using gpiozero (Pi 5 compatible).
    
    Hardware Connection:
        Left Encoder:  GPIO 17 (A), GPIO 27 (B)
        Right Encoder: GPIO 22 (A), GPIO 4  (B)
    """
    
    def __init__(
        self,
        pin_a: int,
        pin_b: int,
        cpr: int = 3232,
    ):
        self._pin_a = pin_a
        self._pin_b = pin_b
        self._cpr = cpr
        
        self._enc_a = None
        self._enc_b = None
        
        self._count = 0
        self._last_count = 0
        self._last_time = time.time()
        self._velocity = 0.0
        self._last_a = 0
        
        self._lock = threading.Lock()
        self._running = False
    
    @property
    def count(self) -> int:
        with self._lock:
            return self._count
    
    @property
    def cpr(self) -> int:
        return self._cpr
    
    def start(self) -> bool:
        """Start encoder reading with gpiozero."""
        try:
            from gpiozero import DigitalInputDevice
            
            self._enc_a = DigitalInputDevice(self._pin_a, pull_up=True)
            self._enc_b = DigitalInputDevice(self._pin_b, pull_up=True)
            
            self._last_a = self._enc_a.value
            
            # Set up callbacks for both edges
            self._enc_a.when_activated = self._on_a_change
            self._enc_a.when_deactivated = self._on_a_change
            
            self._running = True
            self._last_time = time.time()
            logger.info(f"Encoder started on GPIO {self._pin_a}, {self._pin_b}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to start encoder: {e}")
            return False
    
    def stop(self) -> None:
        """Stop encoder reading."""
        self._running = False
        if self._enc_a:
            self._enc_a.close()
        if self._enc_b:
            self._enc_b.close()
    
    def _on_a_change(self) -> None:
        """Callback when encoder A changes."""
        if not self._running:
            return
        
        a = self._enc_a.value
        b = self._enc_b.value
        
        with self._lock:
            if a == self._last_a:
                return
            self._last_a = a
            
            # Direction detection
            if a == b:
                self._count -= 1
            else:
                self._count += 1
    
    def get_count(self) -> int:
        with self._lock:
            return self._count
    
    def get_delta_count(self) -> int:
        with self._lock:
            delta = self._count - self._last_count
            self._last_count = self._count
            return delta
    
    def get_velocity(self, dt: Optional[float] = None) -> float:
        current_time = time.time()
        
        with self._lock:
            delta_count = self._count - self._last_count
            self._last_count = self._count
            
            if dt is None:
                dt = current_time - self._last_time
            self._last_time = current_time
            
            if dt > 0:
                self._velocity = delta_count / dt
            
            return self._velocity
    
    def reset(self) -> None:
        with self._lock:
            self._count = 0
            self._last_count = 0
            self._velocity = 0.0


# =============================================================================
# Differential Drive Kinematics
# =============================================================================

class DifferentialDriveKinematics:
    """Differential drive kinematics for tracked robots."""
    
    def __init__(self, config: OdometryConfig):
        self._config = config
        self._meters_per_count = (
            2.0 * math.pi * config.wheel_radius / config.encoder_cpr
        )
    
    @property
    def config(self) -> OdometryConfig:
        return self._config
    
    def counts_to_distance(self, counts: int) -> float:
        return counts * self._meters_per_count
    
    def counts_to_velocity(self, delta_counts: int, dt: float) -> float:
        if dt <= 0:
            return 0.0
        return self.counts_to_distance(delta_counts) / dt
    
    def wheel_to_robot_velocity(self, v_left: float, v_right: float) -> Tuple[float, float]:
        vx = (v_left + v_right) / 2.0
        wz = (v_right - v_left) / self._config.track_width
        return (vx, wz)
    
    def integrate_pose(
        self, x: float, y: float, theta: float,
        vx: float, wz: float, dt: float
    ) -> Tuple[float, float, float]:
        if abs(wz) < 1e-6:
            x_new = x + vx * math.cos(theta) * dt
            y_new = y + vx * math.sin(theta) * dt
            theta_new = theta
        else:
            radius = vx / wz
            dtheta = wz * dt
            x_new = x + radius * (math.sin(theta + dtheta) - math.sin(theta))
            y_new = y - radius * (math.cos(theta + dtheta) - math.cos(theta))
            theta_new = theta + dtheta
        
        # Normalize theta
        while theta_new > math.pi:
            theta_new -= 2.0 * math.pi
        while theta_new < -math.pi:
            theta_new += 2.0 * math.pi
        
        return (x_new, y_new, theta_new)


# =============================================================================
# Wheel Odometry
# =============================================================================

class WheelOdometry:
    """Complete wheel odometry using two encoders."""
    
    def __init__(
        self, 
        config: OdometryConfig,
        left_encoder: Optional[EncoderReader] = None,
        right_encoder: Optional[EncoderReader] = None,
    ):
        self._config = config
        self._kinematics = DifferentialDriveKinematics(config)
        
        self._left_encoder = left_encoder or EncoderReader(
            pin_a=config.left_encoder_a,
            pin_b=config.left_encoder_b,
            cpr=config.encoder_cpr,
        )
        self._right_encoder = right_encoder or EncoderReader(
            pin_a=config.right_encoder_a,
            pin_b=config.right_encoder_b,
            cpr=config.encoder_cpr,
        )
        
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._vx = 0.0
        self._wz = 0.0
        self._last_update = time.time()
        self._lock = threading.Lock()
    
    def start(self) -> bool:
        left_ok = self._left_encoder.start()
        right_ok = self._right_encoder.start()
        self._last_update = time.time()
        return left_ok and right_ok
    
    def stop(self) -> None:
        self._left_encoder.stop()
        self._right_encoder.stop()
    
    def update(self) -> None:
        current_time = time.time()
        dt = current_time - self._last_update
        self._last_update = current_time
        
        if dt <= 0:
            return
        
        left_delta = self._left_encoder.get_delta_count()
        right_delta = self._right_encoder.get_delta_count()
        
        v_left = self._kinematics.counts_to_velocity(left_delta, dt)
        v_right = self._kinematics.counts_to_velocity(right_delta, dt)
        
        vx, wz = self._kinematics.wheel_to_robot_velocity(v_left, v_right)
        
        with self._lock:
            alpha = self._config.velocity_smoothing
            self._vx = alpha * vx + (1 - alpha) * self._vx
            self._wz = alpha * wz + (1 - alpha) * self._wz
            
            if abs(self._vx) < self._config.min_velocity_threshold:
                self._vx = 0.0
            if abs(self._wz) < self._config.min_velocity_threshold:
                self._wz = 0.0
            
            self._x, self._y, self._theta = self._kinematics.integrate_pose(
                self._x, self._y, self._theta, vx, wz, dt
            )
    
    def get_pose(self) -> Tuple[float, float, float]:
        with self._lock:
            return (self._x, self._y, self._theta)
    
    def get_velocity(self) -> Tuple[float, float]:
        with self._lock:
            return (self._vx, self._wz)
    
    def reset_pose(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0) -> None:
        with self._lock:
            self._x = x
            self._y = y
            self._theta = theta
            self._vx = 0.0
            self._wz = 0.0
        
        self._left_encoder.reset()
        self._right_encoder.reset()
