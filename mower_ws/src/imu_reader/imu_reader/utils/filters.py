"""
Orientation Filters for IMU Data.

Story 1.4: Madgwick Filter for MPU-6050

Provides software orientation estimation for IMUs without built-in fusion.
The Madgwick filter computes orientation from accelerometer and gyroscope data.

Note: For this project with BNO085, we primarily use built-in fusion.
This filter is for MPU-6050 fallback.
"""

import math
from dataclasses import dataclass
from typing import Tuple, Optional


@dataclass
class Quaternion:
    """Quaternion representation (w, x, y, z)."""
    w: float = 1.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    
    def normalize(self) -> 'Quaternion':
        """Return normalized quaternion."""
        norm = math.sqrt(self.w**2 + self.x**2 + self.y**2 + self.z**2)
        if norm > 0:
            return Quaternion(self.w/norm, self.x/norm, self.y/norm, self.z/norm)
        return Quaternion()
    
    def to_tuple(self) -> Tuple[float, float, float, float]:
        """Convert to tuple (w, x, y, z)."""
        return (self.w, self.x, self.y, self.z)
    
    def to_euler(self) -> Tuple[float, float, float]:
        """Convert to Euler angles (roll, pitch, yaw) in radians."""
        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (self.w * self.x + self.y * self.z)
        cosr_cosp = 1.0 - 2.0 * (self.x * self.x + self.y * self.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2.0 * (self.w * self.y - self.z * self.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (self.w * self.z + self.x * self.y)
        cosy_cosp = 1.0 - 2.0 * (self.y * self.y + self.z * self.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return (roll, pitch, yaw)


class MadgwickFilter:
    """
    Madgwick orientation filter for 6-DOF IMU (accelerometer + gyroscope).
    
    This filter estimates orientation from IMU data using gradient descent
    optimization. It provides good orientation estimates without a magnetometer,
    but yaw will drift over time.
    
    For this project:
    - Roll and pitch are reliable from this filter
    - Yaw comes from BNO085 magnetometer (production) or drifts (prototype)
    """
    
    def __init__(
        self,
        beta: float = 0.1,
        sample_period: float = 0.01,
        initial_quaternion: Optional[Quaternion] = None
    ):
        """
        Initialize Madgwick filter.
        
        Args:
            beta: Filter gain (higher = faster convergence, more noise)
            sample_period: Expected time between updates in seconds
            initial_quaternion: Starting orientation (default: identity)
        """
        self._beta = beta
        self._sample_period = sample_period
        self._q = initial_quaternion or Quaternion()
        self._q = self._q.normalize()
    
    @property
    def quaternion(self) -> Quaternion:
        """Get current orientation as quaternion."""
        return self._q
    
    @property
    def beta(self) -> float:
        """Get filter gain."""
        return self._beta
    
    @beta.setter
    def beta(self, value: float) -> None:
        """Set filter gain."""
        self._beta = max(0.0, min(1.0, value))
    
    def update(
        self,
        gyro: Tuple[float, float, float],
        accel: Tuple[float, float, float],
        dt: Optional[float] = None
    ) -> Quaternion:
        """
        Update orientation estimate with new IMU data.
        
        Args:
            gyro: Angular velocity (gx, gy, gz) in rad/s
            accel: Linear acceleration (ax, ay, az) in m/s²
            dt: Time since last update in seconds
            
        Returns:
            Updated quaternion orientation.
        """
        if dt is None:
            dt = self._sample_period
        
        gx, gy, gz = gyro
        ax, ay, az = accel
        
        q0, q1, q2, q3 = self._q.w, self._q.x, self._q.y, self._q.z
        
        # Normalize accelerometer measurement
        norm = math.sqrt(ax*ax + ay*ay + az*az)
        if norm < 1e-10:
            return self._integrate_gyro(gyro, dt)
        
        ax, ay, az = ax/norm, ay/norm, az/norm
        
        # Auxiliary variables
        _2q0, _2q1, _2q2, _2q3 = 2.0*q0, 2.0*q1, 2.0*q2, 2.0*q3
        _4q0, _4q1, _4q2 = 4.0*q0, 4.0*q1, 4.0*q2
        _8q1, _8q2 = 8.0*q1, 8.0*q2
        q0q0, q1q1, q2q2, q3q3 = q0*q0, q1*q1, q2*q2, q3*q3
        
        # Gradient descent corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az
        s2 = 4.0 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az
        s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay
        
        # Normalize step magnitude
        norm = math.sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3)
        if norm > 1e-10:
            s0, s1, s2, s3 = s0/norm, s1/norm, s2/norm, s3/norm
        
        # Compute rate of change
        qDot0 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz) - self._beta * s0
        qDot1 = 0.5 * (q0 * gx + q2 * gz - q3 * gy) - self._beta * s1
        qDot2 = 0.5 * (q0 * gy - q1 * gz + q3 * gx) - self._beta * s2
        qDot3 = 0.5 * (q0 * gz + q1 * gy - q2 * gx) - self._beta * s3
        
        # Integrate
        q0 += qDot0 * dt
        q1 += qDot1 * dt
        q2 += qDot2 * dt
        q3 += qDot3 * dt
        
        self._q = Quaternion(q0, q1, q2, q3).normalize()
        return self._q
    
    def _integrate_gyro(self, gyro: Tuple[float, float, float], dt: float) -> Quaternion:
        """Integrate gyroscope only (when accelerometer is unavailable)."""
        gx, gy, gz = gyro
        q0, q1, q2, q3 = self._q.w, self._q.x, self._q.y, self._q.z
        
        qDot0 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
        qDot1 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
        qDot2 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
        qDot3 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)
        
        q0 += qDot0 * dt
        q1 += qDot1 * dt
        q2 += qDot2 * dt
        q3 += qDot3 * dt
        
        self._q = Quaternion(q0, q1, q2, q3).normalize()
        return self._q
    
    def get_quaternion(self) -> Tuple[float, float, float, float]:
        """Get current orientation as quaternion tuple (w, x, y, z)."""
        return self._q.to_tuple()
    
    def get_euler(self) -> Tuple[float, float, float]:
        """Get current orientation as Euler angles (roll, pitch, yaw)."""
        return self._q.to_euler()
    
    def get_roll_pitch(self) -> Tuple[float, float]:
        """Get roll and pitch only."""
        roll, pitch, _ = self._q.to_euler()
        return (roll, pitch)
    
    def get_quaternion_roll_pitch_only(self, external_yaw: float = 0.0) -> Tuple[float, float, float, float]:
        """Get quaternion with roll/pitch from filter and yaw from external source."""
        roll, pitch, _ = self._q.to_euler()
        
        cy = math.cos(external_yaw * 0.5)
        sy = math.sin(external_yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return (w, x, y, z)
    
    def reset(self, quaternion: Optional[Quaternion] = None) -> None:
        """Reset filter to initial state."""
        self._q = quaternion or Quaternion()
        self._q = self._q.normalize()


class ComplementaryFilter:
    """
    Simple complementary filter for orientation estimation.
    
    Combines accelerometer (long-term stable) with gyroscope (short-term accurate).
    Simpler than Madgwick but less accurate.
    """
    
    def __init__(self, alpha: float = 0.98):
        """
        Initialize complementary filter.
        
        Args:
            alpha: Weight for gyroscope (0-1). Higher = trust gyro more.
        """
        self._alpha = alpha
        self._roll = 0.0
        self._pitch = 0.0
    
    def update(
        self,
        gyro: Tuple[float, float, float],
        accel: Tuple[float, float, float],
        dt: float
    ) -> Tuple[float, float]:
        """Update orientation estimate."""
        gx, gy, gz = gyro
        ax, ay, az = accel
        
        # Calculate roll and pitch from accelerometer
        accel_roll = math.atan2(ay, az)
        accel_pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))
        
        # Integrate gyroscope
        gyro_roll = self._roll + gx * dt
        gyro_pitch = self._pitch + gy * dt
        
        # Complementary filter
        self._roll = self._alpha * gyro_roll + (1 - self._alpha) * accel_roll
        self._pitch = self._alpha * gyro_pitch + (1 - self._alpha) * accel_pitch
        
        return (self._roll, self._pitch)
    
    def get_roll_pitch(self) -> Tuple[float, float]:
        """Get current roll and pitch in radians."""
        return (self._roll, self._pitch)
    
    def reset(self) -> None:
        """Reset filter to zero orientation."""
        self._roll = 0.0
        self._pitch = 0.0
