"""
Obstacle Handler Module.

Story 3.1: Obstacle Detection Integration
Story 3.2: Obstacle Avoidance

Integrates with camera vision for obstacle detection and avoidance.
"""

from dataclasses import dataclass
from typing import List, Optional, Callable, Tuple
from enum import Enum
import math
import logging
import time

logger = logging.getLogger(__name__)


# =============================================================================
# Enums and Data Classes
# =============================================================================

class ObstacleType(Enum):
    """Types of obstacles detected."""
    UNKNOWN = "unknown"
    PERSON = "person"
    ANIMAL = "animal"
    VEHICLE = "vehicle"
    OBJECT = "object"
    BOUNDARY = "boundary"


class AvoidanceAction(Enum):
    """Actions to take for obstacle avoidance."""
    NONE = "none"
    SLOW_DOWN = "slow_down"
    STOP = "stop"
    TURN_LEFT = "turn_left"      # Gentle turn while moving
    TURN_RIGHT = "turn_right"    # Gentle turn while moving
    PIVOT_LEFT = "pivot_left"    # Tank turn in place (left wheel back, right forward)
    PIVOT_RIGHT = "pivot_right"  # Tank turn in place (right wheel back, left forward)
    PIVOT_180 = "pivot_180"      # Full 180° turn in place


@dataclass
class Obstacle:
    """Detected obstacle information."""
    type: ObstacleType
    distance: float          # meters
    angle: float             # radians from center (0 = straight ahead)
    width: float = 0.3       # estimated width in meters
    confidence: float = 0.5  # detection confidence (0-1)
    timestamp: float = 0.0   # detection time
    
    @property
    def is_in_path(self) -> bool:
        """Check if obstacle is in robot's path."""
        # Robot width + margin
        robot_half_width = 0.20  # meters
        
        # Calculate lateral distance at obstacle distance
        lateral_distance = abs(self.distance * math.sin(self.angle))
        
        return lateral_distance < (robot_half_width + self.width / 2)
    
    @property
    def time_to_collision(self) -> float:
        """Estimate time to collision at current speed."""
        # This would need current velocity - placeholder
        return self.distance / 0.5  # Assume 0.5 m/s


@dataclass
class SafetyZone:
    """Safety zone configuration."""
    critical_distance: float = 0.3   # meters - emergency stop
    warning_distance: float = 0.8    # meters - slow down
    detection_distance: float = 2.0  # meters - track obstacles


# =============================================================================
# Obstacle Handler
# =============================================================================

class ObstacleHandler:
    """
    Handles obstacle detection and avoidance.
    
    Integrates with camera vision system to:
    - Track detected obstacles
    - Determine avoidance actions
    - Trigger safety responses
    
    Usage:
        handler = ObstacleHandler()
        handler.set_stop_callback(motor_controller.emergency_stop)
        
        # Update with camera detections (called by camera node)
        handler.update_obstacles([
            Obstacle(ObstacleType.PERSON, distance=1.5, angle=0.1),
        ])
        
        # Get avoidance action
        action = handler.get_avoidance_action()
        if action == AvoidanceAction.STOP:
            motor_controller.emergency_stop()
    """
    
    def __init__(
        self,
        safety_zone: Optional[SafetyZone] = None,
    ):
        """
        Initialize obstacle handler.
        
        Args:
            safety_zone: Safety zone configuration
        """
        self._safety_zone = safety_zone or SafetyZone()
        
        # Current obstacles
        self._obstacles: List[Obstacle] = []
        self._last_update = 0.0
        
        # Callbacks
        self._stop_callback: Optional[Callable[[], None]] = None
        self._slow_callback: Optional[Callable[[float], None]] = None
        
        # State
        self._emergency_active = False
        self._max_age = 1.0  # seconds - discard old detections
        
        # Recovery state (new)
        self._stop_start_time: float = 0.0
        self._stop_timeout: float = 2.0  # seconds before pivot recovery
        self._in_recovery: bool = False
        
        # Boundary constraints (set by external caller)
        self._left_boundary_blocked: bool = False
        self._right_boundary_blocked: bool = False
    
    def set_stop_callback(self, callback: Callable[[], None]) -> None:
        """Set callback for emergency stop."""
        self._stop_callback = callback
    
    def set_slow_callback(self, callback: Callable[[float], None]) -> None:
        """Set callback for speed reduction (takes speed factor 0-1)."""
        self._slow_callback = callback
    
    def update_obstacles(self, obstacles: List[Obstacle]) -> None:
        """
        Update obstacle list from camera detections.
        
        Args:
            obstacles: List of detected obstacles
        """
        current_time = time.time()
        
        # Update timestamps and store
        for obs in obstacles:
            obs.timestamp = current_time
        
        self._obstacles = obstacles
        self._last_update = current_time
        
        # Check for emergency conditions
        self._check_safety()
    
    def add_obstacle(self, obstacle: Obstacle) -> None:
        """Add a single obstacle."""
        obstacle.timestamp = time.time()
        self._obstacles.append(obstacle)
        self._check_safety()
    
    def clear_obstacles(self) -> None:
        """Clear all obstacles."""
        self._obstacles = []
        self._emergency_active = False
    
    def get_obstacles(self) -> List[Obstacle]:
        """Get current obstacle list."""
        self._remove_stale_obstacles()
        return self._obstacles.copy()
    
    def get_closest_obstacle(self) -> Optional[Obstacle]:
        """Get closest obstacle in path."""
        self._remove_stale_obstacles()
        
        in_path = [o for o in self._obstacles if o.is_in_path]
        if not in_path:
            return None
        
        return min(in_path, key=lambda o: o.distance)
    
    def set_boundary_constraints(self, left_blocked: bool, right_blocked: bool) -> None:
        """Set whether turns are blocked by boundaries.
        
        This should be called by the navigation system based on current
        position relative to boundary edges.
        
        Args:
            left_blocked: True if left boundary is too close to turn left
            right_blocked: True if right boundary is too close to turn right
        """
        self._left_boundary_blocked = left_blocked
        self._right_boundary_blocked = right_blocked
    
    def _choose_pivot_direction(self, obstacle: Optional[Obstacle]) -> AvoidanceAction:
        """Choose pivot direction considering boundary constraints.
        
        Priority:
        1. Boundary constraints (don't turn into boundary)
        2. Obstacle position (turn away from obstacle)
        3. Default to 180° if both blocked
        """
        # Both boundaries blocked - must do 180
        if self._left_boundary_blocked and self._right_boundary_blocked:
            return AvoidanceAction.PIVOT_180
        
        # Left boundary blocked - must go right
        if self._left_boundary_blocked:
            return AvoidanceAction.PIVOT_RIGHT
        
        # Right boundary blocked - must go left
        if self._right_boundary_blocked:
            return AvoidanceAction.PIVOT_LEFT
        
        # No boundary constraint - turn away from obstacle
        if obstacle is not None:
            if obstacle.angle > 0.1:  # Obstacle to the right
                return AvoidanceAction.PIVOT_LEFT
            elif obstacle.angle < -0.1:  # Obstacle to the left
                return AvoidanceAction.PIVOT_RIGHT
            else:  # Dead center
                return AvoidanceAction.PIVOT_180
        
        # No obstacle info - default to left
        return AvoidanceAction.PIVOT_LEFT
    
    def reset_recovery_state(self) -> None:
        """Reset recovery state (call after mower has completed a maneuver)."""
        self._stop_start_time = 0.0
        self._in_recovery = False
    
    def get_avoidance_action(self) -> AvoidanceAction:
        """
        Determine avoidance action based on obstacles.
        
        Includes recovery logic:
        - If stopped for > stop_timeout seconds, enters recovery mode
        - Recovery mode: pivot turn to avoid obstacle
        - Considers boundary constraints when choosing direction
        
        Returns:
            Recommended avoidance action
        """
        self._remove_stale_obstacles()
        
        closest = self.get_closest_obstacle()
        if closest is None:
            # Path clear - reset recovery state
            self._stop_start_time = 0.0
            self._in_recovery = False
            return AvoidanceAction.NONE
        
        # Critical zone - stop or recover
        if closest.distance < self._safety_zone.critical_distance:
            if not self._in_recovery:
                # Start or continue stop timer
                if self._stop_start_time == 0:
                    self._stop_start_time = time.time()
                    logger.info("Obstacle in critical zone - stopping")
                
                # Check if we've waited long enough to recover
                elapsed = time.time() - self._stop_start_time
                if elapsed > self._stop_timeout:
                    self._in_recovery = True
                    self._stop_start_time = 0.0
                    pivot_action = self._choose_pivot_direction(closest)
                    logger.info(f"Stop timeout - entering recovery: {pivot_action.value}")
                    return pivot_action
                
                return AvoidanceAction.STOP
            else:
                # Already in recovery - continue pivoting
                return self._choose_pivot_direction(closest)
        
        # Warning zone - choose turn or slow down
        if closest.distance < self._safety_zone.warning_distance:
            self._stop_start_time = 0.0  # Reset stop timer
            
            # Determine turn direction considering boundaries
            if closest.angle > 0.1:
                # Obstacle to the right - prefer left
                if not self._left_boundary_blocked:
                    return AvoidanceAction.TURN_LEFT
                elif not self._right_boundary_blocked:
                    return AvoidanceAction.TURN_RIGHT
                else:
                    return AvoidanceAction.SLOW_DOWN  # Both blocked
            elif closest.angle < -0.1:
                # Obstacle to the left - prefer right
                if not self._right_boundary_blocked:
                    return AvoidanceAction.TURN_RIGHT
                elif not self._left_boundary_blocked:
                    return AvoidanceAction.TURN_LEFT
                else:
                    return AvoidanceAction.SLOW_DOWN  # Both blocked
            else:
                # Dead center - slow down
                return AvoidanceAction.SLOW_DOWN
        
        # Outside warning zone - clear
        self._in_recovery = False
        return AvoidanceAction.NONE
    
    def get_speed_factor(self) -> float:
        """
        Get speed reduction factor based on obstacles.
        
        Returns:
            Factor to multiply speed by (0-1)
        """
        closest = self.get_closest_obstacle()
        if closest is None:
            return 1.0
        
        if closest.distance < self._safety_zone.critical_distance:
            return 0.0
        
        if closest.distance < self._safety_zone.warning_distance:
            # Linear interpolation
            ratio = (closest.distance - self._safety_zone.critical_distance) / \
                    (self._safety_zone.warning_distance - self._safety_zone.critical_distance)
            return max(0.3, ratio)  # Minimum 30% speed
        
        return 1.0
    
    def is_path_clear(
        self, 
        distance: float = 1.0, 
        width: float = 0.4
    ) -> bool:
        """
        Check if path ahead is clear.
        
        Args:
            distance: Look-ahead distance in meters
            width: Path width in meters
            
        Returns:
            True if path is clear
        """
        self._remove_stale_obstacles()
        
        half_width = width / 2
        
        for obs in self._obstacles:
            if obs.distance > distance:
                continue
            
            # Calculate lateral distance
            lateral = abs(obs.distance * math.sin(obs.angle))
            
            if lateral < half_width + obs.width / 2:
                return False
        
        return True
    
    def _check_safety(self) -> None:
        """Check for emergency conditions and trigger callbacks."""
        closest = self.get_closest_obstacle()
        
        if closest is None:
            self._emergency_active = False
            return
        
        # Critical obstacle - emergency stop
        if closest.distance < self._safety_zone.critical_distance:
            if not self._emergency_active:
                self._emergency_active = True
                logger.warning(
                    f"EMERGENCY: {closest.type.value} at {closest.distance:.2f}m"
                )
                if self._stop_callback:
                    self._stop_callback()
        else:
            self._emergency_active = False
        
        # Warning zone - slow down
        if closest.distance < self._safety_zone.warning_distance:
            factor = self.get_speed_factor()
            if self._slow_callback:
                self._slow_callback(factor)
    
    def _remove_stale_obstacles(self) -> None:
        """Remove obstacles older than max age."""
        current_time = time.time()
        self._obstacles = [
            o for o in self._obstacles 
            if current_time - o.timestamp < self._max_age
        ]


# =============================================================================
# Camera Integration Helper
# =============================================================================

def parse_vision_detections(
    detections: List[dict],
    depth_data: Optional[List[float]] = None,
) -> List[Obstacle]:
    """
    Parse camera vision detections to obstacles.
    
    Args:
        detections: List of detection dicts from camera node
        depth_data: Optional depth values for each detection
        
    Returns:
        List of Obstacle objects
    """
    obstacles = []
    
    # Safety-critical classes
    critical_classes = {
        'person': ObstacleType.PERSON,
        'dog': ObstacleType.ANIMAL,
        'cat': ObstacleType.ANIMAL,
        'bird': ObstacleType.ANIMAL,
        'car': ObstacleType.VEHICLE,
        'bicycle': ObstacleType.VEHICLE,
        'motorcycle': ObstacleType.VEHICLE,
    }
    
    for i, det in enumerate(detections):
        label = det.get('label', 'unknown').lower()
        confidence = det.get('confidence', 0.5)
        
        # Get obstacle type
        obs_type = critical_classes.get(label, ObstacleType.OBJECT)
        
        # Get distance (from depth or bbox)
        if depth_data and i < len(depth_data):
            distance = depth_data[i]
        else:
            # Estimate from bounding box size
            bbox = det.get('bbox', [0, 0, 0, 0])
            bbox_height = bbox[3] - bbox[1] if len(bbox) >= 4 else 0.1
            # Rough estimate: larger bbox = closer
            distance = max(0.5, 5.0 / max(0.1, bbox_height))
        
        # Get angle from bbox center x position
        bbox = det.get('bbox', [0.5, 0, 0.5, 0])
        center_x = (bbox[0] + bbox[2]) / 2 if len(bbox) >= 4 else 0.5
        # Convert to angle (assuming 60 degree FOV)
        angle = (center_x - 0.5) * math.radians(60)
        
        # Estimate width from bbox
        width = (bbox[2] - bbox[0]) * distance if len(bbox) >= 4 else 0.3
        
        obstacles.append(Obstacle(
            type=obs_type,
            distance=distance,
            angle=angle,
            width=width,
            confidence=confidence,
        ))
    
    return obstacles
