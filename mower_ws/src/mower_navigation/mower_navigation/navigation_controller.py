"""
Navigation Controller Module.

Story 3.7: Path Following Launch

Coordinates between coverage planner and Nav2:
- Receives coverage path from planner
- Sends waypoints to Nav2 for execution
- Tracks progress and handles failures
"""

from dataclasses import dataclass
from typing import List, Optional, Callable
from enum import Enum
import logging
import math
import time

logger = logging.getLogger(__name__)


# =============================================================================
# Data Classes and Enums
# =============================================================================

class NavigationState(Enum):
    """Navigation state machine states."""
    IDLE = "idle"
    READY = "ready"          # Path loaded, waiting to start
    NAVIGATING = "navigating"  # Following waypoints
    PAUSED = "paused"
    RECOVERING = "recovering"
    ERROR = "error"
    COMPLETE = "complete"


@dataclass
class NavigationProgress:
    """Navigation progress information."""
    current_waypoint: int = 0
    total_waypoints: int = 0
    distance_remaining: float = 0.0
    estimated_time_remaining: float = 0.0
    percent_complete: float = 0.0


@dataclass
class Waypoint:
    """A navigation waypoint."""
    x: float
    y: float
    heading: float  # radians
    
    def distance_to(self, other: 'Waypoint') -> float:
        """Calculate distance to another waypoint."""
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)


# =============================================================================
# Navigation Controller
# =============================================================================

class NavigationController:
    """
    Controls autonomous navigation using Nav2.
    
    Manages the navigation state machine and interfaces with Nav2
    to follow coverage paths.
    
    Usage:
        controller = NavigationController()
        controller.set_path(waypoints)
        controller.start_navigation()
        
        # In your loop:
        progress = controller.get_progress()
        if controller.state == NavigationState.COMPLETE:
            print("Mowing finished!")
    """
    
    def __init__(
        self,
        goal_tolerance: float = 0.15,
        heading_tolerance: float = 0.25,
        navigation_speed: float = 0.35,
    ):
        """
        Initialize navigation controller.
        
        Args:
            goal_tolerance: Position tolerance in meters
            heading_tolerance: Heading tolerance in radians
            navigation_speed: Default navigation speed m/s
        """
        self._goal_tolerance = goal_tolerance
        self._heading_tolerance = heading_tolerance
        self._navigation_speed = navigation_speed
        
        # State
        self._state = NavigationState.IDLE
        self._waypoints: List[Waypoint] = []
        self._current_index = 0
        self._start_time = 0.0
        
        # Current pose (set by external source)
        self._current_x = 0.0
        self._current_y = 0.0
        self._current_heading = 0.0
        
        # Callbacks
        self._goal_callback: Optional[Callable[[Waypoint], bool]] = None
        self._cancel_callback: Optional[Callable[[], None]] = None
        self._state_callback: Optional[Callable[[NavigationState], None]] = None
        
        # Recovery
        self._recovery_attempts = 0
        self._max_recovery_attempts = 3
        self._last_progress_time = 0.0
        self._progress_timeout = 30.0  # seconds
    
    @property
    def state(self) -> NavigationState:
        """Get current navigation state."""
        return self._state
    
    @property
    def waypoints(self) -> List[Waypoint]:
        """Get current waypoint list."""
        return self._waypoints.copy()
    
    @property
    def current_waypoint(self) -> Optional[Waypoint]:
        """Get current target waypoint."""
        if 0 <= self._current_index < len(self._waypoints):
            return self._waypoints[self._current_index]
        return None
    
    def set_goal_callback(self, callback: Callable[[Waypoint], bool]) -> None:
        """Set callback to send goal to Nav2. Returns True if accepted."""
        self._goal_callback = callback
    
    def set_cancel_callback(self, callback: Callable[[], None]) -> None:
        """Set callback to cancel current Nav2 goal."""
        self._cancel_callback = callback
    
    def set_state_callback(self, callback: Callable[[NavigationState], None]) -> None:
        """Set callback for state changes."""
        self._state_callback = callback
    
    def _set_state(self, new_state: NavigationState) -> None:
        """Update state and notify callback."""
        if new_state != self._state:
            old_state = self._state
            self._state = new_state
            logger.info(f"Navigation state: {old_state.value} -> {new_state.value}")
            if self._state_callback:
                self._state_callback(new_state)
    
    def update_pose(self, x: float, y: float, heading: float) -> None:
        """Update current robot pose."""
        self._current_x = x
        self._current_y = y
        self._current_heading = heading
    
    def set_path(self, waypoints: List[Waypoint]) -> bool:
        """
        Set the navigation path.
        
        Args:
            waypoints: List of waypoints to follow
            
        Returns:
            True if path accepted
        """
        if self._state == NavigationState.NAVIGATING:
            logger.warning("Cannot set path while navigating - pause first")
            return False
        
        if not waypoints:
            logger.warning("Empty waypoint list")
            return False
        
        self._waypoints = waypoints
        self._current_index = 0
        self._set_state(NavigationState.READY)
        logger.info(f"Path set with {len(waypoints)} waypoints")
        return True
    
    def start_navigation(self) -> bool:
        """
        Start navigating the path.
        
        Returns:
            True if navigation started
        """
        if self._state not in (NavigationState.READY, NavigationState.PAUSED):
            logger.warning(f"Cannot start from state: {self._state.value}")
            return False
        
        if not self._waypoints:
            logger.warning("No path loaded")
            return False
        
        self._start_time = time.time()
        self._last_progress_time = time.time()
        self._recovery_attempts = 0
        self._set_state(NavigationState.NAVIGATING)
        
        # Send first waypoint
        return self._send_current_goal()
    
    def pause_navigation(self) -> None:
        """Pause navigation (can be resumed)."""
        if self._state == NavigationState.NAVIGATING:
            if self._cancel_callback:
                self._cancel_callback()
            self._set_state(NavigationState.PAUSED)
    
    def stop_navigation(self) -> None:
        """Stop navigation and clear path."""
        if self._cancel_callback:
            self._cancel_callback()
        self._waypoints = []
        self._current_index = 0
        self._set_state(NavigationState.IDLE)
    
    def _send_current_goal(self) -> bool:
        """Send current waypoint to Nav2."""
        if self._current_index >= len(self._waypoints):
            self._set_state(NavigationState.COMPLETE)
            logger.info("Navigation complete!")
            return True
        
        waypoint = self._waypoints[self._current_index]
        
        if self._goal_callback:
            success = self._goal_callback(waypoint)
            if not success:
                logger.warning(f"Goal rejected for waypoint {self._current_index}")
                return False
        
        logger.info(f"Navigating to waypoint {self._current_index + 1}/{len(self._waypoints)}")
        return True
    
    def on_goal_reached(self) -> None:
        """Called when Nav2 reports goal reached."""
        if self._state != NavigationState.NAVIGATING:
            return
        
        self._current_index += 1
        self._last_progress_time = time.time()
        self._recovery_attempts = 0
        
        if self._current_index >= len(self._waypoints):
            self._set_state(NavigationState.COMPLETE)
            logger.info("All waypoints reached - navigation complete!")
        else:
            self._send_current_goal()
    
    def on_goal_failed(self, reason: str = "") -> None:
        """Called when Nav2 reports goal failure."""
        if self._state != NavigationState.NAVIGATING:
            return
        
        logger.warning(f"Goal failed: {reason}")
        self._recovery_attempts += 1
        
        if self._recovery_attempts >= self._max_recovery_attempts:
            logger.error("Max recovery attempts reached - stopping")
            self._set_state(NavigationState.ERROR)
        else:
            logger.info(f"Recovery attempt {self._recovery_attempts}/{self._max_recovery_attempts}")
            self._set_state(NavigationState.RECOVERING)
            # Retry current goal
            self._set_state(NavigationState.NAVIGATING)
            self._send_current_goal()
    
    def skip_waypoint(self) -> bool:
        """Skip current waypoint and move to next."""
        if self._state not in (NavigationState.NAVIGATING, NavigationState.RECOVERING, NavigationState.ERROR):
            return False
        
        self._current_index += 1
        self._recovery_attempts = 0
        
        if self._current_index >= len(self._waypoints):
            self._set_state(NavigationState.COMPLETE)
        else:
            self._set_state(NavigationState.NAVIGATING)
            self._send_current_goal()
        
        return True
    
    def get_progress(self) -> NavigationProgress:
        """Get navigation progress."""
        if not self._waypoints:
            return NavigationProgress()
        
        total = len(self._waypoints)
        current = min(self._current_index, total)
        
        # Calculate remaining distance
        remaining_dist = 0.0
        if current < total:
            # Distance to current target
            wp = self._waypoints[current]
            remaining_dist = math.sqrt(
                (wp.x - self._current_x)**2 + 
                (wp.y - self._current_y)**2
            )
            # Plus distance between remaining waypoints
            for i in range(current, total - 1):
                remaining_dist += self._waypoints[i].distance_to(self._waypoints[i + 1])
        
        # Estimate time
        est_time = remaining_dist / self._navigation_speed if self._navigation_speed > 0 else 0
        
        # Percent complete
        if total > 0:
            percent = (current / total) * 100.0
        else:
            percent = 0.0
        
        return NavigationProgress(
            current_waypoint=current,
            total_waypoints=total,
            distance_remaining=remaining_dist,
            estimated_time_remaining=est_time,
            percent_complete=percent,
        )
    
    def check_progress_timeout(self) -> bool:
        """
        Check if we've made progress recently.
        
        Returns:
            True if timed out (stuck)
        """
        if self._state != NavigationState.NAVIGATING:
            return False
        
        elapsed = time.time() - self._last_progress_time
        return elapsed > self._progress_timeout
