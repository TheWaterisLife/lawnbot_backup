"""
Coverage Path Planning Module.

Story 2.1: Simple Coverage Planner
Story 2.2: Coverage Optimization

Implements Boustrophedon (back-and-forth) coverage pattern
for efficient lawn mowing.
"""

from dataclasses import dataclass
from typing import List, Tuple, Optional
import math
import logging

from .boundary import Boundary, LocalPoint

logger = logging.getLogger(__name__)


# =============================================================================
# Data Classes
# =============================================================================

@dataclass
class Waypoint:
    """A waypoint in the coverage path."""
    x: float  # meters
    y: float  # meters
    heading: float  # radians (0 = East, pi/2 = North)
    
    def to_tuple(self) -> Tuple[float, float, float]:
        """Convert to tuple (x, y, heading)."""
        return (self.x, self.y, self.heading)


@dataclass
class CoverageConfig:
    """Configuration for coverage planning."""
    
    # Mowing parameters
    cutting_width: float = 0.30      # meters (cutting deck width)
    overlap_ratio: float = 0.10      # 10% overlap between passes
    
    # Robot parameters
    turn_radius: float = 0.30        # meters (minimum turn radius)
    
    # Safety margins
    boundary_margin: float = 0.15    # meters from boundary edge
    
    # Planning parameters
    angle_deg: float = 0.0           # Mowing angle (0 = East-West)
    start_corner: str = 'sw'         # Starting corner: 'sw', 'se', 'nw', 'ne'
    
    @property
    def effective_width(self) -> float:
        """Effective width per pass (accounting for overlap)."""
        return self.cutting_width * (1.0 - self.overlap_ratio)
    
    @property
    def angle_rad(self) -> float:
        """Mowing angle in radians."""
        return math.radians(self.angle_deg)


# =============================================================================
# Coverage Planner
# =============================================================================

class CoveragePlanner:
    """
    Boustrophedon coverage path planner.
    
    Generates a back-and-forth mowing pattern that covers the entire
    lawn area efficiently while staying within boundaries.
    
    Usage:
        config = CoverageConfig(cutting_width=0.30, overlap_ratio=0.10)
        planner = CoveragePlanner(config)
        
        path = planner.generate_path(boundary)
        for waypoint in path:
            print(f"Go to ({waypoint.x:.2f}, {waypoint.y:.2f})")
    """
    
    def __init__(self, config: Optional[CoverageConfig] = None):
        """
        Initialize coverage planner.
        
        Args:
            config: Coverage configuration (uses defaults if None)
        """
        self._config = config or CoverageConfig()
    
    @property
    def config(self) -> CoverageConfig:
        """Get configuration."""
        return self._config
    
    def generate_path(self, boundary: Boundary) -> List[Waypoint]:
        """
        Generate coverage path for a boundary.
        
        Args:
            boundary: Lawn boundary
            
        Returns:
            List of waypoints forming the coverage path
        """
        if not boundary.is_valid or not boundary.local_points:
            logger.error("Invalid boundary for path generation")
            return []
        
        # Get bounding box
        points = boundary.local_points
        min_x = min(p.x for p in points)
        max_x = max(p.x for p in points)
        min_y = min(p.y for p in points)
        max_y = max(p.y for p in points)
        
        # Apply boundary margin
        margin = self._config.boundary_margin
        min_x += margin
        max_x -= margin
        min_y += margin
        max_y -= margin
        
        if max_x <= min_x or max_y <= min_y:
            logger.warning("Boundary too small after margins")
            return []
        
        # Generate basic boustrophedon pattern
        path = self._generate_boustrophedon(
            min_x, max_x, min_y, max_y,
            boundary.local_points
        )
        
        logger.info(
            f"Generated coverage path: {len(path)} waypoints, "
            f"effective width: {self._config.effective_width:.3f}m"
        )
        
        return path
    
    def _generate_boustrophedon(
        self,
        min_x: float,
        max_x: float,
        min_y: float,
        max_y: float,
        boundary_points: List[LocalPoint],
    ) -> List[Waypoint]:
        """
        Generate back-and-forth mowing pattern.
        
        The pattern depends on the mowing angle:
        - 0° (East-West): Rows parallel to X axis
        - 90° (North-South): Rows parallel to Y axis
        """
        path = []
        effective_width = self._config.effective_width
        angle = self._config.angle_rad
        
        # For now, implement East-West pattern (angle = 0)
        # TODO: Support arbitrary angles with coordinate rotation
        
        if abs(angle) < 0.01:  # ~0 degrees - horizontal passes
            path = self._generate_horizontal_passes(
                min_x, max_x, min_y, max_y, boundary_points
            )
        elif abs(angle - math.pi / 2) < 0.01:  # ~90 degrees - vertical passes
            path = self._generate_vertical_passes(
                min_x, max_x, min_y, max_y, boundary_points
            )
        else:
            # General case - rotate, generate, rotate back
            # For simplicity, default to horizontal
            logger.warning(f"Angle {math.degrees(angle):.0f}° not optimized, using horizontal")
            path = self._generate_horizontal_passes(
                min_x, max_x, min_y, max_y, boundary_points
            )
        
        return path
    
    def _generate_horizontal_passes(
        self,
        min_x: float,
        max_x: float,
        min_y: float,
        max_y: float,
        boundary_points: List[LocalPoint],
    ) -> List[Waypoint]:
        """Generate horizontal (East-West) mowing passes."""
        path = []
        effective_width = self._config.effective_width
        
        # Start from selected corner
        start_y = min_y if 's' in self._config.start_corner else max_y
        start_left = 'w' in self._config.start_corner
        direction_y = 1 if 's' in self._config.start_corner else -1
        
        y = start_y
        going_right = not start_left
        
        pass_count = 0
        while (direction_y > 0 and y <= max_y) or (direction_y < 0 and y >= min_y):
            # Find valid X range at this Y level (respecting boundary)
            x_start, x_end = self._find_x_range_at_y(
                y, min_x, max_x, boundary_points
            )
            
            if x_start is not None and x_end is not None:
                if going_right:
                    # Left to right
                    heading = 0.0  # East
                    path.append(Waypoint(x_start, y, heading))
                    path.append(Waypoint(x_end, y, heading))
                else:
                    # Right to left
                    heading = math.pi  # West
                    path.append(Waypoint(x_end, y, heading))
                    path.append(Waypoint(x_start, y, heading))
                
                going_right = not going_right
                pass_count += 1
            
            y += direction_y * effective_width
        
        logger.debug(f"Generated {pass_count} horizontal passes")
        return path
    
    def _generate_vertical_passes(
        self,
        min_x: float,
        max_x: float,
        min_y: float,
        max_y: float,
        boundary_points: List[LocalPoint],
    ) -> List[Waypoint]:
        """Generate vertical (North-South) mowing passes."""
        path = []
        effective_width = self._config.effective_width
        
        # Start from selected corner
        start_x = min_x if 'w' in self._config.start_corner else max_x
        start_bottom = 's' in self._config.start_corner
        direction_x = 1 if 'w' in self._config.start_corner else -1
        
        x = start_x
        going_up = start_bottom
        
        pass_count = 0
        while (direction_x > 0 and x <= max_x) or (direction_x < 0 and x >= min_x):
            # Find valid Y range at this X level
            y_start, y_end = self._find_y_range_at_x(
                x, min_y, max_y, boundary_points
            )
            
            if y_start is not None and y_end is not None:
                if going_up:
                    # Bottom to top
                    heading = math.pi / 2  # North
                    path.append(Waypoint(x, y_start, heading))
                    path.append(Waypoint(x, y_end, heading))
                else:
                    # Top to bottom
                    heading = -math.pi / 2  # South
                    path.append(Waypoint(x, y_end, heading))
                    path.append(Waypoint(x, y_start, heading))
                
                going_up = not going_up
                pass_count += 1
            
            x += direction_x * effective_width
        
        logger.debug(f"Generated {pass_count} vertical passes")
        return path
    
    def _find_x_range_at_y(
        self,
        y: float,
        min_x: float,
        max_x: float,
        boundary_points: List[LocalPoint],
    ) -> Tuple[Optional[float], Optional[float]]:
        """Find valid X range at given Y level within boundary."""
        # Simple approach: use bounding box
        # TODO: Proper polygon intersection for complex shapes
        return (min_x, max_x)
    
    def _find_y_range_at_x(
        self,
        x: float,
        min_y: float,
        max_y: float,
        boundary_points: List[LocalPoint],
    ) -> Tuple[Optional[float], Optional[float]]:
        """Find valid Y range at given X level within boundary."""
        # Simple approach: use bounding box
        # TODO: Proper polygon intersection for complex shapes
        return (min_y, max_y)
    
    def estimate_coverage_time(
        self,
        boundary: Boundary,
        mowing_speed: float = 0.5,  # m/s
        turn_time: float = 3.0,     # seconds per turn
    ) -> float:
        """
        Estimate total mowing time.
        
        Args:
            boundary: Lawn boundary
            mowing_speed: Forward speed in m/s
            turn_time: Time for each turn in seconds
            
        Returns:
            Estimated time in seconds
        """
        if not boundary.is_valid:
            return 0.0
        
        # Calculate number of passes
        points = boundary.local_points
        min_y = min(p.y for p in points)
        max_y = max(p.y for p in points)
        height = max_y - min_y - 2 * self._config.boundary_margin
        
        num_passes = max(1, int(height / self._config.effective_width))
        
        # Total distance
        min_x = min(p.x for p in points)
        max_x = max(p.x for p in points)
        width = max_x - min_x - 2 * self._config.boundary_margin
        
        total_distance = num_passes * width
        
        # Time calculation
        mowing_time = total_distance / mowing_speed
        turn_time_total = (num_passes - 1) * turn_time
        
        return mowing_time + turn_time_total
    
    def get_coverage_stats(self, boundary: Boundary) -> dict:
        """Get statistics about coverage plan."""
        if not boundary.is_valid:
            return {}
        
        points = boundary.local_points
        min_x = min(p.x for p in points) + self._config.boundary_margin
        max_x = max(p.x for p in points) - self._config.boundary_margin
        min_y = min(p.y for p in points) + self._config.boundary_margin
        max_y = max(p.y for p in points) - self._config.boundary_margin
        
        width = max_x - min_x
        height = max_y - min_y
        
        num_passes = max(1, int(height / self._config.effective_width))
        total_distance = num_passes * width
        
        return {
            'area_sq_meters': boundary.area_sq_meters,
            'width_meters': width,
            'height_meters': height,
            'effective_pass_width': self._config.effective_width,
            'num_passes': num_passes,
            'total_distance_meters': total_distance,
            'estimated_time_seconds': self.estimate_coverage_time(boundary),
            'cutting_width': self._config.cutting_width,
            'overlap_ratio': self._config.overlap_ratio,
        }
