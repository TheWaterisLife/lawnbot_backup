"""
Boundary Management Module.

Story 1.1: Boundary Learning Mode
Story 1.2: Boundary Validation  
Story 1.3: Boundary Storage

Handles lawn boundary definition, storage, and checking.
Boundaries are defined by GPS coordinates recorded while
the user drives the mower around the perimeter.
"""

from dataclasses import dataclass, field, asdict
from typing import List, Tuple, Optional, Dict, Any
from pathlib import Path
import json
import math
import logging
from datetime import datetime

logger = logging.getLogger(__name__)


# =============================================================================
# Data Classes
# =============================================================================

@dataclass
class GPSPoint:
    """A GPS coordinate point."""
    latitude: float
    longitude: float
    altitude: float = 0.0
    timestamp: Optional[str] = None
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary."""
        return asdict(self)
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'GPSPoint':
        """Create from dictionary."""
        return cls(
            latitude=data['latitude'],
            longitude=data['longitude'],
            altitude=data.get('altitude', 0.0),
            timestamp=data.get('timestamp'),
        )


@dataclass
class LocalPoint:
    """A point in local ENU coordinates (meters from origin)."""
    x: float  # East
    y: float  # North
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary."""
        return {'x': self.x, 'y': self.y}
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'LocalPoint':
        """Create from dictionary."""
        return cls(x=data['x'], y=data['y'])


@dataclass
class Boundary:
    """
    A lawn boundary definition.
    
    Stores both GPS and local coordinates for flexibility.
    GPS coordinates are the source of truth; local coordinates
    are derived from GPS relative to the origin.
    """
    name: str
    gps_points: List[GPSPoint] = field(default_factory=list)
    local_points: List[LocalPoint] = field(default_factory=list)
    origin: Optional[GPSPoint] = None  # First point becomes origin
    created_at: str = field(default_factory=lambda: datetime.now().isoformat())
    modified_at: str = field(default_factory=lambda: datetime.now().isoformat())
    metadata: Dict[str, Any] = field(default_factory=dict)
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization."""
        return {
            'name': self.name,
            'gps_points': [p.to_dict() for p in self.gps_points],
            'local_points': [p.to_dict() for p in self.local_points],
            'origin': self.origin.to_dict() if self.origin else None,
            'created_at': self.created_at,
            'modified_at': self.modified_at,
            'metadata': self.metadata,
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'Boundary':
        """Create from dictionary."""
        return cls(
            name=data['name'],
            gps_points=[GPSPoint.from_dict(p) for p in data.get('gps_points', [])],
            local_points=[LocalPoint.from_dict(p) for p in data.get('local_points', [])],
            origin=GPSPoint.from_dict(data['origin']) if data.get('origin') else None,
            created_at=data.get('created_at', datetime.now().isoformat()),
            modified_at=data.get('modified_at', datetime.now().isoformat()),
            metadata=data.get('metadata', {}),
        )
    
    @property
    def is_valid(self) -> bool:
        """Check if boundary has enough points to form a polygon."""
        return len(self.gps_points) >= 3
    
    @property
    def point_count(self) -> int:
        """Get number of points in boundary."""
        return len(self.gps_points)
    
    @property
    def area_sq_meters(self) -> float:
        """Calculate area in square meters using shoelace formula."""
        if not self.local_points or len(self.local_points) < 3:
            return 0.0
        
        n = len(self.local_points)
        area = 0.0
        
        for i in range(n):
            j = (i + 1) % n
            area += self.local_points[i].x * self.local_points[j].y
            area -= self.local_points[j].x * self.local_points[i].y
        
        return abs(area) / 2.0
    
    @property
    def perimeter_meters(self) -> float:
        """Calculate perimeter in meters."""
        if not self.local_points or len(self.local_points) < 2:
            return 0.0
        
        perimeter = 0.0
        n = len(self.local_points)
        
        for i in range(n):
            j = (i + 1) % n
            dx = self.local_points[j].x - self.local_points[i].x
            dy = self.local_points[j].y - self.local_points[i].y
            perimeter += math.sqrt(dx * dx + dy * dy)
        
        return perimeter


# =============================================================================
# Boundary Manager
# =============================================================================

class BoundaryManager:
    """
    Manages lawn boundaries - learning, storage, and validation.
    
    Usage:
        manager = BoundaryManager(storage_path="/config/boundaries")
        
        # Learning mode
        manager.start_learning("back_yard")
        manager.add_gps_point(45.5, -73.6, 50.0)
        manager.add_gps_point(45.5001, -73.6, 50.0)
        # ... more points
        manager.finish_learning()
        
        # Save and load
        manager.save_boundary("back_yard")
        manager.load_boundary("back_yard")
        
        # Check if point is inside
        if manager.is_inside("back_yard", x=5.0, y=3.0):
            print("Inside boundary!")
    """
    
    # WGS84 constants for GPS to local conversion
    EARTH_RADIUS = 6371000  # meters
    
    def __init__(self, storage_path: str = "config/boundaries"):
        """
        Initialize boundary manager.
        
        Args:
            storage_path: Directory for storing boundary JSON files
        """
        self._storage_path = Path(storage_path)
        self._storage_path.mkdir(parents=True, exist_ok=True)
        
        # Active boundaries
        self._boundaries: Dict[str, Boundary] = {}
        
        # Learning state
        self._learning_active = False
        self._learning_boundary: Optional[Boundary] = None
    
    @property
    def is_learning(self) -> bool:
        """Check if currently in learning mode."""
        return self._learning_active
    
    @property
    def boundary_names(self) -> List[str]:
        """Get list of loaded boundary names."""
        return list(self._boundaries.keys())
    
    def start_learning(self, name: str) -> bool:
        """
        Start boundary learning mode.
        
        Args:
            name: Name for the new boundary
            
        Returns:
            True if learning started, False if already learning
        """
        if self._learning_active:
            logger.warning("Already in learning mode")
            return False
        
        self._learning_boundary = Boundary(name=name)
        self._learning_active = True
        logger.info(f"Started learning boundary: {name}")
        return True
    
    def add_gps_point(
        self, 
        latitude: float, 
        longitude: float, 
        altitude: float = 0.0
    ) -> bool:
        """
        Add a GPS point during learning mode.
        
        Args:
            latitude: Latitude in degrees
            longitude: Longitude in degrees
            altitude: Altitude in meters
            
        Returns:
            True if point added, False if not in learning mode
        """
        if not self._learning_active or self._learning_boundary is None:
            return False
        
        point = GPSPoint(
            latitude=latitude,
            longitude=longitude,
            altitude=altitude,
            timestamp=datetime.now().isoformat(),
        )
        
        # Set origin as first point
        if self._learning_boundary.origin is None:
            self._learning_boundary.origin = point
        
        self._learning_boundary.gps_points.append(point)
        
        # Convert to local coordinates
        local = self._gps_to_local(point, self._learning_boundary.origin)
        self._learning_boundary.local_points.append(local)
        
        logger.debug(
            f"Added point {len(self._learning_boundary.gps_points)}: "
            f"GPS({latitude:.6f}, {longitude:.6f}) -> Local({local.x:.2f}, {local.y:.2f})"
        )
        
        return True
    
    def finish_learning(self) -> Optional[Boundary]:
        """
        Finish boundary learning and validate.
        
        Returns:
            Completed Boundary if valid, None otherwise
        """
        if not self._learning_active or self._learning_boundary is None:
            return None
        
        boundary = self._learning_boundary
        
        # Validate
        if not boundary.is_valid:
            logger.warning(
                f"Boundary '{boundary.name}' invalid: "
                f"needs at least 3 points, has {boundary.point_count}"
            )
            self._learning_active = False
            self._learning_boundary = None
            return None
        
        # Update metadata
        boundary.modified_at = datetime.now().isoformat()
        boundary.metadata['area_sq_meters'] = boundary.area_sq_meters
        boundary.metadata['perimeter_meters'] = boundary.perimeter_meters
        
        # Store
        self._boundaries[boundary.name] = boundary
        
        logger.info(
            f"Finished learning boundary '{boundary.name}': "
            f"{boundary.point_count} points, "
            f"{boundary.area_sq_meters:.1f} m², "
            f"{boundary.perimeter_meters:.1f} m perimeter"
        )
        
        self._learning_active = False
        self._learning_boundary = None
        
        return boundary
    
    def cancel_learning(self) -> None:
        """Cancel boundary learning without saving."""
        if self._learning_active:
            name = self._learning_boundary.name if self._learning_boundary else "unknown"
            logger.info(f"Cancelled learning boundary: {name}")
        
        self._learning_active = False
        self._learning_boundary = None
    
    def get_learning_progress(self) -> Dict[str, Any]:
        """Get current learning progress."""
        if not self._learning_active or self._learning_boundary is None:
            return {'active': False}
        
        return {
            'active': True,
            'name': self._learning_boundary.name,
            'point_count': self._learning_boundary.point_count,
            'is_valid': self._learning_boundary.is_valid,
            'area_sq_meters': self._learning_boundary.area_sq_meters,
            'perimeter_meters': self._learning_boundary.perimeter_meters,
        }
    
    def save_boundary(self, name: str) -> bool:
        """
        Save boundary to JSON file.
        
        Args:
            name: Name of boundary to save
            
        Returns:
            True if saved successfully
        """
        if name not in self._boundaries:
            logger.error(f"Boundary '{name}' not found")
            return False
        
        boundary = self._boundaries[name]
        file_path = self._storage_path / f"{name}.json"
        
        try:
            with open(file_path, 'w') as f:
                json.dump(boundary.to_dict(), f, indent=2)
            
            logger.info(f"Saved boundary '{name}' to {file_path}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to save boundary '{name}': {e}")
            return False
    
    def load_boundary(self, name: str) -> Optional[Boundary]:
        """
        Load boundary from JSON file.
        
        Args:
            name: Name of boundary to load
            
        Returns:
            Loaded Boundary, or None if failed
        """
        file_path = self._storage_path / f"{name}.json"
        
        if not file_path.exists():
            logger.error(f"Boundary file not found: {file_path}")
            return None
        
        try:
            with open(file_path, 'r') as f:
                data = json.load(f)
            
            boundary = Boundary.from_dict(data)
            self._boundaries[name] = boundary
            
            logger.info(f"Loaded boundary '{name}' from {file_path}")
            return boundary
            
        except Exception as e:
            logger.error(f"Failed to load boundary '{name}': {e}")
            return None
    
    def list_saved_boundaries(self) -> List[str]:
        """List all saved boundary files."""
        return [f.stem for f in self._storage_path.glob("*.json")]
    
    def delete_boundary(self, name: str) -> bool:
        """
        Delete a boundary (from memory and disk).
        
        Args:
            name: Name of boundary to delete
            
        Returns:
            True if deleted successfully
        """
        # Remove from memory
        if name in self._boundaries:
            del self._boundaries[name]
        
        # Remove from disk
        file_path = self._storage_path / f"{name}.json"
        if file_path.exists():
            try:
                file_path.unlink()
                logger.info(f"Deleted boundary '{name}'")
                return True
            except Exception as e:
                logger.error(f"Failed to delete boundary file: {e}")
                return False
        
        return True
    
    def get_boundary(self, name: str) -> Optional[Boundary]:
        """Get a loaded boundary by name."""
        return self._boundaries.get(name)
    
    def is_inside(
        self, 
        boundary_name: str, 
        x: float, 
        y: float
    ) -> bool:
        """
        Check if a local point is inside the boundary.
        
        Uses ray casting algorithm.
        
        Args:
            boundary_name: Name of boundary to check against
            x: X coordinate (East) in meters
            y: Y coordinate (North) in meters
            
        Returns:
            True if point is inside boundary
        """
        boundary = self._boundaries.get(boundary_name)
        if boundary is None or not boundary.local_points:
            return False
        
        return self._point_in_polygon(x, y, boundary.local_points)
    
    def is_inside_gps(
        self, 
        boundary_name: str, 
        latitude: float, 
        longitude: float
    ) -> bool:
        """
        Check if a GPS point is inside the boundary.
        
        Args:
            boundary_name: Name of boundary to check against
            latitude: Latitude in degrees
            longitude: Longitude in degrees
            
        Returns:
            True if point is inside boundary
        """
        boundary = self._boundaries.get(boundary_name)
        if boundary is None or boundary.origin is None:
            return False
        
        # Convert GPS to local
        local = self._gps_to_local(
            GPSPoint(latitude=latitude, longitude=longitude),
            boundary.origin
        )
        
        return self._point_in_polygon(local.x, local.y, boundary.local_points)
    
    def distance_to_boundary(
        self, 
        boundary_name: str, 
        x: float, 
        y: float
    ) -> float:
        """
        Calculate minimum distance from point to boundary edge.
        
        Args:
            boundary_name: Name of boundary
            x: X coordinate (East) in meters
            y: Y coordinate (North) in meters
            
        Returns:
            Distance in meters (negative if outside)
        """
        boundary = self._boundaries.get(boundary_name)
        if boundary is None or not boundary.local_points:
            return float('inf')
        
        points = boundary.local_points
        n = len(points)
        min_dist = float('inf')
        
        for i in range(n):
            j = (i + 1) % n
            dist = self._point_to_segment_distance(
                x, y,
                points[i].x, points[i].y,
                points[j].x, points[j].y
            )
            min_dist = min(min_dist, dist)
        
        # Negative if outside
        if not self.is_inside(boundary_name, x, y):
            min_dist = -min_dist
        
        return min_dist
    
    def _gps_to_local(self, point: GPSPoint, origin: GPSPoint) -> LocalPoint:
        """Convert GPS to local ENU coordinates."""
        lat1 = math.radians(origin.latitude)
        lat2 = math.radians(point.latitude)
        dlon = math.radians(point.longitude - origin.longitude)
        dlat = lat2 - lat1
        
        # Approximate conversion for small distances
        x = dlon * self.EARTH_RADIUS * math.cos(lat1)  # East
        y = dlat * self.EARTH_RADIUS  # North
        
        return LocalPoint(x=x, y=y)
    
    def _point_in_polygon(
        self, 
        x: float, 
        y: float, 
        polygon: List[LocalPoint]
    ) -> bool:
        """Ray casting algorithm for point-in-polygon."""
        n = len(polygon)
        inside = False
        
        j = n - 1
        for i in range(n):
            xi, yi = polygon[i].x, polygon[i].y
            xj, yj = polygon[j].x, polygon[j].y
            
            if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
                inside = not inside
            
            j = i
        
        return inside
    
    def _point_to_segment_distance(
        self,
        px: float, py: float,
        x1: float, y1: float,
        x2: float, y2: float
    ) -> float:
        """Calculate distance from point to line segment."""
        dx = x2 - x1
        dy = y2 - y1
        
        if dx == 0 and dy == 0:
            # Segment is a point
            return math.sqrt((px - x1) ** 2 + (py - y1) ** 2)
        
        # Parameter t of closest point on line
        t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)))
        
        # Closest point on segment
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy
        
        return math.sqrt((px - closest_x) ** 2 + (py - closest_y) ** 2)
