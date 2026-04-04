#!/usr/bin/env python3
from dataclasses import dataclass
from typing import List, Tuple

from shapely.geometry import Point, Polygon, LineString, MultiPolygon

from mower_autonomy.boundary_utils import sanitize_boundary

XY = Tuple[float, float]


def _largest_polygon(g) -> Polygon:
    """
    If buffer(0) returns MultiPolygon, keep the largest area polygon.
    """
    if isinstance(g, Polygon):
        return g
    if isinstance(g, MultiPolygon):
        best = None
        best_area = -1.0
        for p in g.geoms:
            a = float(p.area)
            if a > best_area:
                best_area = a
                best = p
        return best if best is not None else Polygon()
    return Polygon()


@dataclass
class Geofence:
    boundary_xy: List[XY]
    polygon: Polygon              # original polygon (repaired)
    boundary_line: LineString     # boundary of original polygon
    safe_polygon: Polygon         # polygon buffered inward by margin
    margin_m: float

    @staticmethod
    def from_boundary(boundary_xy: List[XY], boundary_margin_m: float = 0.15) -> "Geofence":
        """
        Tolerant polygon creation:
        - Sanitizes boundary (dedup, closes, repairs)
        - Repairs invalid polygons with buffer(0)
        - If repair makes MultiPolygon, keeps the largest
        - If inward buffer too large, falls back to 0.0m margin automatically
        """
        if not boundary_xy or len(boundary_xy) < 3:
            raise ValueError("boundary_xy must have at least 3 points")

        # 1) sanitize (this is the key fix)
        b = sanitize_boundary(boundary_xy, close_tol=0.20, dedup_tol=0.03)
        if not b or len(b) < 3:
            raise ValueError("boundary_xy too small after sanitize")

        # 2) build polygon + repair if needed
        poly = Polygon(b)
        if (not poly.is_valid) or (poly.area <= 1e-6):
            repaired = poly.buffer(0)
            poly = _largest_polygon(repaired)

        if poly.is_empty or (not poly.is_valid) or (poly.area <= 1e-6):
            raise ValueError("boundary_xy does not form a usable polygon (even after repair)")

        # Re-export boundary coords from repaired polygon
        coords = list(poly.exterior.coords)
        boundary_out: List[XY] = [(float(x), float(y)) for (x, y) in coords]

        line = LineString(coords)

        # 3) safe polygon (inward buffer)
        margin = float(boundary_margin_m)
        safe = poly
        if margin > 0:
            safe_try = poly.buffer(-margin)
            if safe_try.is_empty or safe_try.area <= 1e-6:
                margin = 0.0
                safe = poly
            else:
                safe = safe_try

        return Geofence(
            boundary_xy=boundary_out,
            polygon=poly,
            boundary_line=line,
            safe_polygon=safe,
            margin_m=margin
        )

    def inside(self, x: float, y: float) -> bool:
        p = Point(x, y)
        return self.polygon.contains(p) or self.polygon.touches(p)

    def inside_safe(self, x: float, y: float) -> bool:
        p = Point(x, y)
        return self.safe_polygon.contains(p) or self.safe_polygon.touches(p)

    def distance_to_boundary(self, x: float, y: float) -> float:
        p = Point(x, y)
        return p.distance(self.boundary_line)

    def nearest_boundary_point(self, x: float, y: float) -> XY:
        p = Point(x, y)
        d = self.boundary_line.project(p)
        q = self.boundary_line.interpolate(d)
        return (float(q.x), float(q.y))

