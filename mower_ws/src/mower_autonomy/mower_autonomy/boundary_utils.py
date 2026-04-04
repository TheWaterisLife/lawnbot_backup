#!/usr/bin/env python3
from typing import List, Tuple
import math

from shapely.geometry import Polygon
from shapely.geometry.polygon import orient

XY = Tuple[float, float]


def _dist(a: XY, b: XY) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def sanitize_boundary(boundary_xy: List[XY],
                      close_tol: float = 0.10,
                      dedup_tol: float = 0.02) -> List[XY]:
    """
    Make boundary tolerant:
      - remove near-duplicate consecutive points
      - ensure closed (append first) if within close_tol OR just force close
      - attempt polygon repair with buffer(0)
      - ensure CCW orientation
    """
    if not boundary_xy or len(boundary_xy) < 3:
        return boundary_xy

    # 1) remove consecutive near-duplicates
    cleaned: List[XY] = []
    for p in boundary_xy:
        pt = (float(p[0]), float(p[1]))
        if not cleaned or _dist(pt, cleaned[-1]) > dedup_tol:
            cleaned.append(pt)

    if len(cleaned) < 3:
        return cleaned

    # 2) close polygon if not closed
    if _dist(cleaned[0], cleaned[-1]) > close_tol:
        cleaned.append(cleaned[0])

    # 3) repair polygon if invalid (self-intersection etc.)
    poly = Polygon(cleaned)
    if not poly.is_valid:
        poly = poly.buffer(0)

    # If still bad, return the cleaned list (caller may choose fallback)
    if poly.is_empty or poly.area <= 1e-6:
        return cleaned

    # 4) enforce CCW orientation
    poly = orient(poly, sign=1.0)
    coords = list(poly.exterior.coords)

    return [(float(x), float(y)) for (x, y) in coords]
