#!/usr/bin/env python3
from typing import List, Tuple
import math

from shapely.geometry import Polygon, LineString

XY = Tuple[float, float]


def _sample_segment(seg: LineString, waypoint_spacing: float) -> List[XY]:
    length = seg.length
    if length <= 1e-6:
        return []
    n = max(2, int(math.ceil(length / waypoint_spacing)))
    pts: List[XY] = []
    for i in range(n + 1):
        d = (i / n) * length
        p = seg.interpolate(d)
        pts.append((float(p.x), float(p.y)))
    return pts


def generate_stripes(boundary: List[XY],
                     stripe_spacing: float = 0.18,
                     waypoint_spacing: float = 0.35,
                     boundary_margin_m: float = 0.15) -> List[List[XY]]:
    """
    Vertical stripes clipped to polygon buffered inward by boundary_margin_m.
    Returns list of stripes, each stripe is list of (x,y).
    """
    if len(boundary) < 3:
        return []

    poly = Polygon(boundary)
    if (not poly.is_valid) or (poly.area <= 0):
        return []

    margin = float(boundary_margin_m)
    work = poly.buffer(-margin) if margin > 0 else poly
    if work.is_empty or work.area <= 0:
        return []

    minx, miny, maxx, maxy = work.bounds

    stripes: List[List[XY]] = []
    direction_up = True

    x = minx
    # Extend vertical line beyond bounds for robust intersection
    y0 = miny - 10.0
    y1 = maxy + 10.0

    while x <= maxx + 1e-9:
        scan = LineString([(x, y0), (x, y1)])
        inter = work.intersection(scan)

        segs: List[LineString] = []
        if inter.is_empty:
            segs = []
        elif inter.geom_type == "LineString":
            segs = [inter]
        elif inter.geom_type == "MultiLineString":
            segs = list(inter.geoms)
        else:
            segs = []

        # For a simple polygon, usually 1 segment per x.
        # If multiple, sample each and concatenate.
        stripe_pts: List[XY] = []
        for s in segs:
            stripe_pts += _sample_segment(s, waypoint_spacing)

        # remove duplicates
        cleaned: List[XY] = []
        for p in stripe_pts:
            if not cleaned or (abs(p[0] - cleaned[-1][0]) > 1e-6 or abs(p[1] - cleaned[-1][1]) > 1e-6):
                cleaned.append(p)

        if len(cleaned) >= 2:
            if not direction_up:
                cleaned.reverse()
            stripes.append(cleaned)
            direction_up = not direction_up

        x += float(stripe_spacing)

    return stripes


def _layer_half_sizes(h_max: float, ring_half_step: float) -> List[float]:
    """Increasing half-sides from ring_half_step up to h_max (always includes outer ring)."""
    step = float(ring_half_step)
    hm = float(h_max)
    if step <= 0 or hm <= 0:
        return []
    if step > hm + 1e-9:
        return [hm]
    out: List[float] = []
    h = step
    while h < hm - 1e-9:
        out.append(h)
        h += step
    if not out or abs(out[-1] - hm) > 1e-6:
        out.append(hm)
    else:
        out[-1] = hm
    return out


def _sample_square_perimeter_ccw(
    cx: float, cy: float, half: float, waypoint_spacing: float
) -> List[XY]:
    """Axis-aligned square centered at (cx,cy), half-side `half`, sampled CCW from bottom-left."""
    h = float(half)
    bl = (cx - h, cy - h)
    br = (cx + h, cy - h)
    tr = (cx + h, cy + h)
    tl = (cx - h, cy + h)
    corners = [bl, br, tr, tl, bl]
    out: List[XY] = []
    for i in range(4):
        seg = LineString([corners[i], corners[i + 1]])
        pts = _sample_segment(seg, waypoint_spacing)
        if not pts:
            continue
        if out:
            if (
                abs(pts[0][0] - out[-1][0]) < 1e-6
                and abs(pts[0][1] - out[-1][1]) < 1e-6
            ):
                pts = pts[1:]
        out.extend(pts)
    # Drop closing duplicate of start corner
    if (
        len(out) >= 2
        and abs(out[0][0] - out[-1][0]) < 1e-6
        and abs(out[0][1] - out[-1][1]) < 1e-6
    ):
        out = out[:-1]
    return out


def generate_expanding_squares(
    boundary: List[XY],
    ring_half_step: float = 0.20,
    waypoint_spacing: float = 0.30,
    boundary_margin_m: float = 0.15,
    start_from_center: bool = True,
) -> List[List[XY]]:
    """
    Concentric square perimeters from the work-polygon center outward.

    Each inner list is one ring: waypoints sampled CCW around a square. The first
    ring optionally starts at the centroid (0-length hop to first perimeter point)
    so the mower begins at the center and spirals outward in square layers.

    Uses the same inward buffer as generate_stripes. The largest square half-side
    is the minimum distance from the centroid to the work polygon bounds (fits
    axis-aligned squares in rectangular lawns).
    """
    if len(boundary) < 3:
        return []

    poly = Polygon(boundary)
    if (not poly.is_valid) or (poly.area <= 0):
        return []

    margin = float(boundary_margin_m)
    work = poly.buffer(-margin) if margin > 0 else poly
    if work.is_empty or work.area <= 0:
        return []

    if work.geom_type == "MultiPolygon":
        work = max(work.geoms, key=lambda g: g.area)

    minx, miny, maxx, maxy = work.bounds
    c = work.centroid
    cx, cy = float(c.x), float(c.y)
    h_max = min(cx - minx, maxx - cx, cy - miny, maxy - cy)
    if h_max <= 1e-6:
        return []

    layers = _layer_half_sizes(h_max, ring_half_step)
    rings: List[List[XY]] = []
    for i, hh in enumerate(layers):
        ring = _sample_square_perimeter_ccw(cx, cy, hh, waypoint_spacing)
        if len(ring) < 2:
            continue
        if i == 0 and start_from_center:
            ring = [(cx, cy)] + ring
        rings.append(ring)

    return rings
