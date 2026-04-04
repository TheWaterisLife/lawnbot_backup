#!/usr/bin/env python3
import os
import json
from dataclasses import dataclass
from typing import List, Tuple, Optional

MAPS_DIR_DEFAULT = os.path.expanduser("~/mower_ws/maps")


@dataclass
class LoadedMap:
    path: str
    origin_latlon: Tuple[float, float]              # (lat0, lon0)
    boundary_xy: List[Tuple[float, float]]          # [(x,y), ...] meters
    created_at: Optional[str] = None
    raw_points_xy: Optional[List[Tuple[float, float]]] = None  # optional


def _get_created_at(data: dict) -> Optional[str]:
    v = data.get("created_at")
    return str(v) if v is not None else None


def _parse_origin(data: dict) -> Tuple[float, float]:
    origin = data.get("origin_latlon")
    if not isinstance(origin, list) or len(origin) != 2:
        raise ValueError("Map JSON missing origin_latlon [lat, lon]")
    lat0 = float(origin[0])
    lon0 = float(origin[1])
    if abs(lat0) < 1e-12 and abs(lon0) < 1e-12:
        # reject bogus origin, forces fallback to older map or GPS-first-fix behavior
        raise ValueError("Map origin_latlon is [0,0] (invalid)")
    return lat0, lon0


def _parse_boundary(data: dict) -> List[Tuple[float, float]]:
    boundary = data.get("boundary_xy")
    if not isinstance(boundary, list) or len(boundary) < 3:
        raise ValueError("Map JSON missing boundary_xy or too few points")
    out: List[Tuple[float, float]] = []
    for p in boundary:
        if not isinstance(p, list) or len(p) != 2:
            raise ValueError("boundary_xy must be list of [x,y]")
        out.append((float(p[0]), float(p[1])))
    return out


def _parse_raw_points(data: dict) -> Optional[List[Tuple[float, float]]]:
    raw = data.get("raw_points_xy")
    if raw is None:
        return None
    if not isinstance(raw, list):
        return None
    out: List[Tuple[float, float]] = []
    for p in raw:
        if not isinstance(p, list) or len(p) != 2:
            continue
        out.append((float(p[0]), float(p[1])))
    return out if out else None


def _list_json_files_sorted(maps_dir: str) -> List[str]:
    if not os.path.isdir(maps_dir):
        raise FileNotFoundError(f"Maps directory not found: {maps_dir}")

    candidates: List[str] = []
    for name in os.listdir(maps_dir):
        if name.lower().endswith(".json"):
            full = os.path.join(maps_dir, name)
            if os.path.isfile(full):
                candidates.append(full)

    if not candidates:
        raise FileNotFoundError(f"No .json maps found in: {maps_dir}")

    # newest first
    candidates.sort(key=lambda p: os.path.getmtime(p), reverse=True)
    return candidates


def load_latest_map(maps_dir: str = MAPS_DIR_DEFAULT) -> LoadedMap:
    """
    Loads the newest VALID map JSON.
    If the newest file is invalid JSON or missing required fields, it is skipped.
    """
    files = _list_json_files_sorted(maps_dir)

    last_err: Optional[Exception] = None
    for path in files:
        try:
            with open(path, "r") as f:
                data = json.load(f)

            origin = _parse_origin(data)
            boundary = _parse_boundary(data)
            created_at = _get_created_at(data)
            raw_points = _parse_raw_points(data)

            return LoadedMap(
                path=path,
                origin_latlon=origin,
                boundary_xy=boundary,
                created_at=created_at,
                raw_points_xy=raw_points,
            )
        except Exception as e:
            last_err = e
            # skip invalid/broken map and try next newest
            continue

    raise ValueError(f"All map files in {maps_dir} are invalid. Last error: {last_err}")


def load_map_by_filename(filename: str, maps_dir: str = MAPS_DIR_DEFAULT) -> LoadedMap:
    """
    Loads a specific map filename (still useful for manual debugging).
    """
    path = os.path.join(maps_dir, filename)
    if not os.path.isfile(path):
        raise FileNotFoundError(f"Map file not found: {path}")

    with open(path, "r") as f:
        data = json.load(f)

    origin = _parse_origin(data)
    boundary = _parse_boundary(data)
    created_at = _get_created_at(data)
    raw_points = _parse_raw_points(data)

    return LoadedMap(
        path=path,
        origin_latlon=origin,
        boundary_xy=boundary,
        created_at=created_at,
        raw_points_xy=raw_points,
    )


if __name__ == "__main__":
    m = load_latest_map()
    print("Loaded map:", m.path)
    print("created_at:", m.created_at)
    print("origin_latlon:", m.origin_latlon)
    print("boundary points:", len(m.boundary_xy))
    print("first point:", m.boundary_xy[0])
    print("last point:", m.boundary_xy[-1])
