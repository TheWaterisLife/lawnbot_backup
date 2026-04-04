#!/usr/bin/env python3
"""Grid-based obstacle memory and coverage tracking for lawn mower navigation.

Two co-located grids over the geofenced work area:

  obstacle_grid[r,c]  uint8   Obstacle confidence (0=free, >=OCCUPIED=blocked, 255=boundary)
  coverage_grid[r,c]  uint8   Visit count (0=unvisited, saturates at 255)

Key operations:
  add_obstacle()      Camera detection → increase obstacle confidence
  clear_frustum()     Camera FOV with no detection → decrease confidence
  decay()             Periodic passive confidence decay
  mark_covered()      Robot passed through → increment visit count
  next_waypoint()     BFS to nearest reachable unvisited cell, heading-biased
"""

from __future__ import annotations

import math
from collections import deque
from typing import List, Optional, Set, Tuple

import numpy as np
from shapely.geometry import Point, Polygon

XY = Tuple[float, float]

OCCUPIED_THRESHOLD = 50
PERMANENT = 255

DEFAULT_CELL_SIZE = 0.20
DEFAULT_OBSERVE_INCREMENT = 25
DEFAULT_FREE_DECREMENT = 12
DEFAULT_DECAY_AMOUNT = 3
DEFAULT_INFLATE_CELLS = 2


def _wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class CoverageGrid:
    """Dual-layer occupancy + coverage grid aligned to the mowable area."""

    def __init__(
        self,
        boundary_xy: List[XY],
        cell_size: float = DEFAULT_CELL_SIZE,
        boundary_margin_m: float = 0.15,
    ):
        poly = Polygon(boundary_xy)
        if not poly.is_valid or poly.area <= 1e-6:
            poly = poly.buffer(0)
        work = poly.buffer(-boundary_margin_m) if boundary_margin_m > 0 else poly
        if work.is_empty or work.area <= 1e-6:
            work = poly
        if work.geom_type == "MultiPolygon":
            work = max(work.geoms, key=lambda g: g.area)

        minx, miny, maxx, maxy = work.bounds
        self.cell_size = float(cell_size)
        self.origin_x = minx - self.cell_size
        self.origin_y = miny - self.cell_size
        self.cols = int(math.ceil((maxx - self.origin_x) / self.cell_size)) + 2
        self.rows = int(math.ceil((maxy - self.origin_y) / self.cell_size)) + 2

        self.obstacle = np.zeros((self.rows, self.cols), dtype=np.uint8)
        self.coverage = np.zeros((self.rows, self.cols), dtype=np.uint8)
        self.mowable = np.zeros((self.rows, self.cols), dtype=bool)

        for r in range(self.rows):
            for c in range(self.cols):
                wx, wy = self.grid_to_world(r, c)
                if work.contains(Point(wx, wy)) or work.boundary.distance(Point(wx, wy)) < 1e-4:
                    self.mowable[r, c] = True
                else:
                    self.obstacle[r, c] = PERMANENT

        self.total_mowable = int(np.sum(self.mowable))
        self._inflated: Optional[np.ndarray] = None
        self._inflated_dirty = True
        self._inflate_cells = DEFAULT_INFLATE_CELLS

    # ── coordinate helpers ────────────────────────────────────────────

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        c = int((x - self.origin_x) / self.cell_size)
        r = int((y - self.origin_y) / self.cell_size)
        return r, c

    def grid_to_world(self, r: int, c: int) -> Tuple[float, float]:
        x = self.origin_x + (c + 0.5) * self.cell_size
        y = self.origin_y + (r + 0.5) * self.cell_size
        return x, y

    def in_bounds(self, r: int, c: int) -> bool:
        return 0 <= r < self.rows and 0 <= c < self.cols

    # ── coverage tracking ─────────────────────────────────────────────

    def mark_covered(self, x: float, y: float, uncertainty_cells: int = 1) -> None:
        cr, cc = self.world_to_grid(x, y)
        for dr in range(-uncertainty_cells, uncertainty_cells + 1):
            for dc in range(-uncertainty_cells, uncertainty_cells + 1):
                nr, nc = cr + dr, cc + dc
                if self.in_bounds(nr, nc) and self.mowable[nr, nc]:
                    v = int(self.coverage[nr, nc])
                    if v < 255:
                        self.coverage[nr, nc] = v + 1

    def coverage_fraction(self) -> float:
        if self.total_mowable == 0:
            return 1.0
        return float(np.sum((self.coverage > 0) & self.mowable)) / self.total_mowable

    # ── obstacle memory ───────────────────────────────────────────────

    def add_obstacle(
        self,
        robot_x: float,
        robot_y: float,
        robot_theta: float,
        obs_distance: float,
        obs_angle_rad: float,
        increment: int = DEFAULT_OBSERVE_INCREMENT,
    ) -> Tuple[int, int]:
        """Project a camera detection into the grid and raise its confidence.
        Returns the (row, col) that was updated."""
        wx = robot_x + obs_distance * math.cos(robot_theta + obs_angle_rad)
        wy = robot_y + obs_distance * math.sin(robot_theta + obs_angle_rad)
        r, c = self.world_to_grid(wx, wy)
        if self.in_bounds(r, c) and self.obstacle[r, c] < PERMANENT:
            self.obstacle[r, c] = min(PERMANENT - 1, int(self.obstacle[r, c]) + increment)
            self._inflated_dirty = True
        return r, c

    def clear_frustum(
        self,
        robot_x: float,
        robot_y: float,
        robot_theta: float,
        fov_rad: float,
        max_range: float,
        detected_cells: Set[Tuple[int, int]],
        decrement: int = DEFAULT_FREE_DECREMENT,
    ) -> None:
        """Decrease obstacle score for cells inside the camera FOV that had
        no detection this frame (positive evidence of free space)."""
        half_fov = fov_rad / 2.0
        cx_list = [robot_x]
        cy_list = [robot_y]
        for a in (robot_theta - half_fov, robot_theta + half_fov):
            cx_list.append(robot_x + max_range * math.cos(a))
            cy_list.append(robot_y + max_range * math.sin(a))

        r_min, c_min = self.world_to_grid(min(cx_list), min(cy_list))
        r_max, c_max = self.world_to_grid(max(cx_list), max(cy_list))
        r_min = max(0, r_min - 1)
        r_max = min(self.rows - 1, r_max + 1)
        c_min = max(0, c_min - 1)
        c_max = min(self.cols - 1, c_max + 1)

        changed = False
        for r in range(r_min, r_max + 1):
            for c in range(c_min, c_max + 1):
                if (r, c) in detected_cells:
                    continue
                val = int(self.obstacle[r, c])
                if val == 0 or val == PERMANENT:
                    continue
                wx, wy = self.grid_to_world(r, c)
                dx, dy = wx - robot_x, wy - robot_y
                dist = math.hypot(dx, dy)
                if dist > max_range or dist < 0.05:
                    continue
                angle_diff = _wrap_pi(math.atan2(dy, dx) - robot_theta)
                if abs(angle_diff) <= half_fov:
                    self.obstacle[r, c] = max(0, val - decrement)
                    changed = True
        if changed:
            self._inflated_dirty = True

    def decay(self, amount: int = DEFAULT_DECAY_AMOUNT) -> None:
        """Passive decay: reduce all non-permanent obstacle scores."""
        mask = (self.obstacle > 0) & (self.obstacle < PERMANENT)
        if not np.any(mask):
            return
        vals = self.obstacle[mask].astype(np.int16) - amount
        self.obstacle[mask] = np.clip(vals, 0, PERMANENT - 1).astype(np.uint8)
        self._inflated_dirty = True

    # ── inflation ─────────────────────────────────────────────────────

    def _ensure_inflated(self, inflate_cells: int = DEFAULT_INFLATE_CELLS) -> np.ndarray:
        if (
            not self._inflated_dirty
            and self._inflated is not None
            and self._inflate_cells == inflate_cells
        ):
            return self._inflated

        inflated = (~self.mowable).copy()
        occ_cells = np.argwhere((self.obstacle >= OCCUPIED_THRESHOLD) & self.mowable)
        for r, c in occ_cells:
            r_lo = max(0, r - inflate_cells)
            r_hi = min(self.rows, r + inflate_cells + 1)
            c_lo = max(0, c - inflate_cells)
            c_hi = min(self.cols, c + inflate_cells + 1)
            inflated[r_lo:r_hi, c_lo:c_hi] = True

        self._inflated = inflated
        self._inflated_dirty = False
        self._inflate_cells = inflate_cells
        return inflated

    def is_blocked(self, x: float, y: float, inflate_cells: int = DEFAULT_INFLATE_CELLS) -> bool:
        inflated = self._ensure_inflated(inflate_cells)
        r, c = self.world_to_grid(x, y)
        if not self.in_bounds(r, c):
            return True
        return bool(inflated[r, c])

    # ── waypoint selection ────────────────────────────────────────────

    def next_waypoint(
        self,
        robot_x: float,
        robot_y: float,
        robot_theta: float,
        inflate_cells: int = DEFAULT_INFLATE_CELLS,
        heading_bias: float = 0.3,
        max_candidates: int = 30,
    ) -> Optional[XY]:
        """BFS from robot cell to find the nearest reachable unvisited cell.

        Collects up to *max_candidates* unvisited cells found in BFS order
        (nearest first), then picks the one best aligned with the current
        heading to promote stripe-like driving patterns.
        """
        start_r, start_c = self.world_to_grid(robot_x, robot_y)
        if not self.in_bounds(start_r, start_c):
            return None

        inflated = self._ensure_inflated(inflate_cells)

        # If the robot cell itself is inside an inflated zone, find the
        # nearest free cell as the BFS seed.
        if inflated[start_r, start_c]:
            best_d = 999
            for dr in range(-6, 7):
                for dc in range(-6, 7):
                    nr, nc = start_r + dr, start_c + dc
                    if self.in_bounds(nr, nc) and not inflated[nr, nc]:
                        d = abs(dr) + abs(dc)
                        if d < best_d:
                            best_d = d
                            start_r, start_c = nr, nc
            if best_d == 999:
                return None

        visited_bfs = np.zeros((self.rows, self.cols), dtype=bool)
        visited_bfs[start_r, start_c] = True
        queue: deque[Tuple[int, int, int]] = deque()
        queue.append((start_r, start_c, 0))
        candidates: List[Tuple[float, float, float]] = []

        while queue:
            r, c, dist = queue.popleft()

            if self.mowable[r, c] and self.coverage[r, c] == 0 and not inflated[r, c]:
                wx, wy = self.grid_to_world(r, c)
                dx, dy = wx - robot_x, wy - robot_y
                angle_to = math.atan2(dy, dx)
                hdiff = abs(_wrap_pi(angle_to - robot_theta))
                score = dist + heading_bias * (hdiff / math.pi) * max(1.0, dist * 0.5)
                candidates.append((score, wx, wy))
                if len(candidates) >= max_candidates:
                    break

            for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                nr, nc = r + dr, c + dc
                if self.in_bounds(nr, nc) and not visited_bfs[nr, nc] and not inflated[nr, nc]:
                    visited_bfs[nr, nc] = True
                    queue.append((nr, nc, dist + 1))

        if not candidates:
            return None
        candidates.sort(key=lambda t: t[0])
        return (candidates[0][1], candidates[0][2])

    # ── diagnostics ───────────────────────────────────────────────────

    def ascii_map(
        self,
        robot_x: float,
        robot_y: float,
        waypoint: Optional[XY] = None,
        max_display: int = 25,
    ) -> str:
        """Render a compact ASCII view of the grid.

        Legend: # boundary  X obstacle  ~ mowed  . unmowed  * target  M mower
        """
        mowable_rc = np.argwhere(self.mowable)
        if len(mowable_rc) == 0:
            return "(empty grid)"

        r_min, c_min = int(mowable_rc[:, 0].min()), int(mowable_rc[:, 1].min())
        r_max, c_max = int(mowable_rc[:, 0].max()), int(mowable_rc[:, 1].max())
        r_span = r_max - r_min + 1
        c_span = c_max - c_min + 1

        # Down-sample if larger than max_display
        step = max(1, max(r_span, c_span) // max_display)
        disp_rows = (r_span + step - 1) // step
        disp_cols = (c_span + step - 1) // step

        grid = [["." for _ in range(disp_cols)] for _ in range(disp_rows)]

        for di in range(disp_rows):
            for dj in range(disp_cols):
                r = r_min + di * step
                c = c_min + dj * step
                if not self.in_bounds(r, c) or not self.mowable[r, c]:
                    grid[di][dj] = "#"
                elif self.obstacle[r, c] >= OCCUPIED_THRESHOLD:
                    grid[di][dj] = "X"
                elif self.coverage[r, c] > 0:
                    grid[di][dj] = "~"

        # Robot marker
        rr, rc = self.world_to_grid(robot_x, robot_y)
        di = (rr - r_min) // step
        dj = (rc - c_min) // step
        if 0 <= di < disp_rows and 0 <= dj < disp_cols:
            grid[di][dj] = "M"

        # Waypoint marker
        if waypoint is not None:
            wr, wc = self.world_to_grid(waypoint[0], waypoint[1])
            wi = (wr - r_min) // step
            wj = (wc - c_min) // step
            if 0 <= wi < disp_rows and 0 <= wj < disp_cols and grid[wi][wj] != "M":
                grid[wi][wj] = "*"

        cov = self.coverage_fraction()
        hdr = f"MAP  {self.rows}×{self.cols} cells  {cov*100:.0f}% covered"
        lines = [f"╔══ {hdr} ══╗"]
        # Flip rows so y-up maps to screen-top
        for row in reversed(grid):
            lines.append("║ " + " ".join(row) + " ║")
        lines.append("╚" + "═" * (disp_cols * 2 + 2) + "╝")
        lines.append("  # boundary  X obstacle  ~ mowed  . unmowed  * target  M mower")
        return "\n".join(lines)
