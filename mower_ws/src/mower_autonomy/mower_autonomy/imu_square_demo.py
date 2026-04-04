#!/usr/bin/env python3
"""2 m × 2 m demo: IMU-only heading + time-based distance, OAK-D obstacles, geofence.

Standalone script (like obstacle_avoidance_pi.py): camera + motors via WebSocket.
Uses ROS 2 only to subscribe to /imu/data from the BNO085.
NO ENCODERS — position is estimated from IMU heading × drive speed × elapsed time.

How it works:
  1. On start, records the IMU heading as yaw=0 and position (0,0).
  2. Builds a 2 m × 2 m boundary centered on start.
  3. Generates expanding square rings from the center (each ring larger) to cover the area.
  4. Navigates using:
       - HEADING from BNO085 quaternion (accurate, no drift)
       - POSITION from integrating heading × DRIVE_SPEED_MPS × dt
         (approximate — calibrate DRIVE_SPEED_MPS to match your mower)
  5. OAK-D YOLO detects obstacles; avoidance respects the boundary.

Calibration:
  Measure ground speed at motor_commander.SPEED_FORWARD on grass; set DRIVE_SPEED_MPS.
  All PWM limits match obstacle_avoidance_pi via motor_commander (SPEED_*).

Run (from Pi, after sourcing the workspace):
  cd ~/mower_ws && source install/setup.bash
  # Terminal A: start imu_node + motor server (or full stack minus autonomy)
  # Terminal B:
  python3 ~/mower_ws/src/mower_autonomy/mower_autonomy/imu_square_demo.py
"""

from __future__ import annotations

import asyncio
import logging
import math
import signal
import sys
import threading
import time
from pathlib import Path
from typing import List, Optional, Set, Tuple

# Import order matters: the ai_camera_vision venv often has NumPy 2.x, while
# Debian's python3-shapely is built against NumPy 1.x. Prepending the venv
# makes NumPy 2 load first and breaks shapely. So: workspace paths first,
# import mower_autonomy (shapely) with system NumPy, then append venv for depthai.
#
# mower_autonomy Python package lives at mower_ws/src/mower_autonomy/mower_autonomy/
# — prepending mower_ws/src alone makes "import mower_autonomy" miss source and
# load stale install/mower_autonomy/.../site-packages instead.
_SCRIPT_DIR = Path(__file__).resolve().parent
_PKG_ROOT = _SCRIPT_DIR.parent  # mower_ws/src/mower_autonomy (parent of package dir)
_WS_SRC = _PKG_ROOT.parent  # mower_ws/src
sys.path.insert(0, str(_WS_SRC / "obstacle_detection"))
sys.path.insert(0, str(_PKG_ROOT))

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from mower_autonomy.coverage_grid import CoverageGrid
from mower_autonomy.geofence import Geofence

_VENV_LIB = Path.home() / "mower_ws" / "src" / "ai_camera_vision" / "venv" / "lib"
for _sp in sorted(_VENV_LIB.glob("python*/site-packages"), reverse=True):
    if _sp.is_dir():
        sys.path.append(str(_sp))
        break

import depthai as dai

from ai_camera_vision.pipeline import create_yolo_only_pipeline
from motor_commander import (
    MotorCommander,
    SPEED_FORWARD,
    SPEED_SLOW,
    SPEED_TURN_INNER,
    SPEED_TURN_OUTER,
)

from obstacle_avoidance_pi import (  # type: ignore
    CONFIDENCE_THRESHOLD,
    FORCE_USB2,
    ROBOT_HALF_WIDTH,
    STOP_ESCAPE_TIMEOUT,
    STUCK_ESCAPE_TIMEOUT,
    MIN_TURN_HOLD_S,
    TrackedObstacle,
    WATCHDOG_TIMEOUT_S,
    YOLO_BLOB,
    ZONE_CRITICAL,
    ZONE_EMERGENCY,
    ZONE_SLOW,
    ZONE_WARNING,
    adjust_bbox_from_letterbox,
    build_obstacles,
    parse_img_detections,
)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("imu_square_demo")

# =====================================================================
# CALIBRATION — tune to your mower (PWM limits come from motor_commander)
# =====================================================================
# Ground speed (m/s) when commanding SPEED_FORWARD — measure 1 m / time.
DRIVE_SPEED_MPS = 0.1
# Proportional: ground speed at SPEED_SLOW, for dead-reckoning during slow_down.
DRIVE_SPEED_SLOW_MPS = DRIVE_SPEED_MPS * (SPEED_SLOW / SPEED_FORWARD)

# =====================================================================
# Navigation parameters (geometry only; motor PWM = motor_commander)
# =====================================================================
# Proportional arc steering magnitude, derived from obstacle turn aggressiveness.
STEER_CMD = min(0.25, (SPEED_TURN_OUTER + abs(SPEED_TURN_INNER)) / 3.0)
# Below this heading error (rad), the mower is "on heading" and drives
# explicit forward L/R (CRUISE) so physical motion matches dead-reckoning.
STRAIGHT_ERR_RAD = 0.10

SQUARE_HALF_M = 1.0
BOUNDARY_MARGIN_M = 0.15
WAYPOINT_ACCEPT_M = 0.35
PIVOT_ERR_RAD = 0.55
AVOID_PREDICT_M = 0.35
STATUS_LOG_INTERVAL_S = 2.0
MAX_OBSTACLE_RANGE_M = 0.80
OBSTACLE_CACHE_TTL_S = 0.35

# Grid-based coverage planner
GRID_CELL_SIZE = 0.20
COVERAGE_TARGET = 0.80
OBSTACLE_DECAY_INTERVAL_S = 3.0
CAMERA_HFOV_RAD = 1.2741  # math.radians(73.0)
REPLAN_INTERVAL_S = 12.0
GRID_MAP_INTERVAL_S = 30.0
# Tighter inflation + stronger stripe bias for small lawns (must match next_waypoint / is_blocked).
GRID_PLAN_INFLATE_CELLS = 1
GRID_HEADING_BIAS = 0.85

# Anti-spin safeguards
MAX_AVOIDANCE_DURATION_S = 20.0   # Force stop + replan after this much continuous avoidance
STUCK_REPLAN_THRESHOLD = 2        # Force replan after this many stuck escapes without forward


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def clamp_forward_arc(L: float, R: float) -> Tuple[float, float]:
    """If both tracks command forward, enforce motor_commander SPEED_SLOW floor."""
    L = max(-1.0, min(1.0, L))
    R = max(-1.0, min(1.0, R))
    if L > 0 and R > 0:
        L = max(SPEED_SLOW, L)
        R = max(SPEED_SLOW, R)
    return L, R


async def motor_pivot(commander: MotorCommander, err: float) -> None:
    """Same pivots as obstacle_avoidance_pi: MotorCommander.execute(pivot_left/right)."""
    if err > 0:
        await commander.execute("pivot_left", speed_factor=1.0)
    else:
        await commander.execute("pivot_right", speed_factor=1.0)


def quat_yaw(w: float, x: float, y: float, z: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def square_boundary(half: float) -> List[Tuple[float, float]]:
    h = float(half)
    return [(-h, -h), (h, -h), (h, h), (-h, h), (-h, -h)]


class ImuBridge(Node):
    """Thread-safe IMU heading from BNO085. Position integrated from speed × heading."""

    def __init__(self):
        super().__init__("imu_square_demo_bridge")
        self._lock = threading.Lock()
        self.imu_yaw: Optional[float] = None
        self._yaw_offset: Optional[float] = None
        self.last_imu_t = 0.0
        self.x = 0.0
        self.y = 0.0
        self._last_integrate_t: Optional[float] = None
        self._driving = False    # set by main loop when motors are driving forward
        self._drive_speed = 0.0  # effective speed being commanded

        self.create_subscription(Imu, "/imu/data", self._on_imu, 10)
        self.get_logger().info("Subscribed /imu/data (IMU-only mode, no encoders)")

    def _on_imu(self, msg: Imu):
        q = msg.orientation
        yaw = quat_yaw(q.w, q.x, q.y, q.z)
        now = time.time()
        with self._lock:
            if self._yaw_offset is None:
                self._yaw_offset = -yaw
                self._last_integrate_t = now
                self.get_logger().info(
                    f"IMU origin locked (raw yaw={math.degrees(yaw):.1f}°, "
                    f"offset={math.degrees(self._yaw_offset):.1f}°)"
                )
            self.imu_yaw = wrap_pi(yaw + self._yaw_offset)
            self.last_imu_t = now

            # Integrate position from speed × heading × dt
            if self._driving and self._drive_speed > 0 and self._last_integrate_t is not None:
                dt = now - self._last_integrate_t
                if 0 < dt < 0.2:  # sanity: ignore big gaps
                    ds = self._drive_speed * dt
                    self.x += ds * math.cos(self.imu_yaw)
                    self.y += ds * math.sin(self.imu_yaw)
            self._last_integrate_t = now

    def set_driving(self, driving: bool, speed: float = 0.0):
        """Called by main loop to tell the bridge when the mower is moving."""
        with self._lock:
            self._driving = driving
            self._drive_speed = speed

    def ready(self) -> bool:
        with self._lock:
            return self.imu_yaw is not None and self._yaw_offset is not None

    def snapshot(self) -> Tuple[float, float, float, bool]:
        with self._lock:
            if self.imu_yaw is None:
                return 0.0, 0.0, 0.0, False
            return self.x, self.y, self.imu_yaw, True

    def imu_fresh(self, timeout: float = 0.6) -> bool:
        return (time.time() - self.last_imu_t) <= timeout


def predict_inside(gf: Geofence, x: float, y: float, theta: float, direction: str) -> bool:
    turn_angle = math.pi / 4
    if direction == "left":
        th = theta + turn_angle
    else:
        th = theta - turn_angle
    px = x + AVOID_PREDICT_M * math.cos(th)
    py = y + AVOID_PREDICT_M * math.sin(th)
    return gf.inside_safe(px, py)


def boundary_filtered_pivot_dir(
    gf: Geofence, x: float, y: float, theta: float, suggested: str
) -> str:
    if suggested == "left":
        natural, opposite = "left", "right"
    else:
        natural, opposite = "right", "left"
    if predict_inside(gf, x, y, theta, natural):
        return natural
    if predict_inside(gf, x, y, theta, opposite):
        return opposite
    return "blocked"


# =====================================================================
# ASCII Map / Radar helpers
# =====================================================================

_HEADING_ARROWS = {
    0: "→", 1: "↗", 2: "↑", 3: "↖", 4: "←", 5: "↙", 6: "↓", 7: "↘",
}


def _heading_arrow(theta: float) -> str:
    idx = round(theta / (math.pi / 4)) % 8
    return _HEADING_ARROWS.get(idx, "→")


def print_startup_map(
    boundary: List[Tuple[float, float]],
    stripes: List[List[Tuple[float, float]]],
    grid_size: int = 21,
) -> None:
    """Print a full boundary map with ring waypoints on startup."""
    all_pts = list(boundary)
    for s in stripes:
        all_pts.extend(s)
    xs = [p[0] for p in all_pts]
    ys = [p[1] for p in all_pts]
    x_min, x_max = min(xs) - 0.1, max(xs) + 0.1
    y_min, y_max = min(ys) - 0.1, max(ys) + 0.1
    span = max(x_max - x_min, y_max - y_min)
    cx = (x_min + x_max) / 2
    cy = (y_min + y_max) / 2
    cell = span / grid_size

    grid = [["." for _ in range(grid_size)] for _ in range(grid_size)]

    def _to_cell(px: float, py: float) -> Tuple[int, int]:
        col = int((px - (cx - span / 2)) / cell)
        row = int(((cy + span / 2) - py) / cell)  # y-up → row-down
        return max(0, min(grid_size - 1, row)), max(0, min(grid_size - 1, col))

    for i in range(len(boundary) - 1):
        ax, ay = boundary[i]
        bx, by = boundary[(i + 1) % len(boundary)]
        steps = max(2, int(math.hypot(bx - ax, by - ay) / cell) + 1)
        for s in range(steps + 1):
            t = s / steps
            r, c = _to_cell(ax + t * (bx - ax), ay + t * (by - ay))
            grid[r][c] = "#"

    ring_markers = "123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    for si, stripe in enumerate(stripes):
        mk = ring_markers[si % len(ring_markers)]
        for wx, wy in stripe:
            r, c = _to_cell(wx, wy)
            if grid[r][c] == ".":
                grid[r][c] = mk

    mr, mc = _to_cell(0, 0)
    grid[mr][mc] = "M"

    print("\n╔══ STARTUP MAP (boundary=#  rings=1,2,3…  mower=M) ══╗")
    for row in grid:
        print("║ " + " ".join(row) + " ║")
    print("╚" + "═" * (grid_size * 2 + 2) + "╝\n")


def print_radar(
    x: float,
    y: float,
    theta: float,
    obstacles: List["TrackedObstacle"],
    waypoint: Optional[Tuple[float, float]],
    action: str,
    reason: str,
    coverage_pct: float,
    grid_size: int = 11,
    radius_m: float = 1.0,
) -> None:
    """Print a compact ego-centric radar: +x forward along heading, +y left.

    `TrackedObstacle.angle_rad` is already horizontal angle in the camera/robot
    frame (see obstacle_avoidance_pi.build_obstacles); do not add `theta`.
    Waypoint is rotated from world delta into the same body frame.
    """
    cell = (2 * radius_m) / grid_size
    grid = [["·" for _ in range(grid_size)] for _ in range(grid_size)]
    mid = grid_size // 2

    def _to_cell(fwd: float, left: float) -> Tuple[int, int]:
        c = mid + int(round(left / cell))
        r = mid - int(round(fwd / cell))
        return max(0, min(grid_size - 1, r)), max(0, min(grid_size - 1, c))

    for o in obstacles:
        fwd = o.distance * math.cos(o.angle_rad)
        left = o.distance * math.sin(o.angle_rad)
        r, c = _to_cell(fwd, left)
        grid[r][c] = "o"

    wp_dist_str = ""
    if waypoint:
        wx, wy = waypoint
        dx, dy = wx - x, wy - y
        w_fwd = dx * math.cos(theta) + dy * math.sin(theta)
        w_left = -dx * math.sin(theta) + dy * math.cos(theta)
        wr, wc = _to_cell(w_fwd, w_left)
        if 0 <= wr < grid_size and 0 <= wc < grid_size and grid[wr][wc] == "·":
            grid[wr][wc] = "*"
        wp_dist_str = f"  wp {math.hypot(dx, dy):.2f}m"

    grid[mid][mid] = _heading_arrow(theta)

    legend_lines = [
        f"{_heading_arrow(theta)}=mower  o=obstacle  *=waypoint",
        f"coverage {coverage_pct*100:.0f}%{wp_dist_str}",
        f"pos ({x:.2f},{y:.2f}) hdg {math.degrees(theta):.0f}°",
        f"ACT: {action}",
        f"     {reason}",
    ]

    print(f"\n┌── RADAR {'─'*((grid_size*2)-7)}┐")
    for i, row in enumerate(grid):
        line = "│ " + " ".join(row) + " │"
        if i < len(legend_lines):
            line += "  " + legend_lines[i]
        print(line)
    print(f"└{'─'*(grid_size*2+2)}┘")


def maybe_print_radar(
    t_last_status: float,
    now: float,
    x: float,
    y: float,
    theta: float,
    radar_obs: List["TrackedObstacle"],
    waypoint: Optional[Tuple[float, float]],
    act: str,
    rsn: str,
    coverage_pct: float,
) -> float:
    """Print radar at STATUS_LOG_INTERVAL_S; returns updated t_last_status."""
    if (now - t_last_status) < STATUS_LOG_INTERVAL_S:
        return t_last_status
    print_radar(
        x, y, theta, radar_obs, waypoint, act, rsn, coverage_pct,
    )
    return now


# =====================================================================
# Nav-aware obstacle avoidance (replaces heading-blind decide_action)
# =====================================================================

_NAV_CENTER_DEG = 5.0  # obstacle within ±5° of intended path = "dead center"


def _nav_side_clearance(
    obstacles: List[TrackedObstacle], side: str, heading_err: float
) -> float:
    """Clearance score on `side` of the *intended* path (not camera axis)."""
    score = 10.0
    for o in obstacles:
        if o.distance > ZONE_SLOW:
            continue
        nav_deg = math.degrees(o.angle_rad - heading_err)
        on_this_side = (side == "left" and nav_deg < 10) or (
            side == "right" and nav_deg > -10
        )
        if not on_this_side:
            continue
        weight = 1.0 / max(o.distance, 0.2) ** 2
        score -= weight
    return max(0.0, score)


def decide_action_nav(
    obstacles: List[TrackedObstacle],
    heading_err: float,
    stop_start: float,
) -> Tuple[str, str]:
    """Heading-aware avoidance: only react to obstacles that block the *intended*
    travel direction, and prefer escaping toward the waypoint when possible.

    heading_err: wrap_pi(desired_heading - current_heading).  Positive = WP is left.
    """
    # Recompute in_path relative to intended heading
    blockers: List[Tuple[TrackedObstacle, float]] = []
    for o in obstacles:
        nav_angle = o.angle_rad - heading_err
        lateral = abs(o.distance * math.sin(nav_angle))
        if lateral < (ROBOT_HALF_WIDTH + o.width / 2):
            blockers.append((o, nav_angle))

    if not blockers:
        return "forward", "nav path clear"

    blockers.sort(key=lambda pair: pair[0].distance)
    closest, closest_nav_angle = blockers[0]
    dist = closest.distance

    # Escape direction: away from obstacle in the nav frame, which naturally
    # biases toward the waypoint side when the obstacle is off-center.
    nav_deg = math.degrees(closest_nav_angle)
    if nav_deg > _NAV_CENTER_DEG:
        turn_dir = "left"
    elif nav_deg < -_NAV_CENTER_DEG:
        turn_dir = "right"
    else:
        ls = _nav_side_clearance(obstacles, "left", heading_err)
        rs = _nav_side_clearance(obstacles, "right", heading_err)
        if heading_err > 0:
            ls += 1.5
        elif heading_err < 0:
            rs += 1.5
        turn_dir = "left" if ls >= rs else "right"

    if dist < ZONE_EMERGENCY:
        if stop_start > 0 and (time.time() - stop_start) > STOP_ESCAPE_TIMEOUT:
            return (
                f"pivot_{turn_dir}",
                f"escape pivot {turn_dir} ({closest.label}@{dist:.2f}m)",
            )
        return "stop", f"EMERGENCY {closest.label}@{dist:.2f}m"

    if dist < ZONE_CRITICAL:
        return (
            f"pivot_{turn_dir}",
            f"critical pivot {turn_dir} ({closest.label}@{dist:.2f}m)",
        )

    if dist < ZONE_WARNING:
        return (
            f"turn_{turn_dir}",
            f"turn {turn_dir} ({closest.label}@{dist:.2f}m)",
        )

    if dist < ZONE_SLOW:
        return "slow_down", f"slow ({closest.label}@{dist:.2f}m)"

    return "forward", "nav path clear"


async def run_demo(bridge: ImuBridge, shutdown: asyncio.Event) -> None:
    commander = MotorCommander()
    await commander.ensure_connected()

    logger.info("Waiting for IMU data from BNO085…")
    t_wait = time.time()
    while not bridge.ready() and not shutdown.is_set():
        await asyncio.sleep(0.05)
        if time.time() - t_wait > 30.0:
            logger.error("Timeout: need /imu/data (start imu_node).")
            await commander.emergency_stop()
            return

    logger.info("IMU ready. Building 2 m × 2 m boundary…")
    boundary = square_boundary(SQUARE_HALF_M)
    gf = Geofence.from_boundary(boundary, boundary_margin_m=BOUNDARY_MARGIN_M)
    grid = CoverageGrid(
        gf.boundary_xy,
        cell_size=GRID_CELL_SIZE,
        boundary_margin_m=gf.margin_m,
    )
    if grid.total_mowable == 0:
        logger.error("No mowable cells inside boundary.")
        await commander.emergency_stop()
        return

    logger.info(
        "Grid ready | %d×%d (%.0f cm cells) | mowable=%d | margin=%.2f m | speed=%.2f m/s",
        grid.rows, grid.cols, grid.cell_size * 100,
        grid.total_mowable, gf.margin_m, DRIVE_SPEED_MPS,
    )
    print("\n" + grid.ascii_map(0.0, 0.0))

    current_wp: Optional[Tuple[float, float]] = None
    t_last_replan = 0.0
    t_last_decay = time.time()
    t_last_grid_map = time.time()
    cx, cy = 0.0, 0.0

    if not YOLO_BLOB.exists():
        logger.error("YOLO blob missing: %s", YOLO_BLOB)
        await commander.emergency_stop()
        return

    pipeline = create_yolo_only_pipeline(
        YOLO_BLOB, enable_depth=True, confidence_threshold=CONFIDENCE_THRESHOLD,
    )
    dev_kw = {"maxUsbSpeed": dai.UsbSpeed.HIGH} if FORCE_USB2 else {}

    radar_act = "nav"
    radar_rsn = "starting"
    last_obs_log_key = ""
    stop_start_time = 0.0
    stuck_start_time = 0.0
    turn_start_time = 0.0
    held_turn_action = ""
    t_last_frame = time.time()
    t_last_status = time.time()
    frame_count = 0
    radar_obstacles: List[TrackedObstacle] = []
    avoidance_start_time = 0.0
    stuck_escape_count = 0

    with dai.Device(pipeline, **dev_kw) as device:
        logger.info("OAK-D connected — USB %s", device.getUsbSpeed().name)
        q_det = device.getOutputQueue("detections", maxSize=4, blocking=False)
        q_depth = device.getOutputQueue("depth", maxSize=4, blocking=False)
        last_depth = None
        cached_obstacles: List[TrackedObstacle] = []
        cached_obstacles_t = 0.0

        while not shutdown.is_set():
            if not bridge.imu_fresh():
                bridge.set_driving(False)
                logger.warning("IMU stale — stopping.")
                await commander.stop("STALE_IMU")
                await asyncio.sleep(0.1)
                continue

            x, y, theta, ok = bridge.snapshot()
            if not ok:
                bridge.set_driving(False)
                await asyncio.sleep(0.02)
                continue

            # --- Geofence recovery ---
            if not gf.inside_safe(x, y):
                rec_dx, rec_dy = cx - x, cy - y
                rec_err = wrap_pi(math.atan2(rec_dy, rec_dx) - theta)
                if abs(rec_err) > PIVOT_ERR_RAD:
                    bridge.set_driving(False)
                    await motor_pivot(commander, rec_err)
                else:
                    rfwd = max(SPEED_SLOW, SPEED_FORWARD * 0.8)
                    rturn = max(-1.0, min(1.0, rec_err)) * STEER_CMD
                    Lr, Rr = clamp_forward_arc(rfwd - rturn, rfwd + rturn)
                    bridge.set_driving(True, DRIVE_SPEED_MPS * 0.8)
                    await commander.drive(Lr, Rr, "RECOVER")
                await asyncio.sleep(0.05)
                continue

            det_data = q_det.tryGet()
            depth_data = q_depth.tryGet()
            if depth_data is not None:
                last_depth = depth_data.getFrame()

            # --- Coverage tracking ---
            grid.mark_covered(x, y)

            now_t = time.time()

            # --- Periodic obstacle decay ---
            if now_t - t_last_decay >= OBSTACLE_DECAY_INTERVAL_S:
                grid.decay()
                t_last_decay = now_t

            # --- Periodic grid map ---
            if now_t - t_last_grid_map >= GRID_MAP_INTERVAL_S:
                print("\n" + grid.ascii_map(x, y, current_wp))
                t_last_grid_map = now_t

            # --- Coverage target check ---
            cov_frac = grid.coverage_fraction()
            if cov_frac >= COVERAGE_TARGET:
                bridge.set_driving(False)
                logger.info("Coverage target reached (%.0f%%) — stopping.", cov_frac * 100)
                print("\n" + grid.ascii_map(x, y))
                await commander.stop("DONE")
                break

            # --- Dynamic waypoint selection ---
            need_replan = current_wp is None
            if not need_replan:
                tx, ty = current_wp
                wp_dist = math.hypot(tx - x, ty - y)
                if wp_dist <= WAYPOINT_ACCEPT_M:
                    need_replan = True
                elif grid.is_blocked(tx, ty, inflate_cells=GRID_PLAN_INFLATE_CELLS):
                    logger.info("Waypoint blocked — replanning")
                    need_replan = True
                elif (now_t - t_last_replan) > REPLAN_INTERVAL_S:
                    need_replan = True

            if need_replan:
                current_wp = grid.next_waypoint(
                    x, y, theta,
                    inflate_cells=GRID_PLAN_INFLATE_CELLS,
                    heading_bias=GRID_HEADING_BIAS,
                )
                t_last_replan = now_t
                if current_wp is None:
                    bridge.set_driving(False)
                    logger.info(
                        "No reachable unvisited cells (%.0f%% covered) — stopping.",
                        cov_frac * 100,
                    )
                    print("\n" + grid.ascii_map(x, y))
                    await commander.stop("DONE")
                    break

            tx, ty = current_wp
            dx = tx - x
            dy = ty - y
            dist = math.hypot(dx, dy)

            # --- Compute heading error for navigation ---
            desired = math.atan2(dy, dx)
            err = wrap_pi(desired - theta)

            # --- Obstacle handling ---
            now_obs = time.time()
            use_obstacle_branch = False
            if det_data is not None:
                frame_count += 1
                t_last_frame = now_obs
                raw = parse_img_detections(det_data)
                adjusted = [adjust_bbox_from_letterbox(d) for d in raw]
                obstacles = [
                    o
                    for o in build_obstacles(adjusted, last_depth)
                    if o.distance <= MAX_OBSTACLE_RANGE_M
                ]
                cached_obstacles = obstacles
                cached_obstacles_t = now_obs
                use_obstacle_branch = True

                # --- Write detections into obstacle grid ---
                detected_cells: Set[Tuple[int, int]] = set()
                for o in obstacles:
                    rc = grid.add_obstacle(
                        x, y, theta, o.distance, o.angle_rad,
                    )
                    detected_cells.add(rc)
                grid.clear_frustum(
                    x, y, theta, CAMERA_HFOV_RAD,
                    MAX_OBSTACLE_RANGE_M, detected_cells,
                )

            elif (now_obs - cached_obstacles_t) < OBSTACLE_CACHE_TTL_S:
                obstacles = cached_obstacles
                use_obstacle_branch = True

            if use_obstacle_branch:
                if obstacles:
                    radar_obstacles = list(obstacles)
                action, reason = decide_action_nav(obstacles, err, stop_start_time)

                is_turn = action.startswith("turn_") or action.startswith("pivot_")
                if is_turn:
                    if turn_start_time == 0:
                        turn_start_time = time.time()
                    held_turn_action = action
                elif action in ("forward", "slow_down") and held_turn_action:
                    if time.time() - turn_start_time < MIN_TURN_HOLD_S:
                        action = held_turn_action
                        reason = "holding turn"
                    else:
                        turn_start_time = 0.0
                        held_turn_action = ""
                else:
                    turn_start_time = 0.0
                    held_turn_action = ""

                if action == "forward":
                    stuck_start_time = 0.0
                    avoidance_start_time = 0.0
                    stuck_escape_count = 0
                else:
                    if avoidance_start_time == 0.0:
                        avoidance_start_time = time.time()
                    if stuck_start_time == 0:
                        stuck_start_time = time.time()

                    # Prolonged avoidance → force stop + replan
                    avoidance_elapsed = time.time() - avoidance_start_time
                    if avoidance_elapsed > MAX_AVOIDANCE_DURATION_S:
                        logger.warning(
                            "Avoidance timeout (%.0fs) — forcing replan",
                            avoidance_elapsed,
                        )
                        action = "stop"
                        reason = "avoidance timeout — replanning"
                        avoidance_start_time = 0.0
                        stuck_start_time = 0.0
                        stuck_escape_count = 0
                        current_wp = None
                        t_last_replan = 0.0
                    elif (time.time() - stuck_start_time) > STUCK_ESCAPE_TIMEOUT:
                        stuck_escape_count += 1
                        if stuck_escape_count >= STUCK_REPLAN_THRESHOLD:
                            logger.warning(
                                "Stuck %d times without forward — forcing replan",
                                stuck_escape_count,
                            )
                            action = "stop"
                            reason = "repeated stuck — replanning"
                            avoidance_start_time = 0.0
                            stuck_start_time = 0.0
                            stuck_escape_count = 0
                            current_wp = None
                            t_last_replan = 0.0
                        else:
                            action = "pivot_180"
                            stuck_start_time = time.time()  # restart for next escape

                # Boundary filter
                if action in ("pivot_left", "turn_left"):
                    side = boundary_filtered_pivot_dir(gf, x, y, theta, "left")
                    if side == "blocked":
                        action = "stop"
                        reason = "obstacle + boundary blocked"
                    elif side == "right":
                        action = action.replace("left", "right")
                elif action in ("pivot_right", "turn_right"):
                    side = boundary_filtered_pivot_dir(gf, x, y, theta, "right")
                    if side == "blocked":
                        action = "stop"
                        reason = "obstacle + boundary blocked"
                    elif side == "left":
                        action = action.replace("right", "left")
                elif action == "pivot_180":
                    left_ok = predict_inside(gf, x, y, theta, "left")
                    right_ok = predict_inside(gf, x, y, theta, "right")
                    if left_ok:
                        action = "pivot_left"
                        reason = "stuck escape (left safe)"
                    elif right_ok:
                        action = "pivot_right"
                        reason = "stuck escape (right safe)"
                    else:
                        action = "stop"
                        reason = "stuck but boundary blocks both sides"

                # Stop timer (after boundary filter)
                if action == "stop":
                    if stop_start_time == 0:
                        stop_start_time = time.time()
                else:
                    stop_start_time = 0.0

                if action == "stop" and stop_start_time and (time.time() - stop_start_time) > STOP_ESCAPE_TIMEOUT:
                    in_path = [o for o in obstacles if o.in_path]
                    if in_path:
                        in_path.sort(key=lambda o: o.distance)
                        sug = "left" if in_path[0].angle_rad > 0 else "right"
                    else:
                        sug = "left"
                    side = boundary_filtered_pivot_dir(gf, x, y, theta, sug)
                    if side != "blocked":
                        action = f"pivot_{side}"
                        reason = "stop escape"
                        stop_start_time = 0.0

                obstacle_active = action not in ("forward",)
                radar_act = action
                radar_rsn = reason

                if obstacle_active:
                    if action != last_obs_log_key:
                        logger.info("OBS: %s (%s)", action, reason)
                        last_obs_log_key = action
                    if action == "slow_down":
                        bridge.set_driving(True, DRIVE_SPEED_SLOW_MPS)
                        await commander.execute("slow_down", speed_factor=1.0)
                    elif action == "stop":
                        bridge.set_driving(False)
                        await commander.execute("stop", speed_factor=1.0)
                    else:
                        # Pivots/turns: not translating forward
                        bridge.set_driving(False)
                        await commander.execute(action, speed_factor=1.0)
                    await asyncio.sleep(0.005)
                    t_last_status = maybe_print_radar(
                        t_last_status, time.time(), x, y, theta,
                        radar_obstacles, current_wp, radar_act, radar_rsn,
                        cov_frac,
                    )
                    continue

            if not use_obstacle_branch:
                radar_act = "nav"
                radar_rsn = f"heading to wp ({dist:.2f}m)"

            # Camera watchdog
            if frame_count > 0 and (time.time() - t_last_frame) > WATCHDOG_TIMEOUT_S:
                bridge.set_driving(False)
                logger.warning("Camera watchdog — stopping.")
                await commander.emergency_stop()
                t_last_frame = time.time()
                continue

            # --- Normal navigation ---
            abs_err = abs(err)
            if abs_err > PIVOT_ERR_RAD:
                bridge.set_driving(False)
                await motor_pivot(commander, err)
            elif abs_err < STRAIGHT_ERR_RAD:
                bridge.set_driving(True, DRIVE_SPEED_MPS)
                await commander.drive(SPEED_FORWARD, SPEED_FORWARD, "CRUISE")
            else:
                scale = min(1.0, max(0.35, dist / 0.35))
                fwd = max(SPEED_SLOW, SPEED_FORWARD * scale)
                turn = max(-1.0, min(1.0, 0.9 * err)) * STEER_CMD
                L, R = clamp_forward_arc(fwd - turn, fwd + turn)
                bridge.set_driving(True, DRIVE_SPEED_MPS * scale)
                await commander.drive(L, R, "NAV")
            t_last_status = maybe_print_radar(
                t_last_status, time.time(), x, y, theta,
                radar_obstacles, current_wp, radar_act, radar_rsn,
                cov_frac,
            )

            await asyncio.sleep(0.02)

    bridge.set_driving(False)
    await commander.emergency_stop()
    await commander.close()
    logger.info("Shutdown complete.")


def main() -> int:
    print("=" * 60)
    print("  GRID COVERAGE NAV (obstacle memory + dynamic replan)")
    print("  Requires: /imu/data, motor WS :8766")
    print(f"  Speed: {DRIVE_SPEED_MPS} m/s | Cell: {GRID_CELL_SIZE*100:.0f} cm | Target: {COVERAGE_TARGET*100:.0f}%")
    print("=" * 60)

    if "--debug" in sys.argv:
        logging.getLogger("imu_square_demo").setLevel(logging.DEBUG)

    rclpy.init()
    bridge = ImuBridge()
    spin_thread = threading.Thread(target=rclpy.spin, args=(bridge,), daemon=True)
    spin_thread.start()

    shutdown = asyncio.Event()

    def _sig(*_):
        logger.info("Signal — shutdown")
        shutdown.set()

    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    try:
        asyncio.run(run_demo(bridge, shutdown))
    except KeyboardInterrupt:
        shutdown.set()
    finally:
        # Shut down rclpy first to break the spin loop,
        # then join the thread, then destroy the node.
        # This prevents "terminate called without an active exception".
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        try:
            bridge.destroy_node()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
