#!/home/lawnbot/mower_ws/src/ai_camera_vision/venv/bin/python3
"""Headless obstacle avoidance for the Raspberry Pi mower.

Runs a single YOLOv4-tiny model (12 shaves) on the OAK-D Lite for object
detection, uses stereo depth for distance estimation, and drives the motors
via WebSocket to avoid obstacles in real time.

No GUI — designed for deployment on the Pi as a long-running process.

Run with:
    ./obstacle_avoidance_pi.py          # normal (uses venv automatically)
    ./obstacle_avoidance_pi.py --debug  # verbose detection logs
"""

import asyncio
import logging
import math
import signal
import sys
import time
from pathlib import Path

# ---------------------------------------------------------------------------
# Path setup
# ---------------------------------------------------------------------------
SCRIPT_DIR = Path(__file__).resolve().parent
SRC_DIR = SCRIPT_DIR.parent
sys.path.insert(0, str(SRC_DIR / "ai_camera_vision"))
sys.path.insert(0, str(SCRIPT_DIR))

import depthai as dai

from ai_camera_vision.pipeline import create_yolo_only_pipeline, CAM_PREVIEW_SIZE, YOLO_INPUT_SIZE
from ai_camera_vision.depth import BoundingBox2D, sample_depth_robust
from ai_camera_vision.detections import Detection2D, COCO_CLASSES, SAFETY_CATEGORY_MAP
from motor_commander import MotorCommander

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("obstacle_avoidance")

# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
YOLO_BLOB = (
    SRC_DIR / "ai_camera_vision" / "models"
    / "yolo-v3-tiny-tf_openvino_2021.4_6shave.blob"
)

# ---------------------------------------------------------------------------
# Tuneable parameters
# ---------------------------------------------------------------------------
CAMERA_HFOV_DEG = 73.0
WATCHDOG_TIMEOUT_S = 5.0
STATUS_LOG_INTERVAL_S = 5.0
FORCE_USB2 = False
CONFIDENCE_THRESHOLD = 0.3

# Avoidance zones (meters). Kept tight so far-off background objects (chairs,
# walls) don't interfere with navigation. Only react to real close threats.
ZONE_EMERGENCY = 0.30   # Closer than this -> full stop
ZONE_CRITICAL = 0.35    # Closer than this -> pivot away
ZONE_WARNING = 0.50     # Closer than this -> arc turn away
ZONE_SLOW = 0.75        # Closer than this -> slow down (outer reactive range)

# Mower is ~45cm wide (22.5cm per side from center).
# Add 15cm safety margin per side to account for turn swing and sensor lag.
ROBOT_HALF_WIDTH = 0.45   # 22.5cm body + 22.5cm margin — generous clearance

# Escape timers
STOP_ESCAPE_TIMEOUT = 2.0   # Seconds stopped before pivot escape
STUCK_ESCAPE_TIMEOUT = 5.0  # Seconds in any non-forward state before 180
MIN_TURN_HOLD_S = 1.5       # Keep turning for at least this long before going forward



# =============================================================================
# Detection helpers
# =============================================================================

def parse_img_detections(img_detections) -> list[Detection2D]:
    detections = []
    for det in img_detections.detections:
        class_id = det.label
        class_label = COCO_CLASSES[class_id] if class_id < len(COCO_CLASSES) else f"class_{class_id}"
        safety_cat = SAFETY_CATEGORY_MAP.get(class_label, "unknown")
        detections.append(Detection2D(
            class_id=class_id, class_label=class_label,
            confidence=det.confidence,
            x_min=det.xmin, y_min=det.ymin,
            x_max=det.xmax, y_max=det.ymax,
            safety_category=safety_cat,
        ))
    return detections


def adjust_bbox_from_letterbox(det: Detection2D) -> Detection2D:
    nn_w, nn_h = YOLO_INPUT_SIZE
    preview_w, preview_h = CAM_PREVIEW_SIZE
    scale = min(nn_w / preview_w, nn_h / preview_h)
    padded_w, padded_h = preview_w * scale, preview_h * scale
    off_x = (nn_w - padded_w) / 2 / nn_w
    off_y = (nn_h - padded_h) / 2 / nn_h
    sx, sy = nn_w / padded_w, nn_h / padded_h
    clamp = lambda v: max(0.0, min(1.0, v))
    return Detection2D(
        class_id=det.class_id, class_label=det.class_label,
        confidence=det.confidence,
        x_min=clamp((det.x_min - off_x) * sx),
        y_min=clamp((det.y_min - off_y) * sy),
        x_max=clamp((det.x_max - off_x) * sx),
        y_max=clamp((det.y_max - off_y) * sy),
        safety_category=det.safety_category,
    )


class TrackedObstacle:
    """A detected obstacle with spatial info."""
    __slots__ = ("label", "distance", "angle_rad", "lateral", "width", "in_path")

    def __init__(self, label: str, distance: float, angle_rad: float, width: float):
        self.label = label
        self.distance = distance
        self.angle_rad = angle_rad
        self.width = width
        self.lateral = abs(distance * math.sin(angle_rad))
        self.in_path = self.lateral < (ROBOT_HALF_WIDTH + width / 2)


def build_obstacles(detections: list[Detection2D], depth_frame) -> list[TrackedObstacle]:
    if depth_frame is None:
        return []
    dh, dw = depth_frame.shape[:2]
    hfov = math.radians(CAMERA_HFOV_DEG)
    result = []
    for det in detections:
        bbox = BoundingBox2D(det.x_min, det.y_min, det.x_max, det.y_max)
        depth_mm, _ = sample_depth_robust(depth_frame, bbox, frame_shape=(dh, dw), sample_count=5)
        if depth_mm <= 0:
            continue
        dist = depth_mm / 1000.0
        angle = (det.center_x - 0.5) * hfov
        width = det.width * dist
        result.append(TrackedObstacle(det.class_label, dist, angle, width))
    return result


# =============================================================================
# Avoidance decision engine
# =============================================================================

def decide_action(obstacles: list[TrackedObstacle],
                  stop_start: float) -> tuple[str, str]:
    """Distance-layered avoidance: deal with the nearest obstacle first,
    pick a direction that also keeps a gap for farther obstacles.

    Returns (action_name, reason) where action_name is one of:
        forward, slow_down, stop, turn_left, turn_right,
        pivot_left, pivot_right, pivot_180
    """
    in_path = [o for o in obstacles if o.in_path]
    if not in_path:
        return "forward", "path clear"

    in_path.sort(key=lambda o: o.distance)
    closest = in_path[0]
    dist = closest.distance

    turn_dir = _best_escape_dir(closest, obstacles)

    # EMERGENCY: too close -> stop (then pivot after timeout)
    if dist < ZONE_EMERGENCY:
        if stop_start > 0 and (time.time() - stop_start) > STOP_ESCAPE_TIMEOUT:
            return f"pivot_{turn_dir}", f"escape pivot {turn_dir} ({closest.label}@{dist:.2f}m)"
        return "stop", f"EMERGENCY {closest.label}@{dist:.2f}m"

    # CRITICAL: pivot away
    if dist < ZONE_CRITICAL:
        return f"pivot_{turn_dir}", f"critical pivot {turn_dir} ({closest.label}@{dist:.2f}m)"

    # WARNING: arc turn away
    if dist < ZONE_WARNING:
        return f"turn_{turn_dir}", f"turn {turn_dir} ({closest.label}@{dist:.2f}m)"

    # SLOW: reduce speed
    if dist < ZONE_SLOW:
        return "slow_down", f"slow ({closest.label}@{dist:.2f}m)"

    return "forward", "path clear"


def _best_escape_dir(nearest: TrackedObstacle,
                     all_obs: list[TrackedObstacle]) -> str:
    """Pick the escape direction that avoids the nearest obstacle AND
    has the best gap for any obstacles behind it.

    Strategy:
      1. If the nearest obstacle is clearly to one side (>5 deg off center),
         the natural escape is away from it.
      2. Score both left and right by how much clear space exists on each
         side, considering ALL obstacles — not just in-path ones.
      3. If the natural direction is significantly worse (much less space),
         override it with the better side.
    """
    angle_deg = math.degrees(nearest.angle_rad)

    left_score = _side_clearance(all_obs, "left")
    right_score = _side_clearance(all_obs, "right")

    if angle_deg > 5:
        natural = "left"
    elif angle_deg < -5:
        natural = "right"
    else:
        return "left" if left_score >= right_score else "right"

    if natural == "left" and right_score > left_score * 2.0:
        return "right"
    if natural == "right" and left_score > right_score * 2.0:
        return "left"

    return natural


def _side_clearance(obstacles: list[TrackedObstacle], side: str) -> float:
    """Estimate clearance on a given side. Higher = more free space.

    Uses inverse-square weighting so close obstacles dominate the score
    and far obstacles barely matter — critical for tight-space navigation.
    An object at 0.5m contributes ~16x more penalty than one at 2m.
    """
    score = 10.0
    for o in obstacles:
        if o.distance > ZONE_SLOW:
            continue
        a = math.degrees(o.angle_rad)
        on_this_side = (side == "left" and a < 10) or (side == "right" and a > -10)
        if not on_this_side:
            continue
        weight = 1.0 / max(o.distance, 0.2) ** 2
        score -= weight
    return max(0.0, score)


# =============================================================================
# Main loop
# =============================================================================

async def run(shutdown_event: asyncio.Event) -> None:
    commander = MotorCommander()
    await commander.ensure_connected()

    if not YOLO_BLOB.exists():
        logger.error("YOLO blob not found: %s", YOLO_BLOB)
        return

    logger.info("Model: %s", YOLO_BLOB.name)
    pipeline = create_yolo_only_pipeline(
        YOLO_BLOB, enable_depth=True, confidence_threshold=CONFIDENCE_THRESHOLD,
    )

    usb_label = "USB2" if FORCE_USB2 else "USB3 (auto)"
    logger.info("Starting OAK-D pipeline (%s) …", usb_label)
    dev_kw = {"maxUsbSpeed": dai.UsbSpeed.HIGH} if FORCE_USB2 else {}

    with dai.Device(pipeline, **dev_kw) as device:
        logger.info("Connected — USB speed: %s", device.getUsbSpeed().name)
        q_det = device.getOutputQueue("detections", maxSize=4, blocking=False)
        q_depth = device.getOutputQueue("depth", maxSize=4, blocking=False)
        logger.info("Pipeline running — obstacle avoidance active")

        frame_count = 0
        t_start = time.time()
        t_last_status = t_start
        t_last_frame = t_start
        last_depth_frame = None

        last_action = "forward"
        last_reason = "startup"
        stop_start_time: float = 0.0
        stuck_start_time: float = 0.0
        turn_start_time: float = 0.0
        held_turn_action: str = ""

        while not shutdown_event.is_set():
            det_data = q_det.tryGet()
            depth_data = q_depth.tryGet()

            if depth_data is not None:
                last_depth_frame = depth_data.getFrame()

            if det_data is not None:
                frame_count += 1
                t_last_frame = time.time()

                raw = parse_img_detections(det_data)
                adjusted = [adjust_bbox_from_letterbox(d) for d in raw]

                if raw:
                    labels = [f"{d.class_label}({d.confidence:.2f})" for d in raw]
                    logger.debug("Raw: %s", ", ".join(labels))

                obstacles = build_obstacles(adjusted, last_depth_frame)

                if obstacles:
                    parts = []
                    for o in obstacles:
                        ad = math.degrees(o.angle_rad)
                        s = "L" if ad < 0 else "R" if ad > 0 else "C"
                        tag = "IN" if o.in_path else "out"
                        parts.append(f"{o.label}@{o.distance:.2f}m {s}{abs(ad):.0f}° {tag}")
                    logger.debug("Obstacles: %s", " | ".join(parts))

                action, reason = decide_action(obstacles, stop_start_time)

                # Enforce minimum turn duration — don't snap back to forward
                # too early, which causes the mower to clip the obstacle
                is_turn = action.startswith("turn_") or action.startswith("pivot_")
                if is_turn:
                    if turn_start_time == 0:
                        turn_start_time = time.time()
                    held_turn_action = action
                elif action in ("forward", "slow_down") and held_turn_action:
                    if time.time() - turn_start_time < MIN_TURN_HOLD_S:
                        action = held_turn_action
                        reason = f"holding turn ({MIN_TURN_HOLD_S}s min)"
                    else:
                        turn_start_time = 0.0
                        held_turn_action = ""
                else:
                    turn_start_time = 0.0
                    held_turn_action = ""

                # Track stop duration for escape pivots
                if action == "stop":
                    if stop_start_time == 0:
                        stop_start_time = time.time()
                else:
                    stop_start_time = 0.0

                # Track stuck duration — if not going forward for too long, 180
                if action == "forward":
                    stuck_start_time = 0.0
                else:
                    if stuck_start_time == 0:
                        stuck_start_time = time.time()
                    elif time.time() - stuck_start_time > STUCK_ESCAPE_TIMEOUT:
                        action = "pivot_180"
                        reason = f"stuck for {STUCK_ESCAPE_TIMEOUT:.0f}s — 180 escape"
                        stuck_start_time = 0.0
                        logger.info("STUCK ESCAPE: forcing 180 pivot")

                if action != last_action:
                    logger.info("ACTION: %s -> %s  (%s)", last_action, action, reason)
                    last_action = action
                    last_reason = reason

                # Map to motor command (forward -> "none" in commander)
                cmd = action if action != "forward" else "none"
                await commander.execute(cmd, speed_factor=1.0)

            # Watchdog
            if time.time() - t_last_frame > WATCHDOG_TIMEOUT_S:
                logger.warning("Watchdog: no frames for %.1fs", WATCHDOG_TIMEOUT_S)
                await commander.emergency_stop()
                t_last_frame = time.time()

            # Status
            now = time.time()
            if now - t_last_status >= STATUS_LOG_INTERVAL_S:
                elapsed = now - t_start
                fps = frame_count / elapsed if elapsed > 0 else 0.0
                logger.info(
                    "FPS=%.1f | Action=%s | %s", fps, last_action, last_reason,
                )
                t_last_status = now

            await asyncio.sleep(0.005)

    logger.info("Shutting down — stopping motors")
    await commander.emergency_stop()
    await commander.close()
    logger.info("Done")


def main() -> int:
    debug = "--debug" in sys.argv
    print("=" * 60)
    print("  OBSTACLE AVOIDANCE — Raspberry Pi (headless)")
    print("  Model: yolo-v4-tiny 12-shave")
    if debug:
        print("  ** DEBUG MODE **")
    print("=" * 60)

    if debug:
        logging.getLogger("obstacle_avoidance").setLevel(logging.DEBUG)

    shutdown_event = asyncio.Event()

    def _sig(sig, frame):
        logger.info("Signal %s — shutting down", sig)
        shutdown_event.set()

    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    try:
        asyncio.run(run(shutdown_event))
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
