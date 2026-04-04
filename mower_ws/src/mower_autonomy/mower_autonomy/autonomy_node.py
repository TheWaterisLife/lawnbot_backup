#!/usr/bin/env python3
import math
import time
import asyncio
from dataclasses import dataclass
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Empty, String
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int32MultiArray

from mower_autonomy.map_loader import load_latest_map
from mower_autonomy.boundary_utils import sanitize_boundary
from mower_autonomy.geofence import Geofence
from mower_autonomy.coverage_planner import generate_stripes
from mower_autonomy.motor_ws_client import MotorWsClient, MotorWsConfig
from mower_autonomy.async_runner import AsyncRunner

XY = Tuple[float, float]


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


@dataclass
class AutonomyParams:
    # planning
    stripe_spacing: float = 0.18
    waypoint_spacing: float = 0.35

    # safety (fatal only if stale)
    pose_timeout_s: float = 0.5
    encoder_timeout_s: float = 0.5

    # geofence comfort margin
    boundary_margin_m: float = 0.15

    # control
    control_hz: float = 20.0
    waypoint_accept_radius_m: float = 0.25
    slow_zone_m: float = 0.25

    # motor command scales
    fwd_cmd: float = 0.14
    turn_cmd: float = 0.16

    # pivot threshold (rad)
    pivot_err_rad: float = 0.60

    # accuracy behavior at waypoint
    waypoint_dwell_s: float = 0.15

    # recovery
    recover_fwd_cmd: float = 0.12
    recover_turn_cmd: float = 0.18
    recover_accept_m: float = 0.30
    recover_push_in_m: float = 0.40

    # encoder 90 deg turn
    turn_ticks_90: int = 4285
    turn_ticks_tol: int = 120

    # websocket
    ws_host: str = "127.0.0.1"
    ws_port: int = 8766


class AutonomyNode(Node):
    def __init__(self):
        super().__init__("autonomy_node")

        # --------- Parameters ---------
        self.declare_parameter("stripe_spacing", 0.18)
        self.declare_parameter("waypoint_spacing", 0.35)

        self.declare_parameter("boundary_margin_m", 0.15)
        self.declare_parameter("pose_timeout_s", 0.5)
        self.declare_parameter("encoder_timeout_s", 0.5)

        self.declare_parameter("control_hz", 20.0)
        self.declare_parameter("waypoint_accept_radius_m", 0.25)
        self.declare_parameter("slow_zone_m", 0.25)

        self.declare_parameter("fwd_cmd", 0.14)
        self.declare_parameter("turn_cmd", 0.16)
        self.declare_parameter("pivot_err_rad", 0.60)

        self.declare_parameter("waypoint_dwell_s", 0.15)

        self.declare_parameter("recover_fwd_cmd", 0.12)
        self.declare_parameter("recover_turn_cmd", 0.18)
        self.declare_parameter("recover_accept_m", 0.30)
        self.declare_parameter("recover_push_in_m", 0.40)

        self.declare_parameter("turn_ticks_90", 8674)
        self.declare_parameter("turn_ticks_tol", 150)

        self.declare_parameter("ws_host", "127.0.0.1")
        self.declare_parameter("ws_port", 8766)

        # Debug / testing
        self.declare_parameter("dry_run", False)  # if true: compute commands but do not drive motors

        self.p = AutonomyParams(
            stripe_spacing=float(self.get_parameter("stripe_spacing").value),
            waypoint_spacing=float(self.get_parameter("waypoint_spacing").value),

            boundary_margin_m=float(self.get_parameter("boundary_margin_m").value),
            pose_timeout_s=float(self.get_parameter("pose_timeout_s").value),
            encoder_timeout_s=float(self.get_parameter("encoder_timeout_s").value),

            control_hz=float(self.get_parameter("control_hz").value),
            waypoint_accept_radius_m=float(self.get_parameter("waypoint_accept_radius_m").value),
            slow_zone_m=float(self.get_parameter("slow_zone_m").value),

            fwd_cmd=float(self.get_parameter("fwd_cmd").value),
            turn_cmd=float(self.get_parameter("turn_cmd").value),
            pivot_err_rad=float(self.get_parameter("pivot_err_rad").value),

            waypoint_dwell_s=float(self.get_parameter("waypoint_dwell_s").value),

            recover_fwd_cmd=float(self.get_parameter("recover_fwd_cmd").value),
            recover_turn_cmd=float(self.get_parameter("recover_turn_cmd").value),
            recover_accept_m=float(self.get_parameter("recover_accept_m").value),
            recover_push_in_m=float(self.get_parameter("recover_push_in_m").value),

            turn_ticks_90=int(self.get_parameter("turn_ticks_90").value),
            turn_ticks_tol=int(self.get_parameter("turn_ticks_tol").value),

            ws_host=str(self.get_parameter("ws_host").value),
            ws_port=int(self.get_parameter("ws_port").value),
        )

        self.dry_run = bool(self.get_parameter("dry_run").value)

        # --------- ROS I/O ---------
        self.pose_sub = self.create_subscription(Pose2D, "/mower/pose", self._on_pose, 10)
        self.enc_sub = self.create_subscription(Int32MultiArray, "/encoders/ticks", self._on_enc_array, 10)

        self.start_sub = self.create_subscription(Empty, "/autonomy/start", self._on_start, 10)
        self.stop_sub = self.create_subscription(Empty, "/autonomy/stop", self._on_stop, 10)

        self.state_pub = self.create_publisher(Bool, "/autonomy/running", 10)

        # Debug stream
        self.debug_pub = self.create_publisher(String, "/autonomy/debug", 10)
        self._last_debug_t = 0.0
        self._debug_period_s = 0.25

        # --------- State ---------
        self.pose: Optional[Pose2D] = None
        self.last_pose_t = 0.0

        self.left_ticks: Optional[int] = None
        self.right_ticks: Optional[int] = None
        self.last_enc_t = 0.0

        self.running = False

        self.map = None
        self.gf: Optional[Geofence] = None
        self.stripes: List[List[XY]] = []
        self.stripe_i = 0
        self.wp_i = 0

        self.mode = "IDLE"   # IDLE, FOLLOW, TURN, RECOVER, DONE, STOPPED
        self.turn_ref_L: Optional[int] = None
        self.turn_ref_R: Optional[int] = None

        # waypoint dwell
        self._dwell_until_t: float = 0.0

        # WS client + async runner
        self.ws = MotorWsClient(MotorWsConfig(host=self.p.ws_host, port=self.p.ws_port))
        self.runner = AsyncRunner()
        self.runner.start()

        # log throttle
        self._last_oob_log_t = 0.0

        # timer
        self.timer = self.create_timer(1.0 / self.p.control_hz, self._tick_ros)

        self.get_logger().info(
            "autonomy_node ready. Publish /autonomy/start (Empty) to begin. "
            f"(dry_run={self.dry_run})"
        )

    # ---------- Debug helper ----------

    def _pub_debug(self, text: str):
        now = time.time()
        if (now - self._last_debug_t) < self._debug_period_s:
            return
        self._last_debug_t = now
        msg = String()
        msg.data = text
        self.debug_pub.publish(msg)

    # --------- Callbacks ---------

    def _on_pose(self, msg: Pose2D):
        self.pose = msg
        self.last_pose_t = time.time()

    def _on_enc_array(self, msg: Int32MultiArray):
        if len(msg.data) < 2:
            return
        self.left_ticks = int(msg.data[0])
        self.right_ticks = int(msg.data[1])
        self.last_enc_t = time.time()

    def _on_start(self, msg: Empty):
        if self.running:
            return
        self.get_logger().warn("START received.")
        self.running = True
        self.mode = "IDLE"
        self.runner.submit(self._start_autonomy())

    def _on_stop(self, msg: Empty):
        self.get_logger().warn("STOP received.")
        self.runner.submit(self._hard_stop("App STOP"))

    # --------- Safety helpers ---------

    def _now(self) -> float:
        return time.time()

    def _have_fresh_pose(self) -> bool:
        return (self.pose is not None) and ((self._now() - self.last_pose_t) <= self.p.pose_timeout_s)

    def _have_fresh_enc(self) -> bool:
        return (self.left_ticks is not None) and (self.right_ticks is not None) and (
            (self._now() - self.last_enc_t) <= self.p.encoder_timeout_s
        )

    def _fatal_reason(self) -> Optional[str]:
        if not self._have_fresh_pose():
            return "pose stale"
        if not self._have_fresh_enc():
            return "encoders stale"
        return None

    async def _hard_stop(self, reason: str):
        # always try to stop motors + blade (best effort)
        try:
            self.get_logger().error(f"HARD STOP: {reason}")
        except Exception:
            pass

        self.running = False
        self.mode = "STOPPED"

        # Best effort: if connected, stop immediately; otherwise try a short reconnect
        try:
            if not self.ws.is_connected():
                await asyncio.wait_for(self.ws.connect(), timeout=1.0)
            if self.ws.is_connected():
                await self.ws.stop()
                await self.ws.blade(0.0)
        except Exception:
            pass

        try:
            self.state_pub.publish(Bool(data=False))
        except Exception:
            pass

    def _failsafe_stop_sync(self, reason: str):
        """
        Best-effort stop during shutdown / exceptions.
        Schedules _hard_stop on the background asyncio runner.
        """
        try:
            self.get_logger().error(f"FAILSAFE STOP: {reason}")
        except Exception:
            pass
        try:
            if hasattr(self, "runner") and self.runner is not None:
                self.runner.submit(self._hard_stop(reason))
        except Exception:
            pass

    # --------- Autonomy init ---------

    async def _start_autonomy(self):
        self.get_logger().info("Starting autonomy init...")

        # Require WS connection first (so we can stop if needed)
        try:
            await self.ws.ensure_connected()
        except Exception as e:
            await self._hard_stop(f"WS connect failed: {e}")
            return

        fatal = self._fatal_reason()
        if fatal is not None:
            await self._hard_stop(f"Missing data at start: {fatal}")
            return

        # Load latest map
        try:
            self.map = load_latest_map()
            self.get_logger().info(f"Loaded latest map file: {self.map.path}")
        except Exception as e:
            await self._hard_stop(f"Map load failed: {e}")
            return

        # Sanitize boundary (tolerant to messy user map)
        try:
            boundary = sanitize_boundary(self.map.boundary_xy, close_tol=0.20, dedup_tol=0.03)
        except Exception as e:
            await self._hard_stop(f"Boundary sanitize failed: {e}")
            return

        # Build tolerant geofence
        try:
            self.gf = Geofence.from_boundary(boundary, boundary_margin_m=self.p.boundary_margin_m)
            if self.gf.margin_m == 0.0:
                self.get_logger().warn("Geofence margin too strict -> using 0.0m margin fallback.")
        except Exception as e:
            await self._hard_stop(f"Geofence invalid: {e}")
            return

        # Build stripes
        try:
            self.stripes = generate_stripes(
                self.gf.boundary_xy,
                stripe_spacing=self.p.stripe_spacing,
                waypoint_spacing=self.p.waypoint_spacing,
                boundary_margin_m=self.gf.margin_m,
            )
        except Exception as e:
            await self._hard_stop(f"Stripe generation failed: {e}")
            return

        if not self.stripes:
            await self._hard_stop("No stripes generated")
            return

        self.stripe_i = 0
        self.wp_i = 0
        self.mode = "FOLLOW"
        self._dwell_until_t = 0.0

        # start blade (unless dry-run)
        if not self.dry_run:
            await self.ws.blade(1.0)

        self.state_pub.publish(Bool(data=True))
        self.get_logger().info(f"Autonomy started. stripes={len(self.stripes)}")

    # --------- Control loop ---------

    def _tick_ros(self):
        if not self.running:
            return
        self.runner.submit(self._tick_async())

    async def _tick_async(self):
        if not self.running:
            return

        if self.mode in ("IDLE", "STOPPED"):
            return

        fatal = self._fatal_reason()
        if fatal is not None:
            await self._hard_stop(f"Fatal safety: {fatal}")
            return

        # Outside polygon -> RECOVER (not fatal)
        if self.gf is not None and self.pose is not None:
            if not self.gf.inside(self.pose.x, self.pose.y):
                now = time.time()
                if self.mode != "RECOVER" and (now - self._last_oob_log_t) > 1.0:
                    self.get_logger().warn("Outside boundary -> RECOVER")
                    self._last_oob_log_t = now
                self.mode = "RECOVER"

        if self.mode == "FOLLOW":
            await self._do_follow()
        elif self.mode == "TURN":
            await self._do_turn()
        elif self.mode == "RECOVER":
            await self._do_recover()
        elif self.mode == "DONE":
            try:
                if not self.dry_run:
                    await self.ws.stop()
                    await self.ws.blade(0.0)
            except Exception:
                pass
            self.running = False
            self.state_pub.publish(Bool(data=False))
            self.get_logger().info("Autonomy DONE.")
        else:
            await self._hard_stop(f"Unknown mode: {self.mode}")

    async def _do_follow(self):
        assert self.pose is not None

        # waypoint dwell window (accuracy)
        if self._dwell_until_t > 0.0 and time.time() < self._dwell_until_t:
            if not self.dry_run:
                await self.ws.stop()
            return
        self._dwell_until_t = 0.0

        if self.stripe_i >= len(self.stripes):
            self.mode = "DONE"
            return

        stripe = self.stripes[self.stripe_i]

        # end of stripe -> TURN
        if self.wp_i >= len(stripe):
            if self.stripe_i >= len(self.stripes) - 1:
                self.mode = "DONE"
                return
            if not self.dry_run:
                await self.ws.stop()
            await asyncio.sleep(0.05)
            self.mode = "TURN"
            self._turn_capture_refs()
            return

        tx, ty = stripe[self.wp_i]
        dx = tx - self.pose.x
        dy = ty - self.pose.y
        dist = math.hypot(dx, dy)

        # accept waypoint -> STOP + dwell
        if dist <= self.p.waypoint_accept_radius_m:
            self.wp_i += 1
            if not self.dry_run:
                await self.ws.stop()
            self._dwell_until_t = time.time() + max(0.0, self.p.waypoint_dwell_s)
            return

        desired = math.atan2(dy, dx)
        err = wrap_pi(desired - self.pose.theta)

        scale = 1.0
        if dist < self.p.slow_zone_m:
            scale = max(0.35, dist / self.p.slow_zone_m)

        # simple steering
        turn = max(-1.0, min(1.0, 0.9 * err)) * self.p.turn_cmd
        fwd = self.p.fwd_cmd * scale

        # pivot if big heading error
        if abs(err) > self.p.pivot_err_rad:
            left = max(-1.0, min(1.0, -turn))
            right = max(-1.0, min(1.0, +turn))
        else:
            left = max(-1.0, min(1.0, fwd - turn))
            right = max(-1.0, min(1.0, fwd + turn))

        self._pub_debug(
            f"mode=FOLLOW stripe={self.stripe_i}/{len(self.stripes)} wp={self.wp_i}/{len(stripe)} "
            f"pose=({self.pose.x:.2f},{self.pose.y:.2f},{self.pose.theta:.2f}) "
            f"target=({tx:.2f},{ty:.2f}) dist={dist:.2f} err={err:.2f} "
            f"cmd=({left:.2f},{right:.2f}) dry_run={self.dry_run}"
        )

        if self.dry_run:
            if self.ws.is_connected():
                await self.ws.stop()
            return

        await self.ws.drive(left, right)

    def _turn_capture_refs(self):
        if self.left_ticks is None or self.right_ticks is None:
            self.turn_ref_L = None
            self.turn_ref_R = None
            return
        self.turn_ref_L = int(self.left_ticks)
        self.turn_ref_R = int(self.right_ticks)
        self.get_logger().info(f"TURN start refs L={self.turn_ref_L} R={self.turn_ref_R}")

    async def _do_turn(self):
        if self.turn_ref_L is None or self.turn_ref_R is None:
            await self._hard_stop("Missing turn encoder refs")
            return
        if self.left_ticks is None or self.right_ticks is None:
            await self._hard_stop("Missing live encoder ticks")
            return

        dL = int(self.left_ticks) - int(self.turn_ref_L)
        dR = int(self.right_ticks) - int(self.turn_ref_R)

        target = int(self.p.turn_ticks_90)
        tol = int(self.p.turn_ticks_tol)

        # Sign-robust: accept magnitude reached on both sides
        okL = (abs(dL) >= (target - tol))
        okR = (abs(dR) >= (target - tol))

        self._pub_debug(
            f"mode=TURN dL={dL} dR={dR} target={target} tol={tol} "
            f"cmd=({+self.p.turn_cmd:.2f},{-self.p.turn_cmd:.2f}) dry_run={self.dry_run}"
        )

        if okL and okR:
            if not self.dry_run:
                await self.ws.stop()
            await asyncio.sleep(0.05)
            self.stripe_i += 1
            self.wp_i = 0
            self.mode = "FOLLOW"
            self._dwell_until_t = time.time() + max(0.0, self.p.waypoint_dwell_s)
            self.get_logger().info(f"TURN done. Next stripe {self.stripe_i+1}/{len(self.stripes)}")
            return

        if self.dry_run:
            if self.ws.is_connected():
                await self.ws.stop()
            return

        # Pivot turn (both tracks active)
        await self.ws.drive(+self.p.turn_cmd, -self.p.turn_cmd)

    async def _do_recover(self):
        assert self.pose is not None
        assert self.gf is not None

        # Back inside -> resume FOLLOW
        if self.gf.inside(self.pose.x, self.pose.y):
            self.get_logger().info("Recovered inside boundary -> FOLLOW")
            self.mode = "FOLLOW"
            if not self.dry_run:
                await self.ws.stop()
            self._dwell_until_t = time.time() + max(0.0, self.p.waypoint_dwell_s)
            return

        # nearest point on polygon boundary
        bx, by = self.gf.nearest_boundary_point(self.pose.x, self.pose.y)

        # aim a bit inside toward centroid
        cx, cy = float(self.gf.polygon.centroid.x), float(self.gf.polygon.centroid.y)
        vx = cx - bx
        vy = cy - by
        vnorm = math.hypot(vx, vy)

        if vnorm < 1e-6:
            tx, ty = bx, by
        else:
            tx = bx + (vx / vnorm) * self.p.recover_push_in_m
            ty = by + (vy / vnorm) * self.p.recover_push_in_m

        dx = tx - self.pose.x
        dy = ty - self.pose.y
        dist = math.hypot(dx, dy)

        desired = math.atan2(dy, dx)
        err = wrap_pi(desired - self.pose.theta)

        turn = max(-1.0, min(1.0, 1.3 * err)) * self.p.recover_turn_cmd
        fwd = self.p.recover_fwd_cmd

        if abs(err) > self.p.pivot_err_rad:
            left = max(-1.0, min(1.0, -turn))
            right = max(-1.0, min(1.0, +turn))
        else:
            if dist < self.p.recover_accept_m:
                fwd *= 0.4
            left = max(-1.0, min(1.0, fwd - turn))
            right = max(-1.0, min(1.0, fwd + turn))

        self._pub_debug(
            f"mode=RECOVER pose=({self.pose.x:.2f},{self.pose.y:.2f},{self.pose.theta:.2f}) "
            f"push_target=({tx:.2f},{ty:.2f}) dist={dist:.2f} err={err:.2f} "
            f"cmd=({left:.2f},{right:.2f}) dry_run={self.dry_run}"
        )

        if self.dry_run:
            if self.ws.is_connected():
                await self.ws.stop()
            return

        await self.ws.drive(left, right)

    def destroy_node(self):
        # If the node is destroyed while running, stop motors (best effort)
        if getattr(self, "running", False):
            self._failsafe_stop_sync("Node destroy/shutdown")
            time.sleep(0.15)

        try:
            if hasattr(self, "runner") and self.runner is not None:
                self.runner.stop()
        except Exception:
            pass

        super().destroy_node()


def main():
    rclpy.init()
    node = AutonomyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        # if something crashes, try to stop
        try:
            node._failsafe_stop_sync(f"Exception: {e}")
        except Exception:
            pass
        raise
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

