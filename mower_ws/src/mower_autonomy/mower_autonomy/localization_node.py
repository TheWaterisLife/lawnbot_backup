#!/usr/bin/env python3
import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import NavSatFix, NavSatStatus

from mower_autonomy.map_loader import load_latest_map


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def deg2rad(d: float) -> float:
    return d * math.pi / 180.0


def latlon_to_local_xy_m(lat: float, lon: float, lat0: float, lon0: float) -> Tuple[float, float]:
    # Equirectangular approximation (good for small lawns)
    R = 6371000.0
    x = deg2rad(lon - lon0) * R * math.cos(deg2rad(lat0))
    y = deg2rad(lat - lat0) * R
    return x, y


@dataclass
class Params:
    track_center_m: float = 0.3725
    meters_per_tick: float = 6.83e-5
    publish_hz: float = 20.0

    gps_timeout_s: float = 1.0
    gps_warn_period_s: float = 2.0

    # GPS usage policy
    use_fix_only: bool = True

    # Complementary filter gain (0.0 = ignore GPS, 1.0 = snap to GPS)
    gps_correction_gain: float = 0.20

    # Reject GPS jumps larger than this (meters)
    max_gps_jump_m: float = 1.50

    # If we have no GPS for a while, do we still publish odom? (Yes)
    publish_without_gps: bool = True


class LocalizationNode(Node):
    """
    Publishes:
      /mower/pose  geometry_msgs/Pose2D  (x,y,theta)

    Fusion approach (no IMU):
      - Encoders drive continuous odometry: x_odom, y_odom, theta
      - GPS FIX-only provides absolute correction (complementary filter):
            x_odom += k*(x_gps - x_odom)
            y_odom += k*(y_gps - y_odom)

    This gives smoother, more stable navigation than "GPS overwrite x/y".
    """

    def __init__(self):
        super().__init__("localization_node")

        # --- parameters ---
        self.declare_parameter("track_center_m", 0.3725)
        self.declare_parameter("meters_per_tick", 3.37e-5)
        self.declare_parameter("publish_hz", 20.0)

        self.declare_parameter("gps_timeout_s", 1.0)
        self.declare_parameter("gps_warn_period_s", 2.0)

        self.declare_parameter("use_fix_only", True)
        self.declare_parameter("gps_correction_gain", 0.20)
        self.declare_parameter("max_gps_jump_m", 1.50)
        self.declare_parameter("publish_without_gps", True)

        self.p = Params(
            track_center_m=float(self.get_parameter("track_center_m").value),
            meters_per_tick=float(self.get_parameter("meters_per_tick").value),
            publish_hz=float(self.get_parameter("publish_hz").value),

            gps_timeout_s=float(self.get_parameter("gps_timeout_s").value),
            gps_warn_period_s=float(self.get_parameter("gps_warn_period_s").value),

            use_fix_only=bool(self.get_parameter("use_fix_only").value),
            gps_correction_gain=float(self.get_parameter("gps_correction_gain").value),
            max_gps_jump_m=float(self.get_parameter("max_gps_jump_m").value),
            publish_without_gps=bool(self.get_parameter("publish_without_gps").value),
        )

        # clamp gain
        if self.p.gps_correction_gain < 0.0:
            self.p.gps_correction_gain = 0.0
        if self.p.gps_correction_gain > 1.0:
            self.p.gps_correction_gain = 1.0

        # subs
        self.sub_gps = self.create_subscription(NavSatFix, "/rtk/fix", self._on_gps, 10)
        self.sub_ticks = self.create_subscription(Int32MultiArray, "/encoders/ticks", self._on_ticks, 10)

        # pub
        self.pub_pose = self.create_publisher(Pose2D, "/mower/pose", 10)

        # origin
        self.origin_latlon: Optional[Tuple[float, float]] = None
        self.have_origin = False

        # odom state (ENCODER integrated)
        self.x_odom = 0.0
        self.y_odom = 0.0
        self.theta = 0.0

        # last encoder ticks
        self.last_left: Optional[int] = None
        self.last_right: Optional[int] = None

        # gps state
        self.last_gps_t = 0.0
        self._last_gps_warn_t = 0.0
        self.gps_valid = False
        self.x_gps = 0.0
        self.y_gps = 0.0

        # --- Load latest map origin (preferred) ---
        try:
            m = load_latest_map()
            self.origin_latlon = (float(m.origin_latlon[0]), float(m.origin_latlon[1]))
            self.have_origin = True
            self.get_logger().info(
                f"Using map origin: lat0={self.origin_latlon[0]} lon0={self.origin_latlon[1]} (map={m.path})"
            )
        except Exception as e:
            self.get_logger().warn(
                f"No valid latest map origin. Will set origin from first GPS FIX. ({e})"
            )

        self.timer = self.create_timer(1.0 / max(1e-6, self.p.publish_hz), self._tick)

        self.get_logger().info(
            "localization_node started (ENC odom + GPS correction). "
            f"L={self.p.track_center_m} m, meters_per_tick={self.p.meters_per_tick}, "
            f"gps_gain={self.p.gps_correction_gain}, fix_only={self.p.use_fix_only}, "
            f"max_gps_jump_m={self.p.max_gps_jump_m}"
        )

    # ----------------- helpers -----------------

    def _gps_is_fresh(self) -> bool:
        return (time.time() - self.last_gps_t) <= self.p.gps_timeout_s

    def _gps_msg_is_valid(self, msg: NavSatFix) -> bool:
        # Reject NaNs
        lat = float(msg.latitude)
        lon = float(msg.longitude)
        if math.isnan(lat) or math.isnan(lon):
            return False

        # Reject obvious bogus
        if abs(lat) < 1e-12 and abs(lon) < 1e-12:
            return False

        # FIX-only logic
        st = msg.status.status if hasattr(msg, "status") else NavSatStatus.STATUS_NO_FIX
        if self.p.use_fix_only:
            # Many stacks use:
            #  -1 = NO_FIX
            #   0 = FIX
            #   1 = SBAS
            #   2 = GBAS
            # Your custom may use 4/5; we handle that too:
            if st in (NavSatStatus.STATUS_NO_FIX,):
                return False
            # If your rtk_reader encodes FIX/FLOAT as 4/5, then accept only 4:
            # We'll treat 5 explicitly as "reject".
            if st == 5:
                return False
            # If it is 4 -> accept. If it is 0/1/2 -> accept.
        else:
            if st == NavSatStatus.STATUS_NO_FIX:
                return False

        return True

    # ----------------- callbacks -----------------

    def _on_gps(self, msg: NavSatFix):
        if not self._gps_msg_is_valid(msg):
            return

        now = time.time()
        self.last_gps_t = now

        # If no origin yet, set origin from first VALID GPS (FIX)
        if self.origin_latlon is None:
            self.origin_latlon = (float(msg.latitude), float(msg.longitude))
            self.have_origin = True
            self.get_logger().info(
                f"GPS origin set: lat0={self.origin_latlon[0]} lon0={self.origin_latlon[1]}"
            )
            # initialize odom to gps (so we start aligned)
            self.x_odom = 0.0
            self.y_odom = 0.0

        if self.origin_latlon is None:
            return

        # Compute GPS local XY (same frame as map)
        x, y = latlon_to_local_xy_m(
            float(msg.latitude), float(msg.longitude),
            self.origin_latlon[0], self.origin_latlon[1]
        )

        # Jump reject (meters)
        if self.gps_valid:
            dx = float(x) - self.x_gps
            dy = float(y) - self.y_gps
            jump = math.hypot(dx, dy)
            if jump > self.p.max_gps_jump_m:
                self.get_logger().warn(f"GPS jump rejected: {jump:.2f} m > {self.p.max_gps_jump_m:.2f} m")
                return

        self.x_gps = float(x)
        self.y_gps = float(y)
        self.gps_valid = True

        # Apply correction immediately when GPS arrives
        k = float(self.p.gps_correction_gain)
        if k > 0.0:
            self.x_odom += k * (self.x_gps - self.x_odom)
            self.y_odom += k * (self.y_gps - self.y_odom)

    def _on_ticks(self, msg: Int32MultiArray):
        if len(msg.data) < 2:
            return
        left = int(msg.data[0])
        right = int(msg.data[1])

        if self.last_left is None or self.last_right is None:
            self.last_left = left
            self.last_right = right
            return

        dL_ticks = left - self.last_left
        dR_ticks = right - self.last_right
        self.last_left = left
        self.last_right = right

        dL = dL_ticks * self.p.meters_per_tick
        dR = dR_ticks * self.p.meters_per_tick

        # heading from differential
        dtheta = (dL - dR) / self.p.track_center_m
        self.theta = wrap_pi(self.theta + dtheta)

        # forward distance
        ds = 0.5 * (dL + dR)

        # integrate odom position
        self.x_odom += float(ds * math.cos(self.theta))
        self.y_odom += float(ds * math.sin(self.theta))

    # ----------------- publish loop -----------------

    def _tick(self):
        # If we have never had GPS and we don't want to publish without it, hold
        if (not self.gps_valid) and (not self.p.publish_without_gps):
            return

        pose = Pose2D()
        pose.x = float(self.x_odom)
        pose.y = float(self.y_odom)
        pose.theta = float(self.theta)
        self.pub_pose.publish(pose)

        # GPS stale warnings (throttled)
        if self.have_origin and (not self._gps_is_fresh()):
            now = time.time()
            if (now - self._last_gps_warn_t) > self.p.gps_warn_period_s:
                self.get_logger().warn("GPS stale: publishing pure encoder odom (x/y drift will accumulate).")
                self._last_gps_warn_t = now


def main():
    rclpy.init()
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

