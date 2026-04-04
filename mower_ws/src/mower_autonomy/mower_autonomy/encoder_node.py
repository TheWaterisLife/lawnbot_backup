#!/usr/bin/env python3
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

from gpiozero import DigitalInputDevice


@dataclass
class Pins:
    a: int
    b: int


class QuadGPIOZero:
    """
    Quadrature decoder using gpiozero edge callbacks on channel A.
    Direction from A vs B state (same logic as your working script).

    This is x2-ish (counts A edges). For your mower this is fine.
    If you want x4 later, we can add edges on B too.
    """
    def __init__(self, pins: Pins, pull_up: bool = True, invert_dir: bool = False):
        self.A = DigitalInputDevice(pins.a, pull_up=pull_up)
        self.B = DigitalInputDevice(pins.b, pull_up=pull_up)

        self.ticks = 0
        self._delta = 0
        self._last_a = self.A.value
        self.invert_dir = invert_dir

        # attach callbacks for A rising + falling
        self.A.when_activated = self._on_a_change
        self.A.when_deactivated = self._on_a_change

    def _on_a_change(self):
        a = self.A.value
        b = self.B.value
        if a == self._last_a:
            return
        self._last_a = a

        # same logic as your test (may need flip depending wiring)
        step = -1 if (a == b) else +1
        if self.invert_dir:
            step = -step

        self.ticks += step
        self._delta += step

    def read_and_reset_delta(self) -> int:
        d = self._delta
        self._delta = 0
        return d

    def close(self):
        try:
            self.A.close()
        except Exception:
            pass
        try:
            self.B.close()
        except Exception:
            pass


class EncoderNode(Node):
    def __init__(self):
        super().__init__("encoder_node")

        # your BCM pins
        self.declare_parameter("ra", 17)
        self.declare_parameter("rb", 22)
        self.declare_parameter("la", 25)
        self.declare_parameter("lb", 27)

        self.declare_parameter("publish_hz", 50.0)

        # flip direction if needed
        self.declare_parameter("invert_left", False)
        self.declare_parameter("invert_right", False)

        self.ra = int(self.get_parameter("ra").value)
        self.rb = int(self.get_parameter("rb").value)
        self.la = int(self.get_parameter("la").value)
        self.lb = int(self.get_parameter("lb").value)

        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.invert_left = bool(self.get_parameter("invert_left").value)
        self.invert_right = bool(self.get_parameter("invert_right").value)

        self.pub_ticks = self.create_publisher(Int32MultiArray, "/encoders/ticks", 10)
        self.pub_delta = self.create_publisher(Int32MultiArray, "/encoders/delta", 10)

        self.left = QuadGPIOZero(Pins(self.la, self.lb), pull_up=True, invert_dir=self.invert_left)
        self.right = QuadGPIOZero(Pins(self.ra, self.rb), pull_up=True, invert_dir=self.invert_right)

        self.get_logger().info(
            f"encoder_node (gpiozero) started. L(A,B)=({self.la},{self.lb}) R(A,B)=({self.ra},{self.rb}) "
            f"publish={self.publish_hz}Hz"
        )

        self.timer = self.create_timer(1.0 / max(1e-6, self.publish_hz), self._publish)

    def _publish(self):
        lt = int(self.left.ticks)
        rt = int(self.right.ticks)
        ld = int(self.left.read_and_reset_delta())
        rd = int(self.right.read_and_reset_delta())

        msg_t = Int32MultiArray()
        msg_t.data = [lt, rt]
        self.pub_ticks.publish(msg_t)

        msg_d = Int32MultiArray()
        msg_d.data = [ld, rd]
        self.pub_delta.publish(msg_d)

    def destroy_node(self):
        try:
            self.left.close()
            self.right.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = EncoderNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
