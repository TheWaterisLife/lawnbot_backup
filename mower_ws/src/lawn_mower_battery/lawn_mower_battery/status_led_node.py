#!/usr/bin/env python3
"""
Status LED controller for GPA0-GPA4 on the MCP23017 GPIO expander.

Uses smbus2 direct register writes to avoid Adafruit pin glitching.

Manages the non-battery indicator LEDs (active-low: 0=ON, 1=OFF):
  GPA0 (bit 0) → GREEN  : Power (always ON while node runs)
  GPA1 (bit 1) → RED    : Blade / Autonomy running
  GPA2 (bit 2) → RED    : Fault (battery dead or overvoltage)
  GPA3 (bit 3) → AMBER  : GNSS/RTK fix quality
  GPA4 (bit 4) → BLUE   : App/comms connected
"""
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import BatteryState, NavSatFix, NavSatStatus
import smbus2

# MCP23017 registers (BANK=0 default)
_IODIRA = 0x00
_OLATA  = 0x14

BIT_POWER = 0   # GPA0
BIT_BLADE = 1   # GPA1
BIT_FAULT = 2   # GPA2
BIT_GNSS  = 3   # GPA3
BIT_COMMS = 4   # GPA4
_LED_MASK = 0x1F  # bits 0-4

RTK_TIMEOUT_S = 3.0


class StatusLedNode(Node):
    def __init__(self):
        super().__init__("status_led_node")

        self.declare_parameter("gpio_extender_address", 0x27)
        self.declare_parameter("update_rate", 5.0)

        self.mcp_addr = self.get_parameter("gpio_extender_address").value
        rate = self.get_parameter("update_rate").value

        # State tracking
        self._blade_on = False
        self._fault = False
        self._rtk_fix = False
        self._last_rtk_time = 0.0
        self._comms_connected = False

        # Hardware init via smbus2
        self.smb = None
        try:
            self.smb = smbus2.SMBus(1)
            iodira = self.smb.read_byte_data(self.mcp_addr, _IODIRA)
            self.smb.write_byte_data(self.mcp_addr, _IODIRA, iodira & ~_LED_MASK)
            # Power LED ON (bit 0 LOW), rest OFF (bits 1-4 HIGH)
            olata = self.smb.read_byte_data(self.mcp_addr, _OLATA)
            olata |= _LED_MASK       # all 5 HIGH (OFF)
            olata &= ~(1 << BIT_POWER)  # power LOW (ON)
            self.smb.write_byte_data(self.mcp_addr, _OLATA, olata)
            self.get_logger().info(
                f"MCP23017 status LEDs ready at {hex(self.mcp_addr)} (smbus2)"
            )
        except Exception as e:
            self.get_logger().error(f"MCP23017 not available: {e}")
            return

        # Subscriptions
        self.create_subscription(Bool, "/autonomy/running", self._on_autonomy, 10)
        self.create_subscription(BatteryState, "/battery", self._on_battery, 10)
        self.create_subscription(NavSatFix, "/rtk/fix", self._on_rtk, 10)
        self.create_subscription(Bool, "/app/connected", self._on_connected, 10)

        self.timer = self.create_timer(1.0 / rate, self._update)

    # ── Callbacks ───────────────────────────────────────────────

    def _on_autonomy(self, msg: Bool):
        self._blade_on = msg.data

    def _on_battery(self, msg: BatteryState):
        self._fault = msg.power_supply_health in (
            BatteryState.POWER_SUPPLY_HEALTH_DEAD,
            BatteryState.POWER_SUPPLY_HEALTH_OVERVOLTAGE,
        )

    def _on_rtk(self, msg: NavSatFix):
        self._last_rtk_time = time.time()
        self._rtk_fix = msg.status.status in (
            NavSatStatus.STATUS_FIX,
            NavSatStatus.STATUS_GBAS_FIX,
        )

    def _on_connected(self, msg: Bool):
        self._comms_connected = msg.data

    # ── Periodic LED update ─────────────────────────────────────

    def _update(self):
        if self.smb is None:
            return

        now = time.time()

        try:
            olata = self.smb.read_byte_data(self.mcp_addr, _OLATA)

            # Preserve bits outside our mask
            olata |= _LED_MASK  # start all 5 OFF (HIGH)

            # Power always ON
            olata &= ~(1 << BIT_POWER)

            if self._blade_on:
                olata &= ~(1 << BIT_BLADE)

            if self._fault:
                olata &= ~(1 << BIT_FAULT)

            rtk_stale = (now - self._last_rtk_time) > RTK_TIMEOUT_S
            if self._rtk_fix and not rtk_stale:
                olata &= ~(1 << BIT_GNSS)

            if self._comms_connected:
                olata &= ~(1 << BIT_COMMS)

            self.smb.write_byte_data(self.mcp_addr, _OLATA, olata)
        except Exception as e:
            self.get_logger().warn(f"LED update error: {e}")

    # ── Cleanup ─────────────────────────────────────────────────

    def destroy_node(self):
        if self.smb is not None:
            try:
                olata = self.smb.read_byte_data(self.mcp_addr, _OLATA)
                olata |= _LED_MASK  # all OFF
                self.smb.write_byte_data(self.mcp_addr, _OLATA, olata)
                self.smb.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StatusLedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
