#!/usr/bin/env python3
import time
import socket

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def get_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "unknown"

class FakeTelemetry(Node):
    def __init__(self):
        super().__init__("fake_telemetry")
        self.pub_state = self.create_publisher(String, "/mower/state", 10)
        self.pub_tel   = self.create_publisher(String, "/mower/telemetry", 10)
        self.timer = self.create_timer(0.2, self.tick)  # 5 Hz

    def tick(self):
        now = time.time()
        ip = get_ip()

        state = String()
        state.data = f"mode=TELEOP armed=false estop=false t={now:.2f}"
        self.pub_state.publish(state)

        tel = String()
        tel.data = f"wifi_ip={ip} t={now:.2f}"
        self.pub_tel.publish(tel)

def main():
    rclpy.init()
    node = FakeTelemetry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
