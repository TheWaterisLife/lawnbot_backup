#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

import smbus  # apt: python3-smbus

MPU_ADDR_DEFAULT = 0x68
I2C_BUS_DEFAULT = 1

# Registers
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B  # burst: accel(6) + temp(2) + gyro(6)

ACCEL_LSB_PER_G = 16384.0
GYRO_LSB_PER_DPS = 131.0

def twos16(msb, lsb):
    v = (msb << 8) | lsb
    return v - 65536 if v & 0x8000 else v

class ImuNode(Node):
    def __init__(self):
        super().__init__("imu_node")

        self.declare_parameter("i2c_bus", I2C_BUS_DEFAULT)
        self.declare_parameter("i2c_addr", MPU_ADDR_DEFAULT)
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("init_retries", 10)
        self.declare_parameter("io_retries", 3)

        self.i2c_bus = int(self.get_parameter("i2c_bus").value)
        self.i2c_addr = int(self.get_parameter("i2c_addr").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.init_retries = int(self.get_parameter("init_retries").value)
        self.io_retries = int(self.get_parameter("io_retries").value)

        self.pub = self.create_publisher(Imu, "/imu/data", 10)

        self.bus = smbus.SMBus(self.i2c_bus)

        ok = self._init_mpu_with_retries()
        if not ok:
            self.get_logger().error("MPU6050 init failed after retries. Check noise/power/wiring.")
        else:
            self.get_logger().info(
                f"IMU started: /dev/i2c-{self.i2c_bus} addr=0x{self.i2c_addr:02X} rate={self.rate_hz}Hz"
            )

        self.last_t = time.time()
        self.timer = self.create_timer(1.0 / self.rate_hz, self._tick)

    def _write(self, reg, val):
        self.bus.write_byte_data(self.i2c_addr, reg, val)

    def _read_block(self, reg, n):
        return self.bus.read_i2c_block_data(self.i2c_addr, reg, n)

    def _write_retry(self, reg, val, retries=5, delay_s=0.05):
        for k in range(retries):
            try:
                self._write(reg, val)
                return True
            except OSError as e:
                self.get_logger().warn(f"I2C write reg 0x{reg:02X} failed ({e}), try {k+1}/{retries}")
                time.sleep(delay_s)
        return False

    def _init_mpu_with_retries(self) -> bool:
        for attempt in range(1, self.init_retries + 1):
            self.get_logger().info(f"Initializing MPU6050 attempt {attempt}/{self.init_retries}...")
            try:
                # Reset then wake
                if not self._write_retry(PWR_MGMT_1, 0x80, retries=3, delay_s=0.1):
                    raise OSError("reset write failed")
                time.sleep(0.2)

                if not self._write_retry(PWR_MGMT_1, 0x00, retries=3, delay_s=0.1):
                    raise OSError("wake write failed")
                time.sleep(0.2)

                # Config
                if not self._write_retry(CONFIG, 0x03, retries=3, delay_s=0.05):
                    raise OSError("CONFIG write failed")
                if not self._write_retry(SMPLRT_DIV, 0x04, retries=3, delay_s=0.05):
                    raise OSError("SMPLRT_DIV write failed")
                if not self._write_retry(GYRO_CONFIG, 0x00, retries=3, delay_s=0.05):
                    raise OSError("GYRO_CONFIG write failed")
                if not self._write_retry(ACCEL_CONFIG, 0x00, retries=3, delay_s=0.05):
                    raise OSError("ACCEL_CONFIG write failed")

                # quick read test
                _ = self._read_block(ACCEL_XOUT_H, 2)
                return True

            except Exception as e:
                self.get_logger().warn(f"Init attempt failed: {e}")
                time.sleep(0.3)
        return False

    def _tick(self):
        now = time.time()
        dt = now - self.last_t
        if dt <= 0:
            dt = 1.0 / self.rate_hz
        self.last_t = now

        raw = None
        for k in range(self.io_retries):
            try:
                raw = self._read_block(ACCEL_XOUT_H, 14)
                break
            except OSError as e:
                self.get_logger().error(f"I2C read failed ({e}) retry {k+1}/{self.io_retries}")
                time.sleep(0.02)

        if raw is None:
            return

        ax = twos16(raw[0], raw[1]) / ACCEL_LSB_PER_G
        ay = twos16(raw[2], raw[3]) / ACCEL_LSB_PER_G
        az = twos16(raw[4], raw[5]) / ACCEL_LSB_PER_G

        gx_dps = twos16(raw[8], raw[9]) / GYRO_LSB_PER_DPS
        gy_dps = twos16(raw[10], raw[11]) / GYRO_LSB_PER_DPS
        gz_dps = twos16(raw[12], raw[13]) / GYRO_LSB_PER_DPS

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.orientation_covariance[0] = -1.0

        msg.angular_velocity.x = math.radians(gx_dps)
        msg.angular_velocity.y = math.radians(gy_dps)
        msg.angular_velocity.z = math.radians(gz_dps)

        msg.linear_acceleration.x = ax * 9.80665
        msg.linear_acceleration.y = ay * 9.80665
        msg.linear_acceleration.z = az * 9.80665

        self.pub.publish(msg)

def main():
    rclpy.init()
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.bus.close()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()

