#!/usr/bin/env python3

import time
import base64
import socket
import threading

import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus


def nmea_checksum_ok(sentence: str) -> bool:
    try:
        if not sentence.startswith('$') or '*' not in sentence:
            return False
        data, chk = sentence[1:].split('*', 1)
        calc = 0
        for c in data:
            calc ^= ord(c)
        return int(chk.strip()[:2], 16) == calc
    except Exception:
        return False


def dm_to_deg(dm: str, hemi: str) -> float:
    if not dm or not hemi:
        raise ValueError("empty dm/hemi")
    dot = dm.find('.')
    if dot == -1:
        raise ValueError("no dot")
    minutes_start = dot - 2
    deg_part = dm[:minutes_start]
    min_part = dm[minutes_start:]
    deg = float(deg_part)
    minutes = float(min_part)
    val = deg + minutes / 60.0
    if hemi in ('S', 'W'):
        val = -val
    return val


def gga_quality_to_text(q: int) -> str:
    # 0=invalid, 1=GNSS, 2=DGPS, 4=RTK FIX, 5=RTK FLOAT
    return {0: "INVALID", 1: "GNSS", 2: "DGPS", 4: "RTK_FIXED", 5: "RTK_FLOAT"}.get(q, f"Q{q}")


def parse_gga(line: str):
    if not (line.startswith('$') and 'GGA' in line):
        return None
    if '*' in line and not nmea_checksum_ok(line):
        return None

    core = line[1:]
    if '*' in core:
        core = core.split('*', 1)[0]
    parts = core.split(',')

    if len(parts) < 10 or not parts[0].endswith('GGA'):
        return None

    lat_dm = parts[2]
    lat_hemi = parts[3]
    lon_dm = parts[4]
    lon_hemi = parts[5]
    qual = parts[6]
    alt = parts[9]

    if not lat_dm or not lon_dm or not qual:
        return None

    q = int(qual) if qual.isdigit() else 0
    try:
        lat = dm_to_deg(lat_dm, lat_hemi)
        lon = dm_to_deg(lon_dm, lon_hemi)
    except Exception:
        return None

    alt_m = float(alt) if alt else float('nan')
    return lat, lon, alt_m, q


class RTKReader(Node):
    def __init__(self):
        super().__init__('rtk_reader')

        # Serial to F9
        self.declare_parameter('serial_port', '/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00')

        self.declare_parameter('serial_baud', 115200)

        # NTRIP settings (from your screenshot)
        self.declare_parameter('caster_host', '3.143.243.81')
        self.declare_parameter('caster_port', 2101)
        self.declare_parameter('mountpoint', 'RD1_STATION_SO')
        self.declare_parameter('username', 'c_eltannir@live.concordia.ca')
        self.declare_parameter('password', 'none')

        self.serial_port = self.get_parameter('serial_port').value
        self.serial_baud = int(self.get_parameter('serial_baud').value)

        self.caster_host = self.get_parameter('caster_host').value
        self.caster_port = int(self.get_parameter('caster_port').value)
        self.mountpoint = self.get_parameter('mountpoint').value
        self.username = self.get_parameter('username').value
        self.password = self.get_parameter('password').value

        if self.password == 'CHANGE_ME':
            self.get_logger().warn("You must set -p password:=YOURPASSWORD (currently CHANGE_ME).")

        # Publisher
        self.pub = self.create_publisher(NavSatFix, '/rtk/fix', 10)

        # Open serial ONCE (this is the key)
        self.get_logger().info(f"Opening serial {self.serial_port} @ {self.serial_baud}")
        self.ser = serial.Serial(self.serial_port, self.serial_baud, timeout=0.2)

        self.last_log = 0.0
        self.buf = ""

        # Start NTRIP thread (writes RTCM into same serial)
        self.stop_event = threading.Event()
        self.ntrip_thread = threading.Thread(target=self.ntrip_loop, daemon=True)
        self.ntrip_thread.start()

        # Read loop timer (reads NMEA from same serial)
        self.timer = self.create_timer(0.02, self.read_tick)

    def ntrip_loop(self):
        """
        Connect to NTRIP caster, read RTCM bytes, write them into the GNSS receiver.
        """
        while not self.stop_event.is_set():
            try:
                self.get_logger().info("Connecting to NTRIP caster...")
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(10)
                s.connect((self.caster_host, self.caster_port))

                auth = base64.b64encode(f"{self.username}:{self.password}".encode()).decode()
                req = (
                    f"GET /{self.mountpoint} HTTP/1.1\r\n"
                    f"Host: {self.caster_host}\r\n"
                    f"Ntrip-Version: Ntrip/2.0\r\n"
                    f"User-Agent: NTRIP rtk_reader\r\n"
                    f"Authorization: Basic {auth}\r\n"
                    f"Connection: close\r\n\r\n"
                )
                s.sendall(req.encode())

                # Read header
                header = b""
                while b"\r\n\r\n" not in header:
                    chunk = s.recv(1)
                    if not chunk:
                        raise ConnectionError("NTRIP closed during header")
                    header += chunk

                header_text = header.decode(errors='ignore')
                if ("200 OK" not in header_text) and ("ICY 200 OK" not in header_text):
                    raise ConnectionError(f"NTRIP auth failed / bad response:\n{header_text}")

                self.get_logger().info("NTRIP connected (RTCM streaming).")

                s.settimeout(5)
                last_bytes = time.time()

                while not self.stop_event.is_set():
                    data = s.recv(4096)
                    if not data:
                        raise ConnectionError("NTRIP stream ended")

                    # Write RTCM into the receiver
                    self.ser.write(data)

                    # occasional keep-alive log
                    now = time.time()
                    if now - last_bytes > 5:
                        last_bytes = now

            except Exception as e:
                self.get_logger().warn(f"NTRIP error: {e} (retrying in 2s)")
                try:
                    time.sleep(2)
                except Exception:
                    pass
            finally:
                try:
                    s.close()
                except Exception:
                    pass

    def read_tick(self):
        try:
            raw = self.ser.read(4096).decode('ascii', errors='ignore')
        except Exception as e:
            self.get_logger().error(f"Serial read error: {e}")
            time.sleep(0.2)
            return

        if not raw:
            return

        self.buf += raw
        while '\n' in self.buf:
            line, self.buf = self.buf.split('\n', 1)
            line = line.strip()

            parsed = parse_gga(line)
            if not parsed:
                continue

            lat, lon, alt, q = parsed

            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "gps"
            msg.latitude = lat
            msg.longitude = lon
            msg.altitude = alt

            msg.status.service = NavSatStatus.SERVICE_GPS
            if q == 0:
                msg.status.status = NavSatStatus.STATUS_NO_FIX
            elif q in (2, 4, 5):
                msg.status.status = NavSatStatus.STATUS_GBAS_FIX
            else:
                msg.status.status = NavSatStatus.STATUS_FIX

            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            self.pub.publish(msg)

            now = time.time()
            if now - self.last_log > 0.5:
                self.get_logger().info(f"{gga_quality_to_text(q)} lat={lat:.8f} lon={lon:.8f} alt={alt:.2f}m")
                self.last_log = now

    def destroy_node(self):
        self.stop_event.set()
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = RTKReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

