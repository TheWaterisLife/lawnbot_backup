cat > ~/rtk/rtk_rover.py <<'PY'
#!/usr/bin/env python3
import base64
import json
import os
import socket
import threading
import time
from datetime import datetime, timezone

import serial

# ====== CONFIG ======
SER_PORT = "/dev/ttyUSB0"
SER_BAUD = 115200

NTRIP_HOST = "3.143.243.81"
NTRIP_PORT = 2101
NTRIP_MOUNT = "RD1_STATION_SO"
NTRIP_USER = "c_eltann@live.concordia.ca"
NTRIP_PASS = ""   # blank password

JSON_OUT = "/run/rtk_status.json"
CSV_OUT  = "/var/log/rtk_path.csv"

WRITE_JSON_EVERY = 1.0     # seconds
WRITE_CSV_EVERY  = 1.0     # seconds
# ====================


def iso_now():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def write_json_atomic(path: str, obj: dict):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(obj, f, indent=2)
    os.replace(tmp, path)


def ensure_csv_header():
    os.makedirs(os.path.dirname(CSV_OUT), exist_ok=True)
    if not os.path.exists(CSV_OUT):
        with open(CSV_OUT, "w") as f:
            f.write("time,lat,lon,alt_m,fix_quality,nsat,hdop\n")


def nmea_checksum(payload: str) -> str:
    c = 0
    for ch in payload:
        c ^= ord(ch)
    return f"{c:02X}"


def send_quectel_msgrate(ser: serial.Serial, msg: str, rate: int):
    # Quectel supports $PQTMCFGMSGRATE,W,<MSG>,<RATE>*CS
    # Example from Quectel forum shows enabling GGA etc.  [oai_citation:0‡Quectel Forums](https://forums.quectel.com/t/testing-with-lg290p-in-rover-and-base-station-mode-need-some-help/38719/8)
    payload = f"PQTMCFGMSGRATE,W,{msg},{rate}"
    cs = nmea_checksum(payload)
    line = f"${payload}*{cs}\r\n".encode("ascii")
    ser.write(line)
    ser.flush()


def send_quectel_save(ser: serial.Serial):
    payload = "PQTMSAVEPAR"
    cs = nmea_checksum(payload)
    ser.write(f"${payload}*{cs}\r\n".encode("ascii"))
    ser.flush()


class NTRIPThread(threading.Thread):
    def __init__(self, ser: serial.Serial, state: dict):
        super().__init__(daemon=True)
        self.ser = ser
        self.state = state

    def connect(self):
        s = socket.create_connection((NTRIP_HOST, NTRIP_PORT), timeout=10)
        auth = f"{NTRIP_USER}:{NTRIP_PASS}".encode("utf-8")
        auth_b64 = base64.b64encode(auth).decode("ascii")

        req = (
            f"GET /{NTRIP_MOUNT} HTTP/1.1\r\n"
            f"Host: {NTRIP_HOST}\r\n"
            f"User-Agent: lawnbot-rtk/1.0\r\n"
            f"Authorization: Basic {auth_b64}\r\n"
            f"Ntrip-Version: Ntrip/2.0\r\n"
            f"Connection: close\r\n"
            f"\r\n"
        ).encode("ascii")
        s.sendall(req)

        data = b""
        while b"\r\n\r\n" not in data:
            chunk = s.recv(1024)
            if not chunk:
                raise RuntimeError("NTRIP: no response")
            data += chunk

        head, rest = data.split(b"\r\n\r\n", 1)
        head_txt = head.decode("latin1", errors="replace")
        if ("200 OK" not in head_txt) and ("ICY 200 OK" not in head_txt):
            raise RuntimeError("NTRIP bad response:\n" + head_txt)

        return s, rest

    def run(self):
        sock = None
        pending = b""
        last_bytes = 0
        t0 = time.time()

        while True:
            try:
                if sock is None:
                    sock, pending = self.connect()
                    sock.settimeout(2.0)
                    self.state["ntrip"] = "CONNECTED"
                    last_bytes = 0
                    t0 = time.time()

                chunk = pending if pending else sock.recv(4096)
                pending = b""
                if not chunk:
                    raise RuntimeError("NTRIP closed")

                # write RTCM into GNSS
                self.ser.write(chunk)
                self.ser.flush()

                last_bytes += len(chunk)
                now = time.time()
                if now - t0 >= 5.0:
                    self.state["rtcm_in_bps"] = (last_bytes * 8.0) / (now - t0)
                    last_bytes = 0
                    t0 = now

            except Exception:
                self.state["ntrip"] = "DISCONNECTED"
                try:
                    if sock:
                        sock.close()
                except Exception:
                    pass
                sock = None
                pending = b""
                time.sleep(2)


def parse_gga(line: str):
    # minimal $GNGGA/$GPGGA parser
    # returns dict or None
    if not (line.startswith("$GNGGA") or line.startswith("$GPGGA")):
        return None
    parts = line.split(",")
    if len(parts) < 10:
        return None

    # lat ddmm.mmmm, N/S
    lat_s = parts[2]
    lat_h = parts[3]
    lon_s = parts[4]
    lon_h = parts[5]
    fixq = parts[6]
    nsat = parts[7]
    hdop = parts[8]
    altm = parts[9]

    def dm_to_deg(dm: str, hemi: str, is_lat: bool):
        if not dm:
            return None
        try:
            if is_lat:
                deg = int(dm[0:2]); mins = float(dm[2:])
            else:
                deg = int(dm[0:3]); mins = float(dm[3:])
            val = deg + mins / 60.0
            if hemi in ("S", "W"):
                val = -val
            return val
        except Exception:
            return None

    lat = dm_to_deg(lat_s, lat_h, True)
    lon = dm_to_deg(lon_s, lon_h, False)

    try:
        fixq_i = int(fixq) if fixq else 0
    except Exception:
        fixq_i = 0

    def fix_name(q):
        # NMEA GGA fix quality:
        # 0 invalid, 1 GPS, 2 DGPS, 4 RTK fixed, 5 RTK float
        return {
            0: "NO_FIX",
            1: "GPS",
            2: "DGPS",
            4: "RTK_FIXED",
            5: "RTK_FLOAT",
        }.get(q, f"Q{q}")

    try:
        nsat_i = int(nsat) if nsat else None
    except Exception:
        nsat_i = None

    try:
        hdop_f = float(hdop) if hdop else None
    except Exception:
        hdop_f = None

    try:
        alt_f = float(altm) if altm else None
    except Exception:
        alt_f = None

    return {
        "lat": lat,
        "lon": lon,
        "alt_m": alt_f,
        "fix_quality": fixq_i,
        "fix": fix_name(fixq_i),
        "nsat": nsat_i,
        "hdop": hdop_f,
    }


class SerialReadThread(threading.Thread):
    def __init__(self, ser: serial.Serial, state: dict):
        super().__init__(daemon=True)
        self.ser = ser
        self.state = state
        self.buf = b""

    def run(self):
        while True:
            try:
                b = self.ser.read(4096)
                if b:
                    self.buf += b
                    # split on newline
                    while b"\n" in self.buf:
                        line, self.buf = self.buf.split(b"\n", 1)
                        try:
                            s = line.decode("ascii", errors="ignore").strip()
                        except Exception:
                            continue
                        if not s.startswith("$"):
                            continue
                        gga = parse_gga(s)
                        if gga and gga["lat"] is not None and gga["lon"] is not None:
                            self.state.update(gga)
                            self.state["time"] = iso_now()
            except Exception:
                time.sleep(0.2)


def main():
    ensure_csv_header()

    ser = serial.Serial(SER_PORT, SER_BAUD, timeout=0.2)
    time.sleep(0.2)

    state = {
        "time": iso_now(),
        "lat": None,
        "lon": None,
        "alt_m": None,
        "fix_quality": 0,
        "fix": "UNKNOWN",
        "nsat": None,
        "hdop": None,
        "rtcm_in_bps": 0.0,
        "ntrip": "DISCONNECTED",
    }

    # Ask module to output useful NMEA at 1 Hz (GGA is the key one for RTK status)
    # This uses the same PQTMCFGMSGRATE family shown in Quectel examples.  [oai_citation:1‡Quectel Forums](https://forums.quectel.com/t/testing-with-lg290p-in-rover-and-base-station-mode-need-some-help/38719/8)
    try:
        send_quectel_msgrate(ser, "GGA", 1)
        send_quectel_msgrate(ser, "RMC", 1)
        send_quectel_msgrate(ser, "GSA", 1)
        send_quectel_msgrate(ser, "GST", 1)
        send_quectel_save(ser)
    except Exception:
        pass  # not fatal

    t_ntrip = NTRIPThread(ser, state)
    t_ser = SerialReadThread(ser, state)
    t_ntrip.start()
    t_ser.start()

    last_json = 0.0
    last_csv = 0.0

    while True:
        now = time.time()

        if now - last_json >= WRITE_JSON_EVERY:
            last_json = now
            write_json_atomic(JSON_OUT, {
                "time": state.get("time", iso_now()),
                "lat": state.get("lat"),
                "lon": state.get("lon"),
                "alt_m": state.get("alt_m"),
                "fix_quality": state.get("fix_quality", 0),
                "fix": state.get("fix", "UNKNOWN"),
                "nsat": state.get("nsat"),
                "hdop": state.get("hdop"),
                "rtcm_in_bps": round(float(state.get("rtcm_in_bps", 0.0)), 1),
                "ntrip": state.get("ntrip", "UNKNOWN"),
            })

        if now - last_csv >= WRITE_CSV_EVERY:
            last_csv = now
            if state.get("lat") is not None and state.get("lon") is not None:
                with open(CSV_OUT, "a") as f:
                    f.write(
                        f'{state.get("time", iso_now())},'
                        f'{state.get("lat")},'
                        f'{state.get("lon")},'
                        f'{state.get("alt_m")},'
                        f'{state.get("fix_quality")},'
                        f'{state.get("nsat")},'
                        f'{state.get("hdop")}\n'
                    )

        time.sleep(0.05)


if __name__ == "__main__":
    main()
PY

chmod +x ~/rtk/rtk_rover.py
