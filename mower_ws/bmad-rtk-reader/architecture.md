# Architecture: RTK Reader Subsystem

## 1. System Context

```
+------------------------------------------------------------------+
|                    Autonomous Lawn Mower                          |
+------------------------------------------------------------------+
|                                                                   |
|  +------------------+     +------------------+                    |
|  |Sensor Integration|     |   Navigation     |                    |
|  |  subscribes to   |     |   Subsystem      |                    |
|  |   /rtk/fix       |     |                  |                    |
|  +--------+---------+     +------------------+                    |
|           ^                                                       |
|           |                                                       |
|  +--------+---------+                                             |
|  |   RTK READER     |                                             |
|  |  (this project)  |                                             |
|  |                  |                                             |
|  |  +------------+  |                                             |
|  |  | NTRIP      |  |                                             |
|  |  | Client     |  |                                             |
|  |  +-----+------+  |                                             |
|  |        | RTCM    |                                             |
|  |        v         |                                             |
|  |  +------------+  |     /rtk/fix                                |
|  |  | Serial I/O |--+---> (NavSatFix @ 1Hz)                       |
|  |  +-----+------+  |                                             |
|  +--------+---------+                                             |
|           ^                                                       |
+-----------+-------------------------------------------------------+
            |
   +--------+--------+
   | u-blox ZED-F9P  |
   | SimpleRTK2B     |
   | /dev/ttyACM0    |
   +-----------------+
```

---

## 2. Component Architecture

### 2.1 Package Structure

```
src/rtk_reader/
├── rtk_reader/
│   ├── __init__.py
│   └── rtk_reader_node.py    # Main ROS2 node
├── resource/
│   └── rtk_reader
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── package.xml
├── setup.py
└── setup.cfg
```

### 2.2 Node Description

#### RTKReader Node (`rtk_reader_node.py`)

**Responsibility:** Read u-blox ZED-F9P via serial, stream NTRIP corrections, publish GPS fix.

**Class Diagram:**
```
+---------------------------+
|       RTKReader           |
+---------------------------+
| - ser: Serial             |
| - pub: Publisher          |
| - ntrip_thread: Thread    |
| - stop_event: Event       |
| - buf: str                |
+---------------------------+
| + __init__()              |
| + ntrip_loop()            |
| + read_tick()             |
| + destroy_node()          |
+---------------------------+
         |
         | uses
         v
+---------------------------+
|    Helper Functions       |
+---------------------------+
| + nmea_checksum_ok()      |
| + dm_to_deg()             |
| + parse_gga()             |
| + gga_quality_to_text()   |
+---------------------------+
```

---

## 3. Data Flow

### 3.1 RTCM Correction Flow

```
    NTRIP Caster          RTKReader              ZED-F9P
         |                    |                     |
         |<---HTTP GET--------|                     |
         |                    |                     |
         |---RTCM stream----->|                     |
         |                    |---serial write----->|
         |                    |                     |
         |                    |<--NMEA GGA---------|
         |                    |                     |
         |                    |---/rtk/fix-------->|
```

### 3.2 Threading Model

```
+-------------------+     +-------------------+
|   Main Thread     |     |   NTRIP Thread    |
+-------------------+     +-------------------+
|                   |     |                   |
| Timer @ 50Hz      |     | Socket recv()    |
|   read_tick()     |     |   ntrip_loop()   |
|   - Serial read   |     |   - NTRIP connect|
|   - Parse GGA     |     |   - Read RTCM    |
|   - Publish fix   |     |   - Serial write |
|                   |     |   - Auto-reconnect|
+-------------------+     +-------------------+
         |                         |
         v                         v
    +----------------------------------+
    |         Serial Port              |
    |       /dev/ttyACM0               |
    +----------------------------------+
```

---

## 4. NMEA Parsing

### 4.1 GGA Sentence Format

```
$GNGGA,123519,4807.038,N,01131.000,E,4,08,0.9,545.4,M,47.0,M,,*47
       |      |         |           |
       |      |         |           +-- Fix quality (4=RTK Fixed)
       |      |         +-- Longitude (DDDMM.MMMMM, E/W)
       |      +-- Latitude (DDMM.MMMMM, N/S)
       +-- UTC Time (HHMMSS)
```

### 4.2 Fix Quality Mapping

| GGA Quality | Meaning | NavSatStatus | Accuracy |
|-------------|---------|--------------|----------|
| 0 | Invalid | STATUS_NO_FIX | N/A |
| 1 | GPS SPS | STATUS_FIX | ~5m |
| 2 | DGPS | STATUS_GBAS_FIX | ~1m |
| 4 | **RTK Fixed** | STATUS_GBAS_FIX | **~2cm** |
| 5 | RTK Float | STATUS_GBAS_FIX | ~30cm |

### 4.3 Coordinate Conversion

NMEA uses DDMM.MMMMM format, converted to decimal degrees:

```python
def dm_to_deg(dm: str, hemi: str) -> float:
    # "4807.038" -> 48 + 07.038/60 = 48.1173
    dot = dm.find('.')
    deg = float(dm[:dot-2])
    minutes = float(dm[dot-2:])
    val = deg + minutes / 60.0
    if hemi in ('S', 'W'):
        val = -val
    return val
```

---

## 5. NTRIP Client

### 5.1 Connection Sequence

```
1. Open TCP socket to caster:port
2. Send HTTP GET request with:
   - Mountpoint path
   - Ntrip-Version: Ntrip/2.0
   - Authorization: Basic base64(user:pass)
3. Verify "200 OK" or "ICY 200 OK" response
4. Stream RTCM data to serial port
5. On disconnect, wait 2s and retry
```

### 5.2 Configuration

| Parameter | Default | Description |
|-----------|---------|-------------|
| caster_host | 3.143.243.81 | NTRIP server IP |
| caster_port | 2101 | NTRIP server port |
| mountpoint | RD1_STATION_SO | Base station stream |
| username | (configured) | Account email |
| password | (configured) | Account password |

---

## 6. Hardware Configuration

### 6.1 Serial Port

| Parameter | Value |
|-----------|-------|
| Device | `/dev/ttyACM0` |
| Baud Rate | 115200 |
| Data Bits | 8 |
| Parity | None |
| Stop Bits | 1 |

> [!NOTE]
> For stable device naming, use `/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00`

### 6.2 u-blox ZED-F9P

| Feature | Specification |
|---------|---------------|
| Receiver | Multi-band (L1/L2) |
| Constellations | GPS, GLONASS, Galileo, BeiDou |
| RTK Accuracy | 1cm + 1ppm (horizontal) |
| Update Rate | 1-20 Hz (default 1 Hz) |
| Interface | USB Serial |

---

## 7. Error Handling

### 7.1 Failure Modes

| Failure | Detection | Response |
|---------|-----------|----------|
| Serial port error | Exception on read | Log error, retry |
| NTRIP disconnect | Socket closed | Reconnect after 2s |
| Invalid NMEA | Checksum mismatch | Discard sentence |
| No RTK fix | Quality < 4 | Publish with higher covariance |
| GPS timeout | No GGA for 5s | Log warning |

### 7.2 Recovery Behavior

```
NTRIP Reconnection:
1. Close socket
2. Wait 2 seconds
3. Reconnect to caster
4. Re-authenticate
5. Resume RTCM streaming
```

---

## 8. Dependencies

### 8.1 ROS2 Packages

- `rclpy` - ROS2 Python client
- `sensor_msgs` - NavSatFix message

### 8.2 Python Libraries

- `pyserial` - Serial communication
- `socket` - NTRIP TCP connection
- `threading` - NTRIP background thread
- `base64` - NTRIP authentication

---

## 9. Launch Configuration

### 9.1 Standalone Launch

```bash
ros2 run rtk_reader rtk_reader --ros-args \
  -p serial_port:=/dev/ttyACM0 \
  -p serial_baud:=115200 \
  -p caster_host:=3.143.243.81 \
  -p caster_port:=2101 \
  -p mountpoint:=RD1_STATION_SO
```

### 9.2 Integration with Sensor Fusion

The `sensor_integration` package subscribes to `/rtk/fix` and uses `navsat_transform_node` to convert GPS coordinates to local ENU frame for EKF fusion.

```
/rtk/fix --> navsat_transform --> /gps/odom --> ekf_node --> /odometry/filtered
```
