# Software Design — RTK Reader (`rtk_reader`)

## 1. Overview

The `rtk_reader` package implements a ROS 2 node that interfaces with a u-blox ZED-F9P GNSS receiver (SimpleRTK2B board) over serial, parses NMEA GGA sentences, and publishes `sensor_msgs/NavSatFix` on `/rtk/fix`. A background NTRIP client streams RTCM3 correction data from a remote base station to the receiver, enabling centimeter-level RTK positioning.

### 1.1 Design Rationale

The node uses raw serial I/O and manual NMEA parsing (no external NMEA library) for minimal dependencies and full control over sentence validation. The NTRIP client runs in a separate daemon thread to avoid blocking the ROS 2 timer that reads and publishes GPS fixes. Serial port access is shared between the main thread (read NMEA) and the NTRIP thread (write RTCM corrections).

### 1.2 Key Responsibilities

- Read NMEA GGA sentences from the ZED-F9P serial port at 50 Hz
- Parse latitude, longitude, altitude, and fix quality with checksum validation
- Map GGA fix quality to `NavSatStatus` values
- Publish `sensor_msgs/NavSatFix` on `/rtk/fix`
- Stream NTRIP RTCM3 corrections to the receiver in a background thread
- Auto-reconnect on NTRIP connection loss with 2-second backoff

---

## 2. Class Diagram

```plantuml
@startuml
skinparam classAttributeIconSize 0

package "rtk_reader" {

  class RTKReader {
    -ser : serial.Serial
    -pub : Publisher<NavSatFix>
    -ntrip_thread : Thread
    -stop_event : threading.Event
    -buf : str
    -caster_host : str
    -caster_port : int
    -mountpoint : str
    -username : str
    -password : str
    +__init__()
    +ntrip_loop() : void
    +read_tick() : void
    +destroy_node() : void
  }

  class NMEAParser <<module>> {
    +{static} nmea_checksum_ok(sentence: str) : bool
    +{static} dm_to_deg(dm: str, hemi: str) : float
    +{static} parse_gga(sentence: str) : Optional[GGAData]
    +{static} gga_quality_to_text(quality: int) : str
  }

  class GGAData <<dataclass>> {
    +latitude : float
    +longitude : float
    +altitude : float
    +quality : int
    +num_sats : int
    +hdop : float
    +utc_time : str
  }

  RTKReader "1" ..> "1" NMEAParser : parses sentences
  NMEAParser ..> GGAData : produces
}

package "rclpy" <<external>> {
  class Node
}

package "pyserial" <<external>> {
  class Serial
}

RTKReader --|> Node
RTKReader "1" *-- "1" Serial : reads/writes

note right of RTKReader
  Two threads:
  - Main: ROS 2 timer → serial read → parse → publish
  - Daemon: NTRIP socket → RTCM → serial write
end note

@enduml
```

---

## 3. Sequence Diagram

```plantuml
@startuml
skinparam sequenceArrowThickness 2

participant "RTKReader\n(main thread)" as Main
participant "Serial Port\n/dev/ttyACM0" as Serial
participant "NMEAParser" as Parser
participant "ROS 2\n/rtk/fix" as Topic
participant "NTRIP Thread\n(daemon)" as NTRIP
participant "NTRIP Caster\n3.143.243.81:2101" as Caster
participant "ZED-F9P\nReceiver" as GPS

== Startup ==
Main -> Serial : open(/dev/ttyACM0, 115200)
Main -> NTRIP : start daemon thread

== NTRIP Correction Stream ==
NTRIP -> Caster : TCP connect
NTRIP -> Caster : GET /RD1_STATION_SO HTTP/1.1\nAuthorization: Basic <b64>
Caster --> NTRIP : ICY 200 OK

loop RTCM streaming
  Caster -> NTRIP : RTCM3 data (≤4096 bytes)
  NTRIP -> Serial : write RTCM to serial
  Serial -> GPS : RTCM corrections applied
end

== GPS Fix Reading (50 Hz timer) ==
loop every 20ms
  Main -> Serial : read available bytes
  Serial --> Main : "$GNGGA,123519,4807.038,N,..."

  Main -> Parser : parse_gga(sentence)
  Parser -> Parser : nmea_checksum_ok()
  Parser -> Parser : dm_to_deg(lat), dm_to_deg(lon)
  Parser --> Main : GGAData(lat, lon, alt, quality=4)

  Main -> Main : map quality → NavSatStatus
  note right : quality 4 = RTK Fixed\n→ STATUS_GBAS_FIX
  Main -> Topic : publish(NavSatFix)
end

== NTRIP Reconnection ==
Caster -x NTRIP : connection dropped
NTRIP -> NTRIP : sleep(2 seconds)
NTRIP -> Caster : TCP reconnect
NTRIP -> Caster : re-authenticate
Caster --> NTRIP : ICY 200 OK
note right of NTRIP : Auto-reconnect\nwith 2s backoff

== Shutdown ==
Main -> NTRIP : stop_event.set()
NTRIP -> Caster : close socket
Main -> Serial : close()

@enduml
```

---

## 4. System Context Diagram

```plantuml
@startuml
!include <C4/C4_Context>

title System Context — RTK Reader

System(rtk, "rtk_reader", "ROS 2 GPS node\nPublishes /rtk/fix")
System(localization, "localization_node", "Fuses GPS into\ndead-reckoning pose")
System(mapping, "mower_mapping", "Records GPS boundary\npolygons for mowing areas")
System_Ext(zedf9p, "u-blox ZED-F9P\n(SimpleRTK2B)", "Multi-band GNSS\n/dev/ttyACM0 @ 115200")
System_Ext(ntrip, "NTRIP Caster", "3.143.243.81:2101\nRTCM3 corrections")

ntrip --> rtk : RTCM3 correction stream\n(TCP socket, auto-reconnect)
zedf9p <--> rtk : Serial: NMEA read\nRTCM write
rtk --> localization : /rtk/fix\n(NavSatFix)
rtk --> mapping : /rtk/fix\n(NavSatFix)

note right of rtk
  Fix quality mapping:
  0 → STATUS_NO_FIX
  1 → STATUS_FIX (~5m)
  4 → STATUS_GBAS_FIX (~2cm)
  5 → STATUS_GBAS_FIX (~30cm)
end note

@enduml
```

---

## 5. Detailed Design

### 5.1 NMEA Parsing

The parser is implemented without external NMEA libraries. It directly splits GGA sentences on commas and validates using XOR-based NMEA checksum. The `dm_to_deg()` function converts NMEA's degrees-minutes format (`DDMM.MMMMM`) to decimal degrees. Only `$GNGGA` and `$GPGGA` sentence types are processed; all others are discarded.

### 5.2 Fix Quality Mapping

| GGA Quality | Description | NavSatStatus | Typical Accuracy |
|---|---|---|---|
| 0 | No fix | `STATUS_NO_FIX` | N/A |
| 1 | GNSS autonomous | `STATUS_FIX` | ~5 m |
| 2 | DGPS | `STATUS_GBAS_FIX` | ~1 m |
| 4 | RTK Fixed | `STATUS_GBAS_FIX` | ~2 cm |
| 5 | RTK Float | `STATUS_GBAS_FIX` | ~30 cm |

### 5.3 NTRIP Client

The NTRIP client connects via raw TCP socket and sends an HTTP-style GET request with Basic authentication (Base64-encoded credentials). The connection sequence:

1. Open TCP socket to `caster_host:caster_port`
2. Send `GET /mountpoint HTTP/1.1` with `Authorization: Basic <credentials>`
3. Verify `200 OK` or `ICY 200 OK` in the response header
4. Enter streaming loop: read RTCM3 data (up to 4096 bytes per read) and write to serial
5. On disconnect: close socket, wait 2 seconds, reconnect

### 5.4 Threading Model

| Thread | Responsibility | Frequency |
|---|---|---|
| Main (ROS 2 timer) | Serial read → NMEA parse → publish `/rtk/fix` | 50 Hz |
| NTRIP daemon | Socket recv → serial write (RTCM corrections) | Continuous |

The `threading.Event` (`stop_event`) coordinates graceful shutdown. The NTRIP thread runs as a daemon so it is killed if the main thread exits unexpectedly.

### 5.5 Serial Configuration

| Parameter | Value |
|---|---|
| Device | `/dev/ttyACM0` |
| Baud rate | 115200 |
| Data bits | 8 |
| Parity | None |
| Stop bits | 1 |

### 5.6 Error Handling

| Failure | Detection | Response |
|---|---|---|
| Serial port error | Exception on read | Log error, retry on next tick |
| NTRIP disconnect | Socket closed/timeout | Auto-reconnect after 2s backoff |
| Invalid NMEA sentence | Checksum mismatch | Discard, continue |
| No RTK fix | GGA quality < 4 | Publish with degraded status |
| GPS data timeout | No GGA sentence for 5s | Log warning |
