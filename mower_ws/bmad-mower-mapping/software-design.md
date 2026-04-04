# Software Design — Mower Mapping (`mower_mapping`)

## 1. Overview

The `mower_mapping` package implements a ROS 2 node with an embedded WebSocket server (port 8770) that records GPS boundary polygons defining mowing areas. It subscribes to `/rtk/fix` for GPS position, applies adaptive sampling to capture boundary geometry efficiently, simplifies paths using the Ramer-Douglas-Peucker algorithm, and stores maps as JSON files.

### 1.1 Design Rationale

The mapping subsystem combines a ROS 2 subscriber (for GPS data) with a WebSocket server (for mobile app control) in a single node. This hybrid approach allows the mobile app to control recording sessions and preview boundaries in real time without requiring ROS 2 on the phone. Adaptive sampling captures corners with high resolution while keeping straight edges sparse.

### 1.2 Key Responsibilities

- Subscribe to `/rtk/fix` and convert GPS coordinates to local Cartesian meters
- Record boundary points using adaptive sampling (distance, heading, and time triggers)
- Simplify recorded paths using RDP algorithm (epsilon = 0.08 m)
- Manage recording state machine (IDLE → MAPPING → PAUSED → IDLE)
- Auto-pause on GPS signal loss, auto-resume on recovery
- Save/load/list maps as JSON in `~/mower_ws/maps/`
- Broadcast status, RTK pose, and boundary preview to connected WebSocket clients at 5 Hz

---

## 2. Class Diagram

```plantuml
@startuml
skinparam classAttributeIconSize 0

package "mower_mapping" {

  class MapServer {
    -latest_fix : FixSample
    -state : MapState
    -origin_latlon : Tuple[float, float]
    -points_raw : List[FixSample]
    -last_saved_xy : Tuple[float, float]
    -last_heading : float
    -last_record_time : float
    -ws_clients : Set[WebSocket]
    -_last_built_map : dict
    -sub_fix : Subscription<NavSatFix>
    -timer : Timer
    -maps_dir : Path
    +on_fix(msg: NavSatFix) : void
    +on_timer() : void
    +maybe_record_point(fix: FixSample) : bool
    +start_mapping() : dict
    +stop_mapping_and_build() : dict
    +save_map(name: str, map_obj: dict) : dict
    +list_maps() : List[dict]
    +load_map(map_id: str) : dict
    +ws_handler(websocket: WebSocket) : void
    +handle_ws_msg(ws, msg: dict) : void
    +ws_broadcast_status() : void
    +ws_broadcast_pose() : void
    +ws_broadcast_preview() : void
  }

  enum MapState {
    IDLE
    MAPPING
    PAUSED
  }

  class FixSample <<dataclass>> {
    +lat : float
    +lon : float
    +ts : float
    +x : float
    +y : float
  }

  class CoordinateConverter <<module>> {
    +{static} latlon_to_local_xy_m(lat, lon, origin) : Tuple[float, float]
    +{static} dist_xy(p1, p2) : float
    +{static} heading_deg(p1, p2) : float
    +{static} smallest_angle_diff_deg(a, b) : float
  }

  class PathSimplifier <<module>> {
    +{static} rdp(points: List, epsilon: float) : List
    +{static} close_loop(points: List, threshold: float) : List
  }

  MapServer "1" *-- "*" FixSample : records
  MapServer "1" *-- "1" MapState : current state
  MapServer ..> CoordinateConverter : uses
  MapServer ..> PathSimplifier : uses
}

package "rclpy" <<external>> {
  class Node
}

package "websockets" <<external>> {
  class WebSocket
}

MapServer --|> Node

@enduml
```

---

## 3. Sequence Diagram

```plantuml
@startuml
skinparam sequenceArrowThickness 2

actor "Mobile App" as App
participant "MapServer\n(ROS 2 + WS)" as Server
participant "CoordinateConverter" as Conv
participant "Adaptive Sampler" as Sampler
participant "PathSimplifier\n(RDP)" as RDP
participant "ROS 2\n/rtk/fix" as GPS
participant "File System\n~/mower_ws/maps/" as FS

== Start Recording ==
App -> Server : {"type":"map_start"}
Server -> Server : state = MAPPING\norigin = first GPS fix
Server --> App : {"type":"map_start_result","ok":true}

== GPS Fix Arrives ==
loop /rtk/fix at ~5 Hz
  GPS -> Server : NavSatFix(lat, lon, alt, quality)

  Server -> Conv : latlon_to_local_xy_m(lat, lon, origin)
  Conv --> Server : (x, y) in meters

  Server -> Sampler : maybe_record_point(fix)
  Sampler -> Sampler : check MIN_TIME_S cooldown (0.2s)
  Sampler -> Sampler : check distance ≥ 0.25m?
  Sampler -> Sampler : check heading change ≥ 8°?
  Sampler -> Sampler : check time elapsed ≥ 1.0s?

  alt any trigger fires
    Sampler --> Server : recorded = true
    Server -> Server : append to points_raw
  else no trigger
    Sampler --> Server : recorded = false
  end
end

== Broadcast (5 Hz) ==
loop every 200ms
  Server -> App : {"type":"map_status","state":"MAPPING","count":47}
  Server -> App : {"type":"rtk_pose","lat":...,"lon":...}
  Server -> App : {"type":"map_preview","points":[[x,y],...]}
end

== GPS Signal Loss ==
Server -> Server : no fix for 2 seconds
Server -> Server : state = PAUSED
Server -> App : {"type":"map_status","state":"PAUSED"}
...
Server -> Server : fix returns within 0.6s
Server -> Server : state = MAPPING (auto-resume)

== Stop and Build ==
App -> Server : {"type":"map_stop"}
Server -> Server : check points_raw ≥ 20
Server -> RDP : rdp(points_xy, epsilon=0.08)
RDP -> RDP : simplify ~120 → ~24 points
RDP --> Server : simplified boundary

Server -> Server : close_loop(simplified, 0.5m threshold)
note right : if end > 0.5m from start,\nappend start point

Server -> Server : state = IDLE
Server --> App : {"type":"map_stop_result","ok":true,"boundary":{...}}

== Save Map ==
App -> Server : {"type":"map_save","name":"front_yard"}
Server -> FS : write front_yard_20260325_143000.json
FS --> Server : saved
Server --> App : {"type":"map_save_result","id":"front_yard_20260325_143000"}

== Load Map ==
App -> Server : {"type":"map_load","id":"front_yard_20260325_143000"}
Server -> FS : read JSON file
FS --> Server : map object
Server --> App : {"type":"map_load_result","map":{...}}

@enduml
```

---

## 4. System Context Diagram

```plantuml
@startuml
!include <C4/C4_Context>

title System Context — Mower Mapping

System(mapping, "mower_mapping", "ROS 2 node + WebSocket\nBoundary recording & storage\nws://0.0.0.0:8770")
System(rtk, "rtk_reader", "GPS positioning\nPublishes /rtk/fix")
System(autonomy, "autonomy_node", "Loads boundary map\nfor coverage planning")
Person(app, "Mobile App", "Controls recording\nPreviews boundary")
System_Ext(fs, "File System", "~/mower_ws/maps/\nJSON boundary files")

rtk --> mapping : /rtk/fix\n(NavSatFix)
app --> mapping : WebSocket commands\n(map_start, map_stop,\nmap_save, map_load)
mapping --> app : WebSocket broadcasts\n(status, pose, preview @ 5 Hz)
mapping --> fs : Save/load JSON maps
autonomy --> mapping : /autonomy/start\n/autonomy/stop (events)

note bottom of mapping
  Adaptive sampling:
  - Distance: ≥ 0.25m
  - Heading: ≥ 8°
  - Time: ≥ 1.0s
  - Cooldown: 0.2s
  
  RDP simplification: ε = 0.08m
end note

@enduml
```

---

## 5. Detailed Design

### 5.1 Coordinate Conversion

GPS coordinates (latitude, longitude) are converted to local Cartesian meters using the equirectangular approximation:

```
x = (lon - lon₀) × R × cos(lat₀) × π/180
y = (lat - lat₀) × R × π/180
```

where R = 6,371,000 m and (lat₀, lon₀) is the first recorded GPS fix, which becomes the local origin. This approximation is accurate within ~1 cm for areas under 1 km².

### 5.2 Adaptive Sampling Algorithm

A new boundary point is recorded when **any** of the following triggers fire:

| Trigger | Threshold | Purpose |
|---|---|---|
| Distance | ≥ 0.25 m | Ensures minimum spatial resolution |
| Heading change | ≥ 8° | Captures corners with high density |
| Time elapsed | ≥ 1.0 s | Prevents gaps on slow movement |

A minimum cooldown of 0.2 seconds prevents over-sampling. The result is dense points on curves and sparse points on straight edges.

### 5.3 Path Simplification (RDP)

After recording stops, the Ramer-Douglas-Peucker algorithm simplifies the raw boundary with epsilon = 0.08 m. This typically reduces point count by ~80% (e.g., 120 → 24 points) while preserving boundary shape within 8 cm accuracy. If the endpoint is more than 0.5 m from the start point, the start point is appended to close the polygon.

### 5.4 State Machine

| State | Description | Transitions |
|---|---|---|
| `IDLE` | Not recording | → `MAPPING` (on `map_start`) |
| `MAPPING` | Actively recording GPS points | → `PAUSED` (GPS loss > 2s), → `IDLE` (map_stop/cancel) |
| `PAUSED` | GPS lost, waiting for recovery | → `MAPPING` (GPS returns < 0.6s), → `IDLE` (map_cancel) |

### 5.5 WebSocket Protocol

**App → Server Commands:** `map_start`, `map_stop`, `map_save`, `map_list`, `map_load`, `map_cancel`

**Server → App Broadcasts (5 Hz):**

| Message | Content | When |
|---|---|---|
| `map_status` | state, point count | Always |
| `rtk_pose` | lat, lon, timestamp | When GPS fix available |
| `map_preview` | boundary [[x,y],...] | During MAPPING/PAUSED |

### 5.6 Map Storage

Maps are stored as JSON files in `~/mower_ws/maps/` with naming convention `<name>_<YYYYMMDD_HHMMSS>.json`. Each file contains the simplified boundary polygon (local XY coordinates), GPS origin coordinates, metadata (point count, creation time), and the raw GPS coordinates for reference.
