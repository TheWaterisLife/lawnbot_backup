# Architecture: Mower Mapping Subsystem

## 1. System Context

```
+------------------------------------------------------------------+
|                    Autonomous Lawn Mower                          |
+------------------------------------------------------------------+
|                                                                   |
|  +--------------+     +----------------+                          |
|  |  rtk_reader  |     | mower_mapping  |                          |
|  |  /rtk/fix    |     | (loads maps)   |                          |
|  +------+-------+     +----------------+                          |
|         |                                                         |
|         v                                                         |
|  +------------------------------------------+                    |
|  |         MOWER MAPPING                     |                    |
|  |          (this project)                   |                    |
|  |                                           |                    |
|  |  ROS2 Sub: /rtk/fix                       |                    |
|  |     ↓                                     |                    |
|  |  Adaptive Sampling → RDP → JSON save      |                    |
|  |     ↓                                     |                    |
|  |  WebSocket Server (ws://0.0.0.0:8770)     |                    |
|  +------------------------------------------+                    |
|                      ^                                            |
+----------------------|--------------------------------------------+
                       | WiFi
                       v
              +------------------+
              |   Mobile App     |
              |  Map recording   |
              |  Live preview    |
              +------------------+
```

---

## 2. Component Architecture

### 2.1 Package Structure

```
src/mower_mapping/
├── mower_mapping/
│   ├── __init__.py
│   └── map_server.py       # Main ROS2 node + WebSocket server
├── resource/
│   └── mower_mapping
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── package.xml
├── setup.py
└── setup.cfg
```

### 2.2 Node Description

#### MapServer Node (`map_server.py`)

**Responsibility:** Record GPS boundary, simplify, store, and serve via WebSocket.

**Class Diagram:**
```
+----------------------------------+
|           MapServer              |
+----------------------------------+
| - latest_fix: FixSample?        |
| - state: str (IDLE|MAPPING|...)  |
| - origin_latlon: (lat, lon)?    |
| - points_raw: List[FixSample]   |
| - last_saved_xy: (x, y)?       |
| - last_heading: float?          |
| - ws_clients: set               |
| - _last_built_map: dict?        |
+----------------------------------+
| + on_fix(msg)                    |
| + on_timer()                     |
| + maybe_record_point(fix)       |
| + start_mapping()               |
| + stop_mapping_and_build()      |
| + save_map(name, map_obj)       |
| + list_maps()                   |
| + load_map(map_id)              |
| + ws_handler(websocket)         |
| + handle_ws_msg(ws, msg)        |
| + ws_broadcast_status()         |
| + ws_broadcast_pose()           |
| + ws_broadcast_preview()        |
+----------------------------------+

+----------------------------------+
|     FixSample (dataclass)        |
+----------------------------------+
| lat: float                       |
| lon: float                       |
| ts: float                        |
| x: float  (local meters)         |
| y: float  (local meters)         |
+----------------------------------+
```

**Helper Functions:**

| Function | Purpose |
|----------|---------|
| `latlon_to_local_xy_m()` | Convert GPS to local meters |
| `dist_xy()` | Euclidean distance |
| `heading_deg()` | Heading between two points |
| `smallest_angle_diff_deg()` | Angular difference |
| `rdp()` | Ramer-Douglas-Peucker simplification |

---

## 3. State Machine

```
        map_start
 IDLE ──────────→ MAPPING
  ↑                  |   |
  |   map_stop/      |   | GPS lost (>2s)
  |   map_cancel     |   |
  |                  v   v
  +←────────────  PAUSED
                    ↑ |
                    | | GPS returns (<0.6s)
                    +─+
```

| State | Description | Transitions |
|-------|-------------|-------------|
| **IDLE** | Not recording | → MAPPING (map_start) |
| **MAPPING** | Actively recording GPS points | → PAUSED (GPS loss) → IDLE (map_stop/cancel) |
| **PAUSED** | GPS lost, waiting | → MAPPING (GPS returns) → IDLE (map_cancel) |

---

## 4. Adaptive Sampling Algorithm

### 4.1 Recording Triggers

A new point is recorded when **any** of these fire:

```
Record if:
  (time since last ≥ MAX_TIME_S)           // 1.0s max gap
  OR (distance ≥ MIN_DIST_M)              // 0.25m moved
  OR (heading change ≥ MIN_HEADING_DEG)    // 8° turn

Skip if:
  (time since last < MIN_TIME_S)           // 0.2s cooldown
```

### 4.2 Why Adaptive?

```
Straight edge:          Corner:
  •         •             •  •
                           • •
  (few points)           •  (many points)
                        •
```

- **Straight lines:** Only distance/time triggers → fewer points
- **Corners/curves:** Heading trigger fires → more points for accuracy

---

## 5. Path Simplification

### 5.1 Ramer-Douglas-Peucker (RDP)

```
Before (120 raw points):
•••••••••••••••••••••••••••••

After (24 simplified points):
•           •               •
 \         / \             /
  •       •   •           •
```

**Parameters:**
- Epsilon: 0.08m (max perpendicular distance error)
- Typical reduction: ~80%

### 5.2 Loop Closure

```
if dist(first_point, last_point) > 0.5m:
    append first_point to close loop

if simplified[-1] != simplified[0]:
    append simplified[0] to close
```

---

## 6. Data Flow

### 6.1 Recording Flow

```
/rtk/fix (1Hz)
    │
    ▼
on_fix()
    │
    ├── Convert lat/lon → local XY
    │
    ├── Update latest_fix
    │
    └── maybe_record_point()
            │
            ├── Check MIN_TIME_S cooldown
            ├── Check distance trigger
            ├── Check heading trigger
            ├── Check time trigger
            │
            └── Append to points_raw
```

### 6.2 Build Flow (on map_stop)

```
points_raw (120 points)
    │
    ├── Check MIN_POINTS (≥20)
    │
    ├── Extract XY coordinates
    │
    ├── Close loop (if within 0.5m)
    │
    ├── RDP simplification (eps=0.08m)
    │
    └── Return boundary map object
```

### 6.3 Map Storage

```
~/mower_ws/maps/
├── front_yard_20260210_183000.json
├── back_yard_20260210_190000.json
└── side_strip_20260211_100000.json
```

---

## 7. WebSocket Protocol

### 7.1 App → Server Commands

| Type | Fields | Response |
|------|--------|----------|
| `map_start` | — | `map_start_result` |
| `map_stop` | — | `map_stop_result` (includes built map) |
| `map_save` | `name: string` | `map_save_result` (includes id, path) |
| `map_list` | — | `map_list_result` (list of maps) |
| `map_load` | `id: string` | `map_load_result` (full map object) |
| `map_cancel` | — | `map_cancel_result` |

### 7.2 Server → App Broadcasts (5 Hz)

| Type | Fields | When |
|------|--------|------|
| `map_status` | `state`, `count` | Always |
| `rtk_pose` | `lat`, `lon`, `ts` | When fix available |
| `map_preview` | `points: [[x,y],...]` | During MAPPING/PAUSED |

---

## 8. Error Handling

| Failure | Detection | Response |
|---------|-----------|----------|
| GPS lost during mapping | No fix for 2s | Auto-pause |
| GPS returns | Fix within 0.6s | Auto-resume |
| Not enough points | < 20 on stop | Return error |
| Non-closed loop | End > 0.5m from start | Auto-append start point |
| Client disconnect | WebSocket close | Remove from broadcast set |
| No fix on start | latest_fix is None | Return error |

---

## 9. Dependencies

| Dependency | Purpose |
|------------|---------|
| `rclpy` | ROS2 Python client |
| `sensor_msgs` | NavSatFix message |
| `websockets` | WebSocket server |
| `asyncio` | Event loop |
