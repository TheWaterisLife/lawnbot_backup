# Architecture: Lawnbot Comms Subsystem

## 1. System Context

```
+------------------------------------------------------------------+
|                    Autonomous Lawn Mower                          |
+------------------------------------------------------------------+
|                                                                   |
|  +-----------------+     +------------------+                     |
|  |   Navigation    |     |Sensor Integration|                     |
|  |  /mower/state   |     | /mower/telemetry |                     |
|  +--------+--------+     +---------+--------+                     |
|           |                        |                              |
|           v                        v                              |
|  +------------------------------------------+                    |
|  |        LAWNBOT COMMS BRIDGE               |                    |
|  |           (this project)                  |                    |
|  |                                           |                    |
|  |  ROS2 Subs ──> JSON ──> WebSocket ──> App |                    |
|  |  ROS2 Pubs <── JSON <── WebSocket <── App |                    |
|  |                                           |                    |
|  |  WebSocket Server: ws://0.0.0.0:9002      |                    |
|  +------------------------------------------+                    |
|                      ^                                            |
+----------------------|--------------------------------------------+
                       | WiFi (phone hotspot)
                       v
              +------------------+
              |   Mobile App     |
              |  (WebSocket      |
              |   client)        |
              +------------------+
```

---

## 2. Component Architecture

### 2.1 Package Structure

```
src/lawnbot_comms/
├── lawnbot_comms/
│   ├── __init__.py
│   ├── bridge_node.py          # WebSocket ↔ ROS2 bridge
│   └── fake_telemetry_node.py  # Test telemetry publisher
├── resource/
│   └── lawnbot_comms
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── package.xml
├── setup.py
└── setup.cfg
```

### 2.2 Node Descriptions

#### Bridge Node (`bridge_node.py`)

**Responsibility:** Bidirectional WebSocket ↔ ROS2 message bridge.

**Class Diagram:**
```
+---------------------------+
|       BridgeNode          |
+---------------------------+
| - pub_heartbeat: Publisher|
| - pub_mode: Publisher     |
| - pub_teleop: Publisher   |
| - sub_state: Subscription|
| - sub_telemetry: Sub      |
| - _send_queue: Queue      |
| - _client_connected: bool |
| - _last_heartbeat: float  |
+---------------------------+
| + publish_heartbeat()     |
| + publish_mode(value)     |
| + publish_teleop(lin,ang) |
| + heartbeat_age_ms()      |
| - _on_state(msg)          |
| - _on_telemetry(msg)      |
+---------------------------+
         |
         | async handler
         v
+---------------------------+
|     ws_handler()          |
+---------------------------+
| + rx_loop() - receive     |
| + tx_loop() - transmit    |
+---------------------------+
```

#### Fake Telemetry Node (`fake_telemetry_node.py`)

**Responsibility:** Publish simulated state and telemetry at 5 Hz for testing.

| Topic | Content | Rate |
|-------|---------|------|
| `/mower/state` | `mode=TELEOP armed=false estop=false t=...` | 5 Hz |
| `/mower/telemetry` | `wifi_ip=<ip> t=...` | 5 Hz |

---

## 3. Data Flow

### 3.1 Phone → Mower (Commands)

```
Mobile App                BridgeNode             ROS2 System
    |                         |                       |
    |--{"type":"heartbeat"}-->|                       |
    |                         |--/app/heartbeat------>|
    |                         |                       |
    |--{"type":"mode",------->|                       |
    |   "value":"TELEOP"}     |--/app/mode----------->|
    |                         |                       |
    |--{"type":"teleop",----->|                       |
    |   "linear":0.3,         |--/app/teleop_cmd----->|
    |   "angular":0.1}        |  (Twist msg)          |
```

### 3.2 Mower → Phone (Telemetry)

```
ROS2 System               BridgeNode             Mobile App
    |                         |                       |
    |--/mower/state---------->|                       |
    |                         |--{"type":"state",---->|
    |                         |   "data":"..."}       |
    |                         |                       |
    |--/mower/telemetry------>|                       |
    |                         |--{"type":"telemetry"->|
    |                         |   "data":"..."}       |
    |                         |                       |
    |                    (idle 200ms)                  |
    |                         |--{"type":"comms",---->|
    |                         |   "heartbeat_age_ms"} |
```

### 3.3 Connection Lifecycle

```
1. Server starts: ws://0.0.0.0:9002
2. App connects → ACK sent
3. App sends "hello" → version ACK
4. App sends heartbeat periodically
5. Bridge forwards state/telemetry
6. On disconnect → slot freed
7. New client can connect
```

---

## 4. Threading Model

```
+-------------------------------+
|        asyncio Event Loop     |
+-------------------------------+
|                               |
|  +----------+  +----------+  |
|  | ROS2     |  | WebSocket|  |
|  | spin_once|  | server   |  |
|  | (50ms)   |  | (async)  |  |
|  +----------+  +-----+----+  |
|                       |       |
|              +--------+----+  |
|              |  Per-Client |  |
|              |  rx_loop()  |  |
|              |  tx_loop()  |  |
|              +-------------+  |
+-------------------------------+
```

- **Main loop**: Alternates between `rclpy.spin_once()` and `asyncio.sleep()`
- **rx_loop**: Receives JSON from phone, publishes to ROS2
- **tx_loop**: Dequeues ROS2 messages, sends JSON to phone
- **Idle**: Sends comms health every 200ms when no messages queued

---

## 5. WebSocket Protocol

### 5.1 Phone → Mower Messages

| Type | Fields | ROS2 Action |
|------|--------|-------------|
| `hello` | — | Reply with ACK |
| `heartbeat` | — | Publish Empty to `/app/heartbeat` |
| `mode` | `value: string` | Publish String to `/app/mode` |
| `teleop` | `linear: float, angular: float` | Publish Twist to `/app/teleop_cmd` |

### 5.2 Mower → Phone Messages

| Type | Fields | Source |
|------|--------|--------|
| `ack` | `msg, version` | On connect / hello |
| `state` | `data: string` | Subscription to `/mower/state` |
| `telemetry` | `data: string` | Subscription to `/mower/telemetry` |
| `comms` | `heartbeat_age_ms: int` | Idle keepalive (every 200ms) |
| `error` | `msg: string` | Rejected 2nd client |

---

## 6. Error Handling

| Failure | Detection | Response |
|---------|-----------|----------|
| Malformed JSON | Parse exception | Silently discard |
| Client disconnect | WebSocket close | Free slot, log |
| Second client | `_client_connected` flag | Send error, close |
| ROS2 topic down | No messages | tx_loop sends comms keepalive |
| WiFi dropout | WebSocket exception | Client freed on disconnect |

---

## 7. Dependencies

### 7.1 ROS2 Packages

- `rclpy` — ROS2 Python client
- `std_msgs` — String, Empty messages
- `geometry_msgs` — Twist message

### 7.2 Python Libraries

- `websockets` — Async WebSocket server
- `asyncio` — Event loop
- `json` — Message serialization
