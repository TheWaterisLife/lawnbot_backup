# Software Design — Lawnbot Comms (`lawnbot_comms`)

## 1. Overview

The `lawnbot_comms` package implements a bidirectional WebSocket-to-ROS 2 bridge on port 9002. It translates JSON messages from the mobile app into ROS 2 topic publications (commands) and forwards ROS 2 subscription data back to the app as JSON (telemetry). The bridge enforces single-client connectivity and uses a cooperative asyncio model interleaved with `rclpy.spin_once()`.

### 1.1 Design Rationale

The mobile app communicates over WiFi (phone hotspot) and cannot run a ROS 2 client. WebSocket provides a lightweight, bidirectional transport that works across platforms. The bridge node translates between the two worlds: it publishes ROS 2 messages from phone commands and serializes ROS 2 subscription data into JSON for the phone. A single-client policy simplifies state management and prevents conflicting control inputs.

### 1.2 Key Responsibilities

- Operate WebSocket server on port 9002
- Translate phone commands (heartbeat, mode, teleop) into ROS 2 topic publications
- Forward ROS 2 state and telemetry messages to connected phone as JSON
- Send keepalive `comms` messages every 200 ms when idle
- Enforce single-client connectivity (reject second connections with error)

---

## 2. Class Diagram

```plantuml
@startuml
skinparam classAttributeIconSize 0

package "lawnbot_comms" {

  class BridgeNode {
    -pub_heartbeat : Publisher<Empty>
    -pub_mode : Publisher<String>
    -pub_teleop : Publisher<Twist>
    -pub_connected : Publisher<Bool>
    -sub_state : Subscription<String>
    -sub_telemetry : Subscription<String>
    -_send_queue : asyncio.Queue
    -_client_connected : bool
    -_last_heartbeat : float
    +publish_heartbeat() : void
    +publish_mode(value: str) : void
    +publish_teleop(linear: float, angular: float) : void
    +heartbeat_age_ms() : int
    -_on_state(msg: String) : void
    -_on_telemetry(msg: String) : void
    -_enqueue(payload: dict) : void
  }

  class WSHandler <<async>> {
    +ws_handler(websocket: WebSocket) : void
    +rx_loop(websocket, node: BridgeNode) : void
    +tx_loop(websocket, node: BridgeNode) : void
  }

  class FakeTelemetryNode {
    -pub_state : Publisher<String>
    -pub_telemetry : Publisher<String>
    -timer : Timer
    +timer_callback() : void
  }

  BridgeNode "1" ..> "1" WSHandler : delegates WS handling
  WSHandler ..> BridgeNode : calls publish methods\nreads send queue
}

package "rclpy" <<external>> {
  class Node
}

package "websockets" <<external>> {
  class WebSocket
}

BridgeNode --|> Node
FakeTelemetryNode --|> Node

note right of WSHandler
  Two concurrent coroutines per client:
  - rx_loop(): phone → ROS 2
  - tx_loop(): ROS 2 → phone
  Idle keepalive every 200ms
end note

@enduml
```

---

## 3. Sequence Diagram

```plantuml
@startuml
skinparam sequenceArrowThickness 2

actor "Mobile App" as App
participant "WSHandler\n(asyncio)" as WS
participant "BridgeNode\n(ROS 2)" as Bridge
participant "ROS 2\nPublishers" as Pubs
participant "ROS 2\nSubscriptions" as Subs

== Connection ==
App -> WS : WebSocket connect (ws://mower:9002)
WS -> Bridge : check _client_connected
alt already connected
  WS --> App : {"type":"error","msg":"another client connected"}
  WS -> WS : close connection
else slot available
  WS -> Bridge : _client_connected = true
  Bridge -> Pubs : publish Bool(true) on /app/connected
  WS --> App : {"type":"ack","msg":"connected","version":"1.0"}
  WS -> WS : spawn rx_loop() + tx_loop()
end

== Phone → Mower (rx_loop) ==
App -> WS : {"type":"hello"}
WS --> App : {"type":"ack","msg":"hello","version":"1.0"}

App -> WS : {"type":"heartbeat"}
WS -> Bridge : publish_heartbeat()
Bridge -> Pubs : publish Empty on /app/heartbeat
Bridge -> Bridge : _last_heartbeat = now()

App -> WS : {"type":"mode","value":"AUTONOMOUS"}
WS -> Bridge : publish_mode("AUTONOMOUS")
Bridge -> Pubs : publish String on /app/mode

App -> WS : {"type":"teleop","linear":0.3,"angular":0.1}
WS -> Bridge : publish_teleop(0.3, 0.1)
Bridge -> Pubs : publish Twist on /app/teleop_cmd

== Mower → Phone (tx_loop) ==
Subs -> Bridge : /mower/state callback
Bridge -> Bridge : _enqueue({"type":"state","data":"..."})
note right : call_soon_threadsafe()\nfrom ROS 2 callback thread
Bridge -> WS : dequeue message
WS --> App : {"type":"state","data":"mode=TELEOP..."}

Subs -> Bridge : /mower/telemetry callback
Bridge -> Bridge : _enqueue({"type":"telemetry","data":"..."})
Bridge -> WS : dequeue message
WS --> App : {"type":"telemetry","data":"wifi_ip=..."}

== Idle Keepalive (200ms) ==
WS -> WS : no messages queued for 200ms
WS -> Bridge : heartbeat_age_ms()
Bridge --> WS : 1500
WS --> App : {"type":"comms","heartbeat_age_ms":1500}

== Disconnection ==
App -x WS : WebSocket close
WS -> Bridge : _client_connected = false
Bridge -> Pubs : publish Bool(false) on /app/connected

@enduml
```

---

## 4. System Context Diagram

```plantuml
@startuml
!include <C4/C4_Context>

title System Context — Lawnbot Comms Bridge

System(comms, "lawnbot_comms", "WebSocket ↔ ROS 2 bridge\nws://0.0.0.0:9002\nSingle-client policy")
Person(app, "Mobile App", "Sends commands\nReceives telemetry")
System(autonomy, "mower_autonomy", "Publishes /mower/state\n/mower/telemetry")
System(motors, "lawnbot_motors", "Receives teleop\nvia /app/teleop_cmd")

app --> comms : WebSocket JSON\n(heartbeat, mode, teleop)
comms --> app : WebSocket JSON\n(state, telemetry, comms)
comms --> autonomy : /app/heartbeat\n/app/mode (ROS 2)
comms --> motors : /app/teleop_cmd\n(Twist, ROS 2)
autonomy --> comms : /mower/state\n/mower/telemetry\n(String, ROS 2)

note right of comms
  Threading model:
  asyncio event loop with
  rclpy.spin_once(50ms)
  
  Per client:
  - rx_loop (phone → ROS 2)
  - tx_loop (ROS 2 → phone)
end note

@enduml
```

---

## 5. Detailed Design

### 5.1 Inbound Message Protocol (Phone → Mower)

| Message Type | Fields | ROS 2 Action | Published Topic |
|---|---|---|---|
| `hello` | — | Reply with ACK and version | — |
| `heartbeat` | — | Publish `Empty` | `/app/heartbeat` |
| `mode` | `value: string` | Publish `String` with mode value | `/app/mode` |
| `teleop` | `linear: float, angular: float` | Publish `Twist` | `/app/teleop_cmd` |

### 5.2 Outbound Message Protocol (Mower → Phone)

| Message Type | Source | Trigger |
|---|---|---|
| `ack` | Bridge node | On connection and `hello` |
| `state` | Subscription to `/mower/state` | On ROS 2 message |
| `telemetry` | Subscription to `/mower/telemetry` | On ROS 2 message |
| `comms` | Internal timer | Every 200 ms idle (no queued messages) |
| `error` | Bridge node | On rejected second client |

### 5.3 Threading Model

The bridge uses a cooperative asyncio model with two coroutines per connected client:

- **`rx_loop()`** — Receives JSON from the phone, parses the message type, and calls the appropriate `publish_*()` method on the BridgeNode. Malformed JSON is silently discarded.
- **`tx_loop()`** — Dequeues outgoing messages (enqueued via `call_soon_threadsafe()` from ROS 2 callbacks) and sends them as JSON over WebSocket. If no messages are queued for 200 ms, a `comms` keepalive is sent containing the elapsed time since the last heartbeat.

The main loop alternates between `rclpy.spin_once()` (50 ms timeout) and the asyncio event loop to process both ROS 2 callbacks and WebSocket I/O.

### 5.4 Single-Client Policy

Only one WebSocket client is allowed at a time. The `_client_connected` flag tracks the active connection. If a second client attempts to connect, it receives `{"type":"error","msg":"another client connected"}` and the connection is immediately closed. The flag is reset when the active client disconnects.

### 5.5 Heartbeat Monitoring

The bridge tracks the last heartbeat timestamp from the phone. The `heartbeat_age_ms()` method computes how long ago the last heartbeat was received. This value is included in every `comms` keepalive message, allowing the phone to detect bidirectional link health. Other nodes can also monitor `/app/heartbeat` to detect phone connectivity.

### 5.6 Test Support

A `FakeTelemetryNode` publishes simulated `/mower/state` and `/mower/telemetry` at 5 Hz for testing the bridge without the full mower stack running.
