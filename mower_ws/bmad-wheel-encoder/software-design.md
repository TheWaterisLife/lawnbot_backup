# Wheel Encoder Subsystem: Software Design

> **Note:** The `wheel_encoder` package is deprecated. Encoder reading is now actively performed by `mower_autonomy/encoder_node.py`. This design document reflects the active encoder implementation.

## 1. System / Component Diagram

This diagram shows the relationship between the physical encoders, the ROS 2 node, and its outputs.

```plantuml
@startuml
!include <C4/C4_Context>

title System Context — Wheel Encoder Subsystem

System_Ext(gpio_left, "Left Encoder (A/B)", "Hardware Interrupts\nGPIO 17, 27")
System_Ext(gpio_right, "Right Encoder (A/B)", "Hardware Interrupts\nGPIO 22, 4")
System(encoder, "encoder_node", "ROS 2 Node (mower_autonomy)\nTracks quadrature phases\nPublishes absolute & delta ticks")
System_Ext(ros2, "ROS 2 Topics", "/encoders/ticks\n/encoders/delta")

gpio_left --> encoder : Phase A/B edges
gpio_right --> encoder : Phase A/B edges
encoder --> ros2 : Publishes tick counts @ 50 Hz

note bottom of encoder
  Hardware Context:
  - FIT0403 hall-effect quadrature
  - Uses gpiozero edge detection
  - Interrupts run independent of ROS 2
end note
@enduml
```

### Component Details
- **encoder_node**: Directly interfaces with FIT0403 hall-effect quadrature encoders via hardware interrupts (`gpiozero`). Converts rapid phase changes into continuous tick counts.
- **/encoders/ticks**: Publishes the absolute running tick count (Int32MultiArray) at 50 Hz. Used by `localization_node` to compute distance traveled and by `autonomy_node` for precise 180° turns.
- **/encoders/delta**: Publishes tick changes since the last cycle. 

---

## 2. Class Diagram

```plantuml
@startuml
!theme toy

class EncoderNode {
    - publish_rate: float
    - invert_left: bool
    - invert_right: bool
    - left_enc: QuadratureEncoderDriver
    - right_enc: QuadratureEncoderDriver
    + timer_callback()
    + cleanup()
}

class QuadratureEncoderDriver {
    - pin_a: int
    - pin_b: int
    - count: int
    - last_state: int
    + __init__(pin_a, pin_b)
    - _isr_a_change()
    - _isr_b_change()
    + get_ticks(): int
    + reset()
}

EncoderNode *-- "2" QuadratureEncoderDriver : manages

@enduml
```

### Class Details
- **EncoderNode**: Subclasses `rclpy.node.Node`, running a fixed 50 Hz control timer that triggers data publishing.
- **QuadratureEncoderDriver**: A low-level interrupt handler. Uses edge-triggered callbacks to evaluate the phase relationship between Channel A and B, accurately tracking forward and reverse rotation. 

---

## 3. Sequence Diagram

This sequence diagram illustrates a single execution loop of the encoder node reading hardware ticks and publishing them.

```plantuml
@startuml
!theme toy

actor "Wheel Rotation" as Wheel
participant "QuadratureEncoderDriver" as Driver
participant "EncoderNode (50Hz)" as Node
participant "/encoders/ticks" as Ticks

loop Continuous Hardware Interrupts
    Wheel -> Driver : Pin A Rises
    Driver -> Driver : _isr_a_change()
    Driver -> Driver : Update internal count (+1 or -1)
    
    Wheel -> Driver : Pin B Rises
    Driver -> Driver : _isr_b_change()
    Driver -> Driver : Update internal count
end

loop Timer Callback (Every 20ms)
    Node -> Driver : left_enc.get_ticks()
    Driver --> Node : count_l
    Node -> Driver : right_enc.get_ticks()
    Driver --> Node : count_r
    
    Node -> Node : Calculate deltas
    
    Node -> Ticks : Publish [count_l, count_r]
end

@enduml
```

### Sequence Flow Details
- **Interrupt Mode**: Ticks are accumulated independently of the ROS 2 event loop via fast C-level `gpiozero` edge interrupts. This ensures no ticks are missed even under heavy CPU load.
- **Publish Mode**: Every 20ms, the ROS 2 node samples the latest tick count and calculates the delta from the last sample, providing precise 50 Hz data points downstream constraints.
