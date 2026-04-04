# Architecture: Wheel Encoder Subsystem

## 1. System Context

```
+-------------------------------------------------------+
|              WHEEL ENCODER SUBSYSTEM                  |
+-------------------------------------------------------+
|                                                       |
|   +-----------+          GPIO            +----------+ |
|   |  NodeJS / | <----------------------> |  L Enc A | |
|   |  Python   | (17,27,22,4)             |  L Enc B | |
|   +-----------+                          |  R Enc A | |
|        |                                 |  R Enc B | |
|        v                                 +----------+ |
|   /wheel/odom (ROS2 Topic)                            |
|   /tf (Optional)                                      |
+-------------------------------------------------------+
```

## 2. Component Architecture

### 2.1 Package Structure

```
src/wheel_encoder/
├── wheel_encoder/
│   ├── __init__.py
│   ├── wheel_odom_node.py   # Main Node
│   └── drivers/
│       ├── encoder.py       # Encoder Logic
├── launch/
│   └── wheel_odom.launch.py # Launch File
├── package.xml
└── setup.py
```

### 2.2 Class Diagram

```
+----------------------------------+
|         WheelOdomNode            |
+----------------------------------+
| - odometry: WheelOdometry        |
| - left_enc: EncoderDriver        |
| - right_enc: EncoderDriver       |
+----------------------------------+
              | Uses
              v
+----------------------------------+
|          WheelOdometry           |
+----------------------------------+
| + update(): void                 |
| + get_pose(): (x, y, theta)      |
| + get_velocity(): (v, w)         |
+----------------------------------+
```

---

## 3. Data Flow

1.  **Interrupt**: GPIO change triggers callback.
2.  **Decode**: Quadrature logic determines direction (+/- 1 tick).
3.  **Accumulate**: Ticks are summed.
4.  **Compute**: `update()` calculates delta ticks -> delta dist -> delta pose.
5.  **Publish**:
    *   `nav_msgs/Odometry` to `/wheel/odom` (50Hz)
    *   `TF` (optional)

---

## 4. Dependencies

| Library | Purpose |
|---------|---------|
| `gpiozero` / `lgpio` | High-performance GPIO access |
| `rclpy` | ROS2 Client |
