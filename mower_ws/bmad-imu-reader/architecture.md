# Architecture: IMU Reader Subsystem

## 1. System Context

```
+-------------------------------------------------------+
|                IMU READER SUBSYSTEM                   |
+-------------------------------------------------------+
|                                                       |
|   +-----------+        I2C Bus 1         +----------+ |
|   |  NodeJS / | <----------------------> |  MPU6050 | |
|   |  Python   |        (GPIO 2,3)        |  BNO085  | |
|   +-----------+                          +----------+ |
|        |                                              |
|        v                                              |
|   /imu/data (ROS2 Topic)                              |
|   /diagnostics (ROS2 Topic)                           |
+-------------------------------------------------------+
```

## 2. Component Architecture

### 2.1 Package Structure

```
src/imu_reader/
├── imu_reader/
│   ├── __init__.py
│   ├── imu_node.py          # Main Node
│   ├── drivers/
│   │   ├── imu_factory.py   # Factory Method
│   │   ├── mpu6050.py       # Driver
│   │   └── bno085.py        # Driver
│   └── utils/
│       └── filters.py       # Helper Filters
├── launch/
│   └── imu.launch.py        # Launch File
├── package.xml
└── setup.py
```

### 2.2 Class Diagram

```
+----------------------------------+
|            IMUFactory            |
+----------------------------------+
| + create(type, bus, addr): ImuDr |
+----------------------------------+
              | Creates
              v
+----------------------------------+       +-------------------+
|          ImuDriver (ABC)         | <|--- |   Mpu6050Driver   |
+----------------------------------+       +-------------------+
| + initialize(): bool             |       | - smbus           |
| + read_quaternion(): [w,x,y,z]   |       | - MadgwickFilter  |
| + read_gyro(): [x,y,z]           |       +-------------------+
| + read_accel(): [x,y,z]          |
+----------------------------------+       +-------------------+
                                     <|--- |   Bno085Driver    |
                                           +-------------------+
                                           | - adafruit_bno08x |
                                           +-------------------+
```

---

## 3. Data Flow

1.  **Initialize**: `ImuNode` calls `IMUFactory.create("imu_type")`.
2.  **Poll**: Timer triggers `_publish_callback` at `publish_rate`.
3.  **Read**: Driver reads I2C registers (and applies Madgwick for MPU6050).
4.  **Publish**:
    *   `sensor_msgs/Imu` to `/imu/data`
    *   `diagnostic_msgs/DiagnosticArray` to `/diagnostics` (1Hz)

---

## 4. Dependencies

| Library | Purpose |
|---------|---------|
| `smbus2` | Low-level I2C for MPU6050 |
| `adafruit-circuitpython-bno08x` | Driver for BNO085 |
| `rclpy` | ROS2 Client |
