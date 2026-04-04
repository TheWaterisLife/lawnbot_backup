# Epic 1: IMU Integration (MPU-6050 / BNO085)

## Goal
Implement a ROS 2 node that reads IMU data via I2C and publishes standard IMU messages for EKF fusion. Support both MPU-6050 (prototype) and BNO085 (production) via configuration.

## Background

### Dual IMU Strategy
- **MPU-6050**: 6-DOF IMU (accelerometer + gyroscope, no magnetometer) - used for prototype testing ($3)
- **BNO085**: 9-DOF IMU with built-in sensor fusion - used for production ($20)

### Key Design Decision
Since SimpleRTK2B is single-antenna, **yaw/heading comes from the IMU Magnetometer**. The IMU provides:
- Roll and pitch orientation
- Angular velocities (vroll, vpitch, vyaw)
- Linear accelerations (ax, ay)

## Stories

### Story 1.1: MPU-6050 I2C Driver
**As a** developer  
**I want** a Python class that communicates with the MPU-6050 via I2C  
**So that** I can read sensor data for prototype testing

**Acceptance Criteria:**
- [ ] Initialize MPU-6050 with appropriate sensitivity settings
- [ ] Read raw accelerometer (x, y, z) and convert to m/s²
- [ ] Read raw gyroscope (x, y, z) and convert to rad/s
- [ ] Read temperature (for drift compensation reference)
- [ ] Handle I2C errors with retries
- [ ] Support configurable sample rate

**Technical Notes:**
- Use smbus2 library
- MPU-6050 I2C address: 0x68 (AD0 to GND)
- Accel range: ±2g (default), Gyro range: ±250°/s (default)
- Reference: InvenSense MPU-6050 datasheet

---

### Story 1.2: BNO085 I2C Driver
**As a** developer  
**I want** a Python class that communicates with the BNO085 via I2C  
**So that** I can read sensor data for production

**Acceptance Criteria:**
- [ ] Initialize BNO085 in NDOF mode
- [ ] Read quaternion (w, x, y, z)
- [ ] Read angular velocity (x, y, z) in rad/s
- [ ] Read linear acceleration (x, y, z) in m/s²
- [ ] Read calibration status (sys, gyro, accel, mag)
- [ ] Handle I2C errors with retries

**Technical Notes:**
- Use smbus2 library
- BNO085 I2C address: 0x4A (default)
- Reference: Bosch BNO085 datasheet

---

### Story 1.3: IMU Driver Factory
**As a** developer  
**I want** a factory class to instantiate the correct IMU driver  
**So that** switching between IMUs requires only a config change

**Acceptance Criteria:**
- [ ] Factory accepts `imu_type` parameter ("mpu6050" or "bno085")
- [ ] Returns appropriate driver instance
- [ ] Common interface (ImuDriver ABC) for both drivers
- [ ] Raises clear error for unknown IMU type

**Interface:**
```python
class ImuDriver(ABC):
    @abstractmethod
    def initialize(self) -> bool: ...
    
    @abstractmethod
    def read_orientation(self) -> Tuple[float, float, float, float]: ...  # quaternion
    
    @abstractmethod
    def read_gyro(self) -> Tuple[float, float, float]: ...  # rad/s
    
    @abstractmethod
    def read_accel(self) -> Tuple[float, float, float]: ...  # m/s²
    
    @abstractmethod
    def get_status(self) -> dict: ...
```

---

### Story 1.4: Madgwick Filter for MPU-6050
**As a** developer  
**I want** a software orientation filter for MPU-6050  
**So that** I can estimate roll and pitch without built-in fusion

**Acceptance Criteria:**
- [ ] Implement Madgwick filter algorithm
- [ ] Compute roll and pitch from accelerometer + gyroscope
- [ ] Compute yaw if magnetometer is available (Madgwick with mag)
- [ ] Configurable filter gain (beta parameter)
- [ ] Output quaternion with yaw = 0 (identity for yaw component)

**Note:** Yaw stability relies on magnetometer calibration.

---

### Story 1.5: IMU ROS 2 Node
**As a** navigation system  
**I want** IMU data published as sensor_msgs/Imu  
**So that** the EKF can fuse orientation and motion data

**Acceptance Criteria:**
- [ ] Node starts without errors
- [ ] Supports `imu_type` parameter for IMU selection
- [ ] Publishes to /imu/data at 100 Hz
- [ ] Message includes orientation quaternion (roll, pitch, yaw)
- [ ] Message includes angular velocity
- [ ] Message includes linear acceleration
- [ ] Covariance matrices populated appropriately
- [ ] frame_id set to "imu_link"
- [ ] Applies Madgwick filter for MPU-6050, uses built-in fusion for BNO085

**Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| imu_type | "mpu6050" | IMU model: "mpu6050" or "bno085" |
| i2c_bus | 1 | I2C bus number |
| i2c_address | auto | 0x68 for MPU6050, 0x4A for BNO085 |
| publish_rate | 100.0 | Publishing rate in Hz |
| frame_id | "imu_link" | TF frame ID |
| madgwick_beta | 0.1 | Madgwick filter gain (MPU6050 only) |

---

### Story 1.6: IMU Diagnostics
**As an** operator  
**I want** to monitor IMU health and calibration  
**So that** I know if sensor data is reliable

**Acceptance Criteria:**
- [ ] Publish to /diagnostics topic
- [ ] Report IMU type in use
- [ ] For BNO085: Report calibration status (0-3 for each sensor)
- [ ] For MPU-6050: Report connection status and temperature
- [ ] Warn if no data for 500ms
- [ ] Report I2C error rate
- [ ] Log IMU type and status on startup

---

### Story 1.7: IMU Testing
**As a** developer  
**I want** tests for the IMU subsystem  
**So that** I can verify correctness

**Acceptance Criteria:**
- [ ] Unit test for MPU-6050 driver with mock I2C
- [ ] Unit test for BNO085 driver with mock I2C
- [ ] Unit test for Madgwick filter
- [ ] Unit test for quaternion normalization
- [ ] Unit test for IMU factory
- [ ] Integration test with mock I2C
- [ ] Manual test procedure documented for both IMU types

## Definition of Done
- All stories completed
- Both IMU drivers implemented and tested
- IMU publishing at 100 Hz for either IMU type
- Diagnostics reporting status
- Tests passing
- Code reviewed
