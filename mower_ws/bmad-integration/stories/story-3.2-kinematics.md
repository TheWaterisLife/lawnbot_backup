# Story 3.2: Differential Drive Kinematics

**Epic**: 3 - Wheel Encoder Odometry  
**Status**: Not Started  
**Priority**: Must  

## User Story

**As a** developer  
**I want** to compute robot velocity from wheel velocities  
**So that** I can publish odometry

## Acceptance Criteria

- [ ] Compute left wheel velocity from encoder count delta
- [ ] Compute right wheel velocity from encoder count delta
- [ ] Compute vx = (v_left + v_right) / 2
- [ ] Compute wz = (v_right - v_left) / track_width
- [ ] Handle configurable wheel_radius and track_width

## Technical Details

### Physical Parameters

| Parameter | Symbol | Value | Notes |
|-----------|--------|-------|-------|
| Track Width | L | 0.24 m | Center-to-center (estimated) |
| Wheel Radius | r | TBD | Measure sprocket |
| Encoder CPR | N | 3960 | Quadrature counts |

### Kinematics Equations

```
# Wheel velocity from encoder counts
v_wheel = (delta_count / cpr) * 2 * pi * wheel_radius / dt

# Robot velocity (differential drive)
vx = (v_left + v_right) / 2
wz = (v_right - v_left) / track_width

# Pose integration (for odometry)
x += vx * cos(theta) * dt
y += vx * sin(theta) * dt
theta += wz * dt
```

### Class Interface

```python
@dataclass
class OdometryConfig:
    track_width: float = 0.24  # meters
    wheel_radius: float = 0.04  # meters (TBD)
    encoder_cpr: int = 3960  # counts per revolution

class DifferentialDriveKinematics:
    def __init__(self, config: OdometryConfig):
        pass
    
    def counts_to_velocity(
        self, 
        delta_counts: int, 
        dt: float
    ) -> float:
        """Convert encoder count delta to linear velocity (m/s)."""
        pass
    
    def wheel_velocities_to_twist(
        self,
        v_left: float,
        v_right: float
    ) -> Tuple[float, float]:
        """Convert wheel velocities to robot (vx, wz)."""
        pass
    
    def twist_to_wheel_velocities(
        self,
        vx: float,
        wz: float
    ) -> Tuple[float, float]:
        """Convert robot twist to wheel velocities (for motor control)."""
        pass
    
    def integrate_pose(
        self,
        x: float, y: float, theta: float,
        vx: float, wz: float,
        dt: float
    ) -> Tuple[float, float, float]:
        """Integrate velocities to update pose."""
        pass
```

## Test File

`Tests/unit/integration/test_odom_3_2_kinematics.py`

## Test Cases

1. Forward motion: equal wheel velocities -> vx > 0, wz = 0
2. Rotation in place: opposite wheel velocities -> vx = 0, wz != 0
3. Curved path: different wheel velocities -> vx > 0, wz != 0
4. Reverse motion: negative velocities -> vx < 0
5. Pose integration: verify x, y, theta updates
6. Inverse kinematics: twist to wheel velocities
7. Zero velocity: no motion

## Definition of Done

- [ ] DifferentialDriveKinematics class implemented
- [ ] All unit tests passing
- [ ] Parameters configurable via config
- [ ] Code reviewed
- [ ] Documented in code

## Notes

> **TODO**: Measure actual wheel radius (sprocket diameter / 2) before final calibration.
