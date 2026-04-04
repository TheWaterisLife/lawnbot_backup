# Story 4.7: Motor Diagnostics

## Status: ✅ Complete

## Description

As a mower system, I need to monitor motor health and detect faults (stalls, communication errors) so that I can alert the user and prevent damage.

## Acceptance Criteria

- [x] Real-time motor health monitoring
- [x] Stall detection (motor commanded but not moving)
- [x] MCP23017 communication check
- [x] Publish to ROS2 /diagnostics topic
- [x] Compatible with diagnostic_aggregator

## Technical Implementation

### Files Created
- `src/mower_navigation/mower_navigation/motor_diagnostics.py`

### ROS2 Node: `motor_diagnostics_node`

**Published Topics:**
| Topic | Type | Description |
|-------|------|-------------|
| `/diagnostics` | `DiagnosticArray` | Motor health status |

**Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `update_rate` | `10.0` | Diagnostics update rate (Hz) |

### Health States
```python
class MotorHealth(Enum):
    OK = "ok"           # Normal operation
    WARNING = "warning" # Speed mismatch
    ERROR = "error"     # Stall or communication failure
    STALE = "stale"     # No recent updates
    UNKNOWN = "unknown" # Not yet initialized
```

### Fault Types
```python
class MotorFault(Enum):
    NONE = "none"
    OVERCURRENT = "overcurrent"
    STALL = "stall"
    ENCODER_FAULT = "encoder_fault"
    COMMUNICATION_ERROR = "communication_error"
    TEMPERATURE_HIGH = "temperature_high"
```

### Stall Detection Algorithm
```python
def _check_stall(self, motor_name, current_speed, target_speed, current_time):
    """
    Stall = target speed > threshold but current speed ~= 0
    for longer than stall_duration.
    """
    is_stalling = (
        abs(target_speed) > stall_speed_threshold and
        abs(current_speed) < stall_speed_threshold
    )
    
    if is_stalling:
        stall_duration = current_time - stall_start_time
        return stall_duration > config.stall_duration
    return False
```

### Configuration
```python
@dataclass
class DiagnosticsConfig:
    stall_speed_threshold: float = 0.05   # Speed below this = stall
    stall_pwm_threshold: float = 20.0     # PWM duty above this
    stall_duration: float = 1.0           # Seconds before fault
    update_rate_hz: float = 10.0          # Update frequency
    stale_timeout: float = 2.0            # Seconds before stale
```

### Diagnostic Message Format
```
motor_controller:
  level: OK
  message: "Motor system: ok"

motor_controller/left_motor:
  level: OK
  values:
    - health: ok
    - fault: none
    - current_speed: 0.450
    - target_speed: 0.450
    - pwm_duty: 55.0%
    - is_enabled: true
```

## Usage Example
```python
# Create diagnostics linked to motor controller
diagnostics = MotorDiagnostics(motor_controller=controller)

# Update in main loop
diagnostics.update()
status = diagnostics.get_status()

if status['overall_health'] == MotorHealth.ERROR:
    # Handle error - stop mowing, alert user
    pass
```

## Test Cases

1. Normal operation → OK status
2. Speed mismatch → WARNING
3. Motor stall → ERROR with STALL fault
4. MCP23017 disconnected → ERROR with COMMUNICATION_ERROR
5. No updates for 2+ seconds → STALE

## Dependencies

- Story 4.1: Motor Controller Basics
- Story 4.6: Motor Calibration

## Related Stories

- Story 3.6: Recovery Behaviors (uses diagnostics to trigger recovery)
