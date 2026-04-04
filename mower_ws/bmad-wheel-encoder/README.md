# BMAD: Wheel Encoder Subsystem

## Overview

The **Wheel Encoder** subsystem provides essential dead-reckoning odometry for the autonomous mower by reading quadrature encoders on the drive wheels.

## Scope

**In Scope:**
- Reading GPIO interrupts (Left/Right, A/B)
- Differential Drive Kinematics (Odometry)
- Publishing `/wheel/odom`
- Configuring Track Width and Wheel Radius

**Out of Scope:**
- Motor Control (PWM) - See [bmad-navigation](../bmad-navigation/README.md)
- Sensor Fusion (EKF) - See [bmad-integration](../bmad-integration/README.md)

## BMAD Documents

| Document | Status | Description |
|----------|--------|-------------|
| [product-brief.md](product-brief.md) | ✅ Done | High-level vision |
| [PRD.md](PRD.md) | ✅ Done | Requirements & acceptance criteria |
| [architecture.md](architecture.md) | ✅ Done | Technical design |

## Quick Start

```bash
# Install Dependencies
sudo pip3 install --break-system-packages gpiozero lgpio

# Build
cd ~/mower_ws
colcon build --packages-select wheel_encoder

# Run
ros2 launch wheel_encoder wheel_odom.launch.py
```

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/wheel/odom` | `nav_msgs/Odometry` | Position & Velocity |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Health Status |

## Use Cases

1.  **Dead Reckoning**: Estimating position between GPS fixes.
2.  **Velocity Control**: Providing feedback loop for motor speed.
3.  **Slip Detection**: Identifying traction loss.
