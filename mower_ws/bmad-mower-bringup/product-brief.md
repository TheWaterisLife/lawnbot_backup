# Product Brief: Mower Bringup Subsystem

## Vision

Provide a single command to launch the entire autonomous lawn mower system, orchestrating all subsystems in the correct order with proper configuration and health verification.

## Problem Statement

The mower consists of 6+ separate packages that must start in a specific order:
1. GPS must get a fix before sensor fusion starts
2. Sensor fusion must be running before navigation
3. Motors must be ready before teleop commands flow
4. Camera must be active before obstacle avoidance works

Manually launching each package is error-prone and time-consuming.

## Solution

A bringup package with launch files that:

| Launch File | Purpose |
|-------------|---------|
| `full_system.launch.py` | Start everything for autonomous mowing |
| `sensors_only.launch.py` | Sensor testing without navigation |
| `teleop.launch.py` | Manual drive mode |

## Target Users

1. **Operator**: One-command system startup
2. **Developer**: Selective subsystem launching for testing
3. **CI/CD**: Automated integration testing

## Success Metrics

| Metric | Target |
|--------|--------|
| Full startup time | < 30 seconds |
| All topics active | Within 60 seconds |
| RTK fix achieved | Within 120 seconds |
| Single command launch | Yes |

## Constraints

- **Platform**: Raspberry Pi 5
- **ROS2**: Jazzy
- **Dependencies**: All subsystem packages must be built
- **Network**: Phone hotspot for NTRIP and app connectivity

## Key Risks

| Risk | Mitigation |
|------|------------|
| Subsystem fails to start | Health checks, fallback launch |
| Wrong startup order | Explicit launch sequencing |
| GPIO conflicts | Pin assignment documented across packages |
| Resource contention | CPU/RAM budgets per subsystem |

## Startup Dependencies

```
RTK Reader ──────────┐
                     ├──> Sensor Integration ──> Navigation
Camera Vision ───────┤
                     ├──> Comms Bridge
Motor Server ────────┘
```
