# Product Brief: Navigation Subsystem

## Vision

Enable the autonomous lawn mower to systematically cover an entire lawn within user-defined boundaries while avoiding obstacles, using the high-precision pose estimates from the Sensor Integration subsystem and obstacle detection from the AI Camera Vision subsystem.

## Problem Statement

A lawn mower with accurate position knowledge still needs:
1. **Boundary awareness**: Know where it's allowed to mow
2. **Coverage strategy**: Efficient path to cover entire area
3. **Obstacle avoidance**: React to dynamic obstacles (people, pets)
4. **Resumption**: Continue after battery recharge or interruption

## Solution

A navigation stack built on Nav2 that:

| Component | Purpose | Technology |
|-----------|---------|------------|
| Boundary Manager | Record/load mowing zones | JSON storage via mower_mapping |
| Coverage Planner | Generate boustrophedon paths | Custom global planner |
| Obstacle Avoidance | React to camera detections | Nav2 costmap + local planner |
| Motor Interface | Execute velocity commands | BTS7960 H-Bridge via gpiozero (lawnbot_motors) |

## Target Users

1. **End User**: Defines boundaries, schedules mowing
2. **Sensor Integration**: Provides fused pose at 50 Hz
3. **AI Camera Vision**: Provides obstacle detections at 15 Hz
4. **Motor System**: Receives velocity commands

## Integration Points

```
+-------------------+     +-------------------+
| Sensor Integration|     | AI Camera Vision  |
| /odometry/filtered|     | /vision/detections|
+--------+----------+     +---------+---------+
         |                          |
         v                          v
+------------------------------------------+
|           NAVIGATION SUBSYSTEM           |
|                                          |
|  Boundary -> Coverage -> Nav2 -> Motors  |
|                                          |
+------------------------------------------+
                    |
                    v
            +---------------+
            | Motor Drivers |
            +---------------+
```

## Success Metrics

| Metric | Target | Measurement |
|--------|--------|-------------|
| Coverage Efficiency | >= 90% | Measured area / total area |
| Boundary Adherence | +/- 20 cm | Max deviation from boundary |
| Obstacle Avoidance | 100% | No collisions with detected obstacles |
| Session Completion | >= 80% | Completed without manual intervention |

## Constraints

- **Mowing Width**: 30 cm (from specs)
- **Operating Speed**: 0.45 m/s maximum
- **Lawn Area**: Up to 1000 m^2
- **Zones**: Up to 5 separate zones
- **Platform**: Raspberry Pi 5 (shared with other subsystems)

## Key Risks

| Risk | Mitigation |
|------|------------|
| Pose accuracy degradation | Monitor EKF health, pause if uncertain |
| Missed obstacles | Safety margins, conservative speed |
| Complex lawn shapes | Support concave polygons |
| Battery depletion mid-row | Save progress, resume after charge |

## Dependencies

| Subsystem | Required For |
|-----------|--------------|
| Sensor Integration | Fused pose for localization |
| AI Camera Vision | Obstacle detection |
| Motor Control | Velocity execution |

## Timeline

This is Phase 2 of Navigation & Localization development. Depends on Sensor Integration subsystem being complete and validated.
