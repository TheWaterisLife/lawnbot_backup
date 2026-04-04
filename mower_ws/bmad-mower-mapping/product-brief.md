# Product Brief: Mower Mapping Subsystem

## Vision

Allow the operator to record precise lawn boundaries by walking the perimeter with the mower, creating a map that the navigation system can follow for autonomous mowing.

## Problem Statement

Before the mower can autonomously mow, it needs to know:
1. **Where the lawn is**: Exact boundary coordinates
2. **Where NOT to go**: Flower beds, driveways, obstacles
3. **Where to start/stop**: Valid mowing area

Manual coordinate entry is impractical. The operator needs a walk-and-record solution.

## Solution

A mapping system that records GPS positions as the operator walks the perimeter:

| Feature | Implementation |
|---------|---------------|
| **GPS Recording** | Subscribe to `/rtk/fix` (RTK accuracy) |
| **Adaptive Sampling** | Distance + heading + time triggers |
| **Path Simplification** | Ramer-Douglas-Peucker algorithm |
| **Loop Closure** | Auto-close when returning to start |
| **Map Storage** | JSON files in `~/mower_ws/maps/` |
| **App Interface** | WebSocket on port 8770 |

## Target Users

1. **Mobile App**: Controls mapping workflow, shows live preview
2. **Navigation System**: Loads saved boundaries for path planning
3. **Boundary Mapper**: Uses boundaries for virtual fence costmap

## Success Metrics

| Metric | Target | Measurement |
|--------|--------|-------------|
| Boundary Accuracy | < 5 cm | RTK positioning |
| Point Sampling | Adaptive | Distance/heading/time |
| Simplification | ~80% reduction | RDP at 0.08m tolerance |
| Loop Closure | Automatic | Within 0.5m of start |
| Save/Load | Reliable | JSON persistence |

## Constraints

- **RTK Required**: Needs RTK-Fixed for cm-level accuracy
- **Walking Speed**: Operator must walk at reasonable pace
- **Minimum Points**: At least 20 points for valid boundary
- **Single Zone**: One boundary per mapping session

## Key Risks

| Risk | Mitigation |
|------|------------|
| GPS dropout during mapping | Auto-pause, resume on fix |
| Not enough points | Minimum 20-point check |
| Non-closed boundary | Auto-close if within 0.5m |
| Complex lawn shapes | Adaptive heading-based sampling |
