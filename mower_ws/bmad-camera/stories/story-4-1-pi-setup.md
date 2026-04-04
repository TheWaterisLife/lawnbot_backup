---
title: Story 4.1 — Document Raspberry Pi setup and run commands
description: Create operational runbook for Raspberry Pi 5 deployment
date: 2026-01-14
epic: epic-04
status: DONE
---

# Story 4.1 — Document Raspberry Pi setup and run commands

## Objective

Create a comprehensive runbook document that explains how to set up and run the AI Camera Vision system on a Raspberry Pi 5.

## Acceptance Criteria

- [ ] A runbook doc explains environment setup for Raspberry Pi
- [ ] Runbook includes how to verify the OAK device is detected
- [ ] Runbook includes known failure modes (USB power, cable quality)

## Technical Design

### Target Platform

| Component | Version |
|-----------|---------|
| Hardware | Raspberry Pi 5 (4GB or 8GB) |
| OS | Ubuntu 24.04 LTS (arm64) |
| ROS 2 | Jazzy Jalisco |
| Python | 3.11+ (system) |
| DepthAI | 2.28.0.0 |

### Runbook Sections

1. **Prerequisites**
   - Hardware requirements
   - OS installation
   - Network setup

2. **ROS 2 Jazzy Installation**
   - Add ROS 2 apt repository
   - Install ros-jazzy-desktop or ros-jazzy-base
   - Environment setup

3. **DepthAI Installation**
   - USB rules for OAK devices
   - pip install depthai
   - Verify installation

4. **Package Installation**
   - Clone repository
   - Build with colcon
   - Source workspace

5. **Device Verification**
   - List connected OAK devices
   - Test camera connection
   - Verify blob files present

6. **Running the Node**
   - Basic launch command
   - Parameter configuration
   - Topic verification

7. **Troubleshooting**
   - USB power issues
   - Cable quality
   - Permission errors
   - Common error messages

### OAK Device Verification Commands

```bash
# List connected OAK devices
python3 -c "import depthai as dai; print(dai.Device.getAllAvailableDevices())"

# Check USB device
lsusb | grep Movidius

# Check dmesg for USB issues
dmesg | tail -20
```

### Known Failure Modes

| Failure | Symptoms | Solution |
|---------|----------|----------|
| USB power brownout | Camera clicks, disconnects | Use powered USB hub or force USB2 mode |
| Bad USB cable | Intermittent disconnects | Use high-quality USB-C cable |
| No USB rules | Permission denied | Install udev rules for OAK |
| Wrong Python | Import errors | Use system Python 3.11 |
| Missing blobs | ValidationError | Copy blob files to correct path |

### USB Power Budget (Pi 5)

The Raspberry Pi 5 USB ports can supply:
- 600mA per port (default)
- 1.6A total across all ports

OAK-D Lite typical draw:
- USB3: ~900mA (may exceed per-port limit)
- USB2: ~500mA (safer)

**Recommendation**: Use `force_usb2=True` on Raspberry Pi, or use a powered USB hub.

## Implementation

### Files Added

- `docs/RUNBOOK_PI.md` — Main runbook document
- `scripts/setup_pi.sh` — Optional setup script
- `bmad/stories/story-4-1-pi-setup.md` (this file)

### Runbook Location

Create `docs/RUNBOOK_PI.md` with full setup instructions.

## Test Plan

### Verification Tests

1. Follow runbook on fresh Pi 5 install
2. Verify all commands work as documented
3. Intentionally trigger failure modes and verify solutions work

## Dependencies

- Story 1.2 (node exists to run)
- Story 2.2 (pipeline works)

## Notes

- Consider creating a Docker container for easier deployment
- Ubuntu 24.04 is preferred over Raspberry Pi OS for ROS 2 compatibility
- Pi 5 has better USB power delivery than Pi 4
- Document both USB2 (stable) and USB3 (fast) options
