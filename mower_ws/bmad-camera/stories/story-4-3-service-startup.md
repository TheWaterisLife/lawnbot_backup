---
title: Story 4.3 — Optional service-style startup
description: Provide systemd service configuration for automatic startup on boot
date: 2026-01-14
epic: epic-04
status: DONE
---

# Story 4.3 — Optional service-style startup

## Objective

Provide a documented way to run the AI Camera Vision node as a systemd service that starts automatically on boot and captures logs persistently.

## Acceptance Criteria

- [ ] Provide a documented option to run on boot (service or script)
- [ ] Logs are captured in a persistent location (documented)
- [ ] Service can be started/stopped/restarted via standard commands
- [ ] Service auto-restarts on crash

## Technical Design

### Systemd Service Unit

Create `/etc/systemd/system/ai-camera-vision.service`:

```ini
[Unit]
Description=AI Camera Vision Node (OAK-D Lite)
After=network.target
Wants=network.target

[Service]
Type=simple
User=pi
Group=pi
WorkingDirectory=/home/pi/capstone
Environment="ROS_DOMAIN_ID=0"

# Source ROS 2 and workspace
ExecStart=/bin/bash -c 'source /opt/ros/jazzy/setup.bash && \
    source /home/pi/capstone/install/setup.bash && \
    ros2 run ai_camera_vision ai_camera_vision_node --ros-args \
    -p yolo_blob_path:=/home/pi/capstone/models/yolov8n_coco_416x416.blob \
    -p seg_blob_path:=/home/pi/capstone/models/deeplab_v3_mnv2_256x256.blob \
    -p visualization_enabled:=false \
    -p jsonl_logging_enabled:=true \
    -p jsonl_output_dir:=/var/log/ai-camera-vision'

Restart=on-failure
RestartSec=5
StandardOutput=journal
StandardError=journal
SyslogIdentifier=ai-camera-vision

[Install]
WantedBy=multi-user.target
```

### Service Management Commands

```bash
# Install service
sudo cp ai-camera-vision.service /etc/systemd/system/
sudo systemctl daemon-reload

# Enable auto-start on boot
sudo systemctl enable ai-camera-vision

# Start/stop/restart
sudo systemctl start ai-camera-vision
sudo systemctl stop ai-camera-vision
sudo systemctl restart ai-camera-vision

# Check status
sudo systemctl status ai-camera-vision

# View logs
journalctl -u ai-camera-vision -f
journalctl -u ai-camera-vision --since "10 minutes ago"
```

### Log Locations

| Log Type | Location | Description |
|----------|----------|-------------|
| systemd journal | `journalctl -u ai-camera-vision` | stdout/stderr from node |
| JSONL detections | `/var/log/ai-camera-vision/detections.jsonl` | Detection records |
| JSONL segmentation | `/var/log/ai-camera-vision/segmentation.jsonl` | Segmentation records |

### Log Directory Setup

```bash
# Create log directory with correct permissions
sudo mkdir -p /var/log/ai-camera-vision
sudo chown pi:pi /var/log/ai-camera-vision

# Optional: Set up log rotation
sudo tee /etc/logrotate.d/ai-camera-vision << EOF
/var/log/ai-camera-vision/*.jsonl {
    daily
    rotate 7
    compress
    delaycompress
    missingok
    notifempty
}
EOF
```

### Alternative: Launch File

For more complex configurations, use a ROS 2 launch file:

```python
# launch/ai_camera_vision.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ai_camera_vision',
            executable='ai_camera_vision_node',
            name='ai_camera_vision',
            parameters=[{
                'visualization_enabled': False,
                'jsonl_logging_enabled': True,
                'jsonl_output_dir': '/var/log/ai-camera-vision',
                'yolo_blob_path': '/home/pi/capstone/models/yolov8n_coco_416x416.blob',
                'seg_blob_path': '/home/pi/capstone/models/deeplab_v3_mnv2_256x256.blob',
            }],
            output='screen',
        ),
    ])
```

## Implementation

### Files Added

- `scripts/ai-camera-vision.service` — systemd unit file
- `scripts/install_service.sh` — installation script
- `launch/ai_camera_vision.launch.py` — ROS 2 launch file
- `bmad/stories/story-4-3-service-startup.md` (this file)

### Docs Updated

- `docs/RUNBOOK_PI.md` — Add service installation section

## Test Plan

### Manual Tests (Raspberry Pi)

1. Install and enable service
2. Reboot Pi, verify service starts automatically
3. Kill node process, verify auto-restart
4. Check journal logs are captured
5. Check JSONL files are created

## Dependencies

- Story 4.1 (Pi setup documented)
- Story 4.2 (headless mode works)

## Notes

- Service runs as user `pi` for USB device access
- `After=network.target` ensures network is up (may need OAK device earlier)
- Consider `After=dev-oakd.device` udev trigger for more reliable startup
- Log rotation prevents disk fill from continuous logging
- `RestartSec=5` prevents rapid restart loops

