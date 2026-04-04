# Camera Testing Guide

Quick guide to test the OAK-D Lite camera on Raspberry Pi.

---

## Prerequisites

1. OAK-D Lite connected via USB
2. Virtual environment activated: `source ~/ai_camera_vision/venv/bin/activate`
3. udev rules installed (see Setup below)

---

## Setup (First Time Only)

```bash
# Install udev rules for camera access
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

# Unplug and replug the camera
```

---

## Test 1: Basic Camera Connection

```bash
cd ~/ai_camera_vision
source venv/bin/activate

python3 -c "import depthai as dai; print('DepthAI version:', dai.__version__); d = dai.Device(); print('Camera found:', d.getMxId())"
```

**Expected output:**
```
DepthAI version: 2.28.0.0
Camera found: 14442C1091F8D3D700
```

---

## Test 2: Run Headless Demo

```bash
cd ~/ai_camera_vision
source venv/bin/activate
python3 demo_headless.py
```

**Expected output:**
```
================================================================================
  📡 ROS2 TOPIC DATA — What the Raspberry Pi Receives
  Timestamp: 1.234s | Sequence: 42
================================================================================

┌─ /vision/detections (Detection2DArray)
│
│  [0] person
│      confidence: 85.0%
│      bbox: x=[0.123, 0.456] y=[0.200, 0.800]
│      safety: human
│
├─ /vision/detections_3d (Detection3DArray)
│
│  [0] person — Distance: 1.23m ✓ VALID
│      position: x=+0.12m, y=+0.05m, z=1.23m
...
```

Press `Ctrl+C` to stop.

---

## Test 3: Check FPS

Watch the output for FPS stats:
```
├─ /vision/status (JSON String)
│
│  {
│    "status": "OK",
│    "detection_fps": 15.2,
│    "segmentation_fps": 12.8,
│    "depth_fps": 25.0
│  }
```

**Expected FPS (USB3):** 10-25 FPS
**Expected FPS (USB2):** 3-10 FPS

---

## Troubleshooting

### Camera not found
```bash
# Check USB connection
lsusb | grep Luxonis
# Should show: Luxonis Holding OAK-D Lite
```

### Permission denied
```bash
# Reinstall udev rules
sudo udevadm control --reload-rules && sudo udevadm trigger
# Unplug and replug camera
```

### Camera resets / brownout
```bash
# Use USB2 mode (lower power)
# Edit demo_headless.py line ~207:
# Change: force_usb2=False
# To:     force_usb2=True
```

### Low FPS
- Check if using USB3 port (blue inside)
- Close other applications
- Try USB2 mode for stability

---

## USB Mode

| Mode | Setting | FPS | Power |
|------|---------|-----|-------|
| USB3 | `force_usb2=False` | 15-25 | Higher |
| USB2 | `force_usb2=True` | 5-10 | Lower |

Use USB3 for best performance. Switch to USB2 if you see random disconnects.
