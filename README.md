# lawnbot_backup

Workspace backup: ROS 2 mower stack (`mower_ws`), camera/vision helpers, and related scripts.

## Layout

- **`mower_ws/`** — colcon workspace (source packages: autonomy, mapping, obstacle detection, motors, etc.)
- **`ai_camera_vision/`** — OAK-D / DepthAI pipeline helpers (often used from `mower_ws`)

Build on the robot (after cloning):

```bash
cd mower_ws
source /opt/ros/<distro>/setup.bash
colcon build
source install/setup.bash
```
