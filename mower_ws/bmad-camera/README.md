# BMAD for AI Camera Vision System

This folder contains the BMAD Method artifacts for the **AI Camera Vision System** (OAK-D Lite dual-model inference: YOLOv8n detection + DeepLabV3+ segmentation).

## Hardware Setup

- **AI Camera**: Connected via USB.

## Document index (recommended reading order)

- `product-brief.md` (Phase 1, optional): product vision and MVP boundary
- `PRD.md` (Phase 2, required): functional + non-functional requirements
- `architecture.md` (Phase 3, recommended): technical decisions + ADRs
- `epics/` (Phase 3, required for story-driven implementation): epic + story backlog
- `test-design-system.md` (Phase 3, recommended): system-level test strategy and risks

## How to execute BMAD in this repo (Phase 4)

BMAD’s implementation loop is **one story at a time**:

- **Initialize tracking**: run the `sprint-planning` workflow to create `sprint-status.yaml` from the epic files in `epics/`.
- **Prepare next story**: run `create-story` to generate a standalone story file with acceptance criteria + technical notes.
- **Implement**: run `dev-story` using that story file as the source of truth.
- **Review**: run `code-review`, then update `sprint-status.yaml` and mark the story **DONE**.

If you are not using the BMAD CLI, you can still follow the same flow manually:

- Copy the next story from an epic file into `bmad/stories/story-<slug>.md`
- Implement exactly to the acceptance criteria
- Update a simple YAML status file or checklist

## ROS2 code in this repo (Raspberry Pi “sensor node”)

This repo is also a **ROS2 (colcon) workspace**. The ROS2 package you run on the Raspberry Pi is:

- `src/ai_camera_vision/` (ament_python, ROS 2 Jazzy)

You can use it two ways:

- **Clone this repo on the Pi** and build from the repo root with `colcon build`
- **Import into an existing ROS2 workspace** by copying `src/ai_camera_vision/` into `<your_ws>/src/`, then building that workspace

Build/run instructions are in `src/ai_camera_vision/README.md`.

## Project conventions for this repo (recommended)

- **Single source of truth for scope**: `PRD.md` + `architecture.md`
- **Single source of truth for sequencing**: `epics/`
- **Acceptance criteria must be testable**: every story should have clear pass/fail checks


