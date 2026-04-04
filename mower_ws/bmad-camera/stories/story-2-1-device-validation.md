---
title: Story 2.1 — Device + blob validation on startup
description: Validate OAK device presence and blob file paths before starting the pipeline
date: 2026-01-12
---

# Story 2.1 — Device + blob validation on startup

## Objective

Ensure the system fails fast with actionable error messages if required hardware or model files are missing, rather than crashing mid-operation.

## Scope

- OAK device presence check
- Blob file path validation
- Clear error messages and non-zero exit codes on failure

## Out of scope

- Full DepthAI pipeline creation (Story 2.2)
- Model compatibility validation (input size, etc.)

## Acceptance criteria

- [ ] Startup checks for OAK device presence
- [ ] Startup checks blob file paths exist and are readable
- [ ] On failure, prints actionable error and exits non-zero
- [ ] Validation can be run independently (testable without full ROS2)
- [ ] Validation is called before pipeline creation in the node

## Technical notes

- Use `depthai` library to detect OAK device
- Use `pathlib.Path.exists()` and `Path.is_file()` for blob validation
- Per FR-008 in PRD: "On failure, exit with a clear error message and non-zero status code"
- Validation should be a separate module for testability

## Suggested implementation

1. Create `ai_camera_vision/validation.py`:
   - `validate_oak_device()` — checks for connected OAK device
   - `validate_blob_file(path)` — checks blob file exists and is readable
   - `ValidationError` — custom exception with actionable message

2. Integrate into `node.py`:
   - Call validation in `__init__` before publisher setup
   - On `ValidationError`, log error and call `sys.exit(1)`

## Error message examples

```
ERROR: OAK device not found. Please connect an OAK-D Lite camera via USB.
ERROR: YOLO blob file not found: /path/to/yolov8n_coco_416x416.blob
ERROR: Segmentation blob file not readable: /path/to/deeplab_v3_mnv2_256x256.blob
```

## Definition of Done

- [ ] Acceptance criteria met
- [ ] Unit tests cover validation scenarios
- [ ] Node exits cleanly with non-zero status on validation failure

