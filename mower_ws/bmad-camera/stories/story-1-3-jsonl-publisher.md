---
title: Story 1.3 — JSONL publisher for schema v1 (optional debug logging)
description: Enable optional file-based logging of detections and segmentation for debugging and replay
date: 2026-01-12
---

# Story 1.3 — JSONL publisher for schema v1 (optional debug logging)

## Objective

Add optional JSONL file logging capability so that detection and segmentation outputs can be written to files for debugging, replay, and offline analysis.

## Scope

- JSONL writer module that produces schema v1-compliant JSON objects
- Integration with the ROS2 node via a configurable parameter
- Append-only files with documented flush behavior

## Out of scope

- Log rotation (can be added later)
- Compression

## Acceptance criteria

- [ ] Host app can write detection messages to `detections.jsonl`
- [ ] Host app can write segmentation messages to `segmentation.jsonl`
- [ ] Files are append-only and flush periodically or on each line (document behavior)
- [ ] JSONL logging is **disabled by default** per ADR-009
- [ ] A ROS parameter `jsonl_logging_enabled` (bool) enables/disables logging
- [ ] A ROS parameter `jsonl_output_dir` (string) configures the output directory
- [ ] Each JSON line includes all schema v1 fields: `schema_version`, `timestamp`, `sequence`, `frame_id`, `model`, and payload
- [ ] Segmentation mask `data_b64` is base64-encoded per schema v1

## Technical notes

- Follow `bmad/schema-v1.md` for message structure
- Per ADR-009: ROS2 logging is the default; JSONL is opt-in for development/replay
- Flush behavior: flush after each write to ensure crash-safety for debugging
- File paths should be constructed as `<jsonl_output_dir>/detections.jsonl` and `<jsonl_output_dir>/segmentation.jsonl`
- If the output directory does not exist, log an error and disable JSONL logging gracefully

## Suggested implementation

1. Create `ai_camera_vision/jsonl_writer.py`:
   - `JsonlWriter` class with `write_detection()` and `write_segmentation()` methods
   - Handles file open/close lifecycle
   - Base64 encoding for mask data

2. Integrate into `node.py`:
   - Declare parameters `jsonl_logging_enabled` and `jsonl_output_dir`
   - Initialize `JsonlWriter` if enabled
   - Provide methods that downstream processing can call to log messages

## Definition of Done

- [ ] Acceptance criteria met
- [ ] README updated with JSONL parameters
- [ ] Code is testable (writer can be used standalone)


