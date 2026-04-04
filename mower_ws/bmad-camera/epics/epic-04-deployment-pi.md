---
title: Epic 04 — Raspberry Pi Deployment and Runbook
description: Make the system easy to deploy and operate on Raspberry Pi with OAK-D Lite
date: 2026-01-12
---

# Epic 04 — Raspberry Pi Deployment and Runbook

## Objective

Run the same host application on a Raspberry Pi reliably, with a minimal operational runbook and repeatable setup steps.

## Scope

- Dependency installation on Pi
- Headless run mode as default
- Operational runbook (common failures and fixes)

## Stories

### Story 4.1 — Document Raspberry Pi setup and run commands

- **Priority**: P0
- **Acceptance criteria**
  - [ ] A runbook doc explains environment setup for Raspberry Pi
  - [ ] Runbook includes how to verify the OAK device is detected
  - [ ] Runbook includes known failure modes (USB power, cable quality)

### Story 4.2 — Headless mode is default and stable

- **Priority**: P0
- **Acceptance criteria**
  - [ ] App runs without any GUI dependencies when visualization is off
  - [ ] Output publishing works in headless mode (JSONL files created)
- **Dependencies**
  - Story 1.3
  - Story 2.2

### Story 4.3 — Optional service-style startup

- **Priority**: P1
- **Acceptance criteria**
  - [ ] Provide a documented option to run on boot (service or script)
  - [ ] Logs are captured in a persistent location (documented)
- **Dependencies**
  - Story 4.2

### Story 4.4 — Publish ROS2 status/diagnostics for health monitoring

- **Priority**: P1
- **Acceptance criteria**
  - [ ] Node publishes a periodic status signal (e.g., `/vision/status` or `/diagnostics`)
  - [ ] Status includes at least: FPS (or publish rate), last error state, and whether depth/NN streams are active
  - [ ] Documentation explains how the “brain” or operators can use the status to detect faults
- **Dependencies**
  - Story 2.2


