---
title: Test Design (System-Level) — AI Camera Vision System
description: System-level test strategy, risks, and coverage plan mapped to requirements and ADRs
date: 2026-01-12
---

# Test Design (System-Level) — AI Camera Vision System

## Test goals

- Verify functional correctness for **dual-model inference**, **output schema**, and **visualization**.
- Verify non-functional behaviors: stability, backpressure, and deployment portability.
- Make integration safe by validating coordinate conventions and schema versioning.

## Architecturally significant risks

- **R1: Dual-inference performance/latency** (NFR-001)
- **R2: Output misalignment / coordinate ambiguity** (FR-004, ADR-003)
- **R3: Queue/backpressure memory growth** (NFR-002)
- **R4: Hardware/USB instability** (NFR-003)
- **R5: Blob/device compatibility drift across SDK versions** (FR-008)

## Test strategy layers

### Layer 1: Unit tests (host-only)

Focus: schema generation, coordinate transforms, serialization.

- Validate detection schema v1 serialization and required fields (FR-002, ADR-005)
- Validate bbox normalization and pixel conversion helpers (ADR-003)
- Validate JSONL writer behavior (FR-007, ADR-004)

### Layer 2: Contract tests (schema consumers)

Focus: external consumer can parse messages without special casing.

- Create a tiny consumer that reads JSONL and validates:
  - `schema_version` exists and matches expected value
  - required fields exist
  - bbox values are within `[0,1]`

### Layer 3: Hardware-in-the-loop integration tests (OAK required)

Focus: end-to-end pipeline, stream robustness.

- Startup validation (device present, blob present) (FR-008)
- Dual stream availability (FR-001)
- Timing metadata present and monotonic per stream (FR-004)
- Backpressure behavior under load (NFR-002)

### Layer 4: Manual validation scenarios (with visualization)

Focus: correctness “to the eye” and developer ergonomics.

- Visual overlay alignment: boxes land on objects; mask overlay aligns to scene (FR-005)
- Threshold tuning: confidence threshold affects displayed detections predictably (FR-006)

## Coverage mapping

### FR coverage

- **FR-001**: HIL test that both streams produce outputs in a single run
- **FR-002**: unit + contract tests for detection schema
- **FR-003**: unit + contract tests for segmentation schema
- **FR-004**: integration test asserts timestamps/sequence metadata exist and are sane
- **FR-005**: manual validation + smoke test for UI startup/shutdown
- **FR-006**: config parsing test + smoke test for toggles
- **FR-007**: JSONL publisher tests
- **FR-008**: startup failure-mode tests

### ADR coverage

- **ADR-003**: unit tests for normalization and conversion; contract validation checks ranges
- **ADR-004**: JSONL output format tests + replay of captured JSONL for regression
- **ADR-005**: contract test asserts schema_version and rejects missing/unknown versions

## Environments

- **Dev PC (Windows)**: visualization-heavy validation
- **Raspberry Pi**: headless default, with optional visualization

## Pass/fail gates (system-level)

- Clean startup and shutdown behavior
- Stable runtime without unbounded memory growth
- Valid schema outputs with versioning and explicit coordinate conventions
- Repeatable runbook steps for Raspberry Pi


