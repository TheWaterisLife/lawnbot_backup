"""JSONL writer for schema v1 detection and segmentation messages.

This module provides optional file-based logging of vision outputs for
debugging and offline replay. Per ADR-009, this is disabled by default.

Usage:
    writer = JsonlWriter(output_dir="/path/to/logs")
    writer.write_detection(detection_dict)
    writer.write_segmentation(segmentation_dict)
    writer.close()

File format:
    - detections.jsonl: one JSON detection message per line
    - segmentation.jsonl: one JSON segmentation message per line
    - Each line follows schema v1 as defined in bmad/schema-v1.md

Flush behavior:
    Files are flushed after each write for crash-safety during debugging.
"""

from __future__ import annotations

import base64
import json
from pathlib import Path
from typing import Any

import numpy as np


SCHEMA_VERSION = "v1"


def _encode_mask_b64(mask: np.ndarray) -> str:
    """Encode a numpy mask array to base64 string.

    Args:
        mask: 2D numpy array of class IDs (dtype should be uint8).

    Returns:
        Base64-encoded string of the mask bytes (row-major order).
    """
    if mask.dtype != np.uint8:
        mask = mask.astype(np.uint8)
    return base64.b64encode(mask.tobytes()).decode("ascii")


def build_detection_message(
    *,
    timestamp_source: str,
    timestamp_ns: int,
    sequence: int,
    frame_id: str,
    model_name: str,
    model_input_width: int,
    model_input_height: int,
    detections: list[dict[str, Any]],
) -> dict[str, Any]:
    """Build a schema v1 detection message dictionary.

    Args:
        timestamp_source: "device" or "host"
        timestamp_ns: timestamp in nanoseconds
        sequence: monotonically increasing sequence number
        frame_id: ROS-style frame ID (e.g., "oak_rgb_optical_frame")
        model_name: name of the detection model
        model_input_width: model input width in pixels
        model_input_height: model input height in pixels
        detections: list of detection dicts, each with:
            - class_id (int)
            - class_label (str)
            - confidence (float)
            - bbox (dict with x_min, y_min, x_max, y_max normalized)
            - safety_category (str, optional)

    Returns:
        A dictionary matching schema v1 for detections.
    """
    return {
        "schema_version": SCHEMA_VERSION,
        "timestamp": {"source": timestamp_source, "t_ns": timestamp_ns},
        "sequence": sequence,
        "frame_id": frame_id,
        "model": {
            "name": model_name,
            "input_width": model_input_width,
            "input_height": model_input_height,
        },
        "detections": detections,
    }


def build_segmentation_message(
    *,
    timestamp_source: str,
    timestamp_ns: int,
    sequence: int,
    frame_id: str,
    model_name: str,
    model_input_width: int,
    model_input_height: int,
    class_map_version: str,
    mask: np.ndarray,
) -> dict[str, Any]:
    """Build a schema v1 segmentation message dictionary.

    Args:
        timestamp_source: "device" or "host"
        timestamp_ns: timestamp in nanoseconds
        sequence: monotonically increasing sequence number
        frame_id: ROS-style frame ID (e.g., "oak_rgb_optical_frame")
        model_name: name of the segmentation model
        model_input_width: model input width in pixels
        model_input_height: model input height in pixels
        class_map_version: version of the class map (e.g., "v1")
        mask: 2D numpy array of class IDs (height x width)

    Returns:
        A dictionary matching schema v1 for segmentation.
    """
    height, width = mask.shape[:2]
    return {
        "schema_version": SCHEMA_VERSION,
        "timestamp": {"source": timestamp_source, "t_ns": timestamp_ns},
        "sequence": sequence,
        "frame_id": frame_id,
        "model": {
            "name": model_name,
            "input_width": model_input_width,
            "input_height": model_input_height,
        },
        "class_map_version": class_map_version,
        "mask": {
            "encoding": "mono8",
            "width": width,
            "height": height,
            "data_b64": _encode_mask_b64(mask),
        },
    }


class JsonlWriter:
    """Append-only JSONL writer for detection and segmentation messages.

    Attributes:
        output_dir: Path to the output directory.
        enabled: Whether the writer is active.
    """

    def __init__(self, output_dir: str | Path) -> None:
        """Initialize the JSONL writer.

        Args:
            output_dir: Directory where JSONL files will be written.
                        Must exist; writer will be disabled if not found.
        """
        self.output_dir = Path(output_dir)
        self.enabled = False
        self._det_file: Any = None
        self._seg_file: Any = None

        if not self.output_dir.exists():
            # Caller should log this; we just stay disabled.
            return

        if not self.output_dir.is_dir():
            return

        # Open files in append mode.
        try:
            self._det_file = open(
                self.output_dir / "detections.jsonl", "a", encoding="utf-8"
            )
            self._seg_file = open(
                self.output_dir / "segmentation.jsonl", "a", encoding="utf-8"
            )
            self.enabled = True
        except OSError:
            self.close()

    def write_detection(self, message: dict[str, Any]) -> None:
        """Write a detection message to detections.jsonl.

        Args:
            message: A schema v1 detection message dictionary.
        """
        if not self.enabled or self._det_file is None:
            return
        try:
            line = json.dumps(message, separators=(",", ":"))
            self._det_file.write(line + "\n")
            self._det_file.flush()
        except (OSError, TypeError, ValueError):
            # Fail silently for logging; don't crash the node.
            pass

    def write_segmentation(self, message: dict[str, Any]) -> None:
        """Write a segmentation message to segmentation.jsonl.

        Args:
            message: A schema v1 segmentation message dictionary.
        """
        if not self.enabled or self._seg_file is None:
            return
        try:
            line = json.dumps(message, separators=(",", ":"))
            self._seg_file.write(line + "\n")
            self._seg_file.flush()
        except (OSError, TypeError, ValueError):
            # Fail silently for logging; don't crash the node.
            pass

    def close(self) -> None:
        """Close the JSONL files."""
        if self._det_file is not None:
            try:
                self._det_file.close()
            except OSError:
                pass
            self._det_file = None

        if self._seg_file is not None:
            try:
                self._seg_file.close()
            except OSError:
                pass
            self._seg_file = None

        self.enabled = False

    def __enter__(self) -> "JsonlWriter":
        return self

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> None:
        self.close()

