from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import Detection3DArray

from ai_camera_vision.jsonl_writer import JsonlWriter
from ai_camera_vision.validation import (
    ValidationError,
    format_validation_success,
    validate_startup,
)


@dataclass(frozen=True)
class NodeConfig:
    visualization_enabled: bool
    yolo_blob_path: str
    seg_blob_path: str
    detections_topic: str
    detections_3d_topic: str
    segmentation_topic: str
    zone_summary_topic: str
    jsonl_logging_enabled: bool
    jsonl_output_dir: str
    skip_device_validation: bool


class AiCameraVisionNode(Node):
    def __init__(self) -> None:
        super().__init__("ai_camera_vision")

        self.declare_parameter("visualization_enabled", False)
        self.declare_parameter("yolo_blob_path", "")
        self.declare_parameter("seg_blob_path", "")
        self.declare_parameter("detections_topic", "/vision/detections")
        self.declare_parameter("detections_3d_topic", "/vision/detections_3d")
        self.declare_parameter("segmentation_topic", "/vision/segmentation/mask")
        self.declare_parameter("zone_summary_topic", "/vision/segmentation/zone_summary")
        # JSONL logging is disabled by default per ADR-009.
        self.declare_parameter("jsonl_logging_enabled", False)
        self.declare_parameter("jsonl_output_dir", ".")
        # Device validation can be skipped for testing without hardware.
        self.declare_parameter("skip_device_validation", False)

        cfg = NodeConfig(
            visualization_enabled=bool(
                self.get_parameter("visualization_enabled").get_parameter_value().bool_value
            ),
            yolo_blob_path=str(
                self.get_parameter("yolo_blob_path").get_parameter_value().string_value
            ),
            seg_blob_path=str(
                self.get_parameter("seg_blob_path").get_parameter_value().string_value
            ),
            detections_topic=str(
                self.get_parameter("detections_topic").get_parameter_value().string_value
            ),
            detections_3d_topic=str(
                self.get_parameter("detections_3d_topic").get_parameter_value().string_value
            ),
            segmentation_topic=str(
                self.get_parameter("segmentation_topic").get_parameter_value().string_value
            ),
            zone_summary_topic=str(
                self.get_parameter("zone_summary_topic").get_parameter_value().string_value
            ),
            jsonl_logging_enabled=bool(
                self.get_parameter("jsonl_logging_enabled").get_parameter_value().bool_value
            ),
            jsonl_output_dir=str(
                self.get_parameter("jsonl_output_dir").get_parameter_value().string_value
            ),
            skip_device_validation=bool(
                self.get_parameter("skip_device_validation").get_parameter_value().bool_value
            ),
        )
        self._cfg = cfg

        # Run startup validation (FR-008: fail fast with actionable errors).
        self._validation_result: dict | None = None
        if cfg.yolo_blob_path and cfg.seg_blob_path:
            try:
                self._validation_result = validate_startup(
                    yolo_blob_path=cfg.yolo_blob_path,
                    seg_blob_path=cfg.seg_blob_path,
                    check_device=not cfg.skip_device_validation,
                )
                self.get_logger().info(format_validation_success(self._validation_result))
            except ValidationError as e:
                self.get_logger().error(f"Startup validation failed: {e.message}")
                raise SystemExit(1) from e

        # Publishers (placeholders; actual publishing happens in later stories).
        self._detections_pub = self.create_publisher(Detection2DArray, cfg.detections_topic, 10)
        self._detections_3d_pub = self.create_publisher(
            Detection3DArray, cfg.detections_3d_topic, 10
        )
        self._segmentation_pub = self.create_publisher(Image, cfg.segmentation_topic, 10)
        self._zone_summary_pub = self.create_publisher(
            Float32MultiArray, cfg.zone_summary_topic, 10
        )

        # JSONL writer (optional debug logging per ADR-009).
        self._jsonl_writer: JsonlWriter | None = None
        if cfg.jsonl_logging_enabled:
            self._jsonl_writer = JsonlWriter(cfg.jsonl_output_dir)
            if self._jsonl_writer.enabled:
                self.get_logger().info(
                    f"JSONL logging enabled -> '{cfg.jsonl_output_dir}'"
                )
            else:
                self.get_logger().error(
                    f"JSONL logging requested but output directory not found or not writable: "
                    f"'{cfg.jsonl_output_dir}'"
                )
                self._jsonl_writer = None

        self.get_logger().info(
            "ai_camera_vision node started "
            f"(visualization_enabled={cfg.visualization_enabled}, "
            f"detections_topic='{cfg.detections_topic}', "
            f"detections_3d_topic='{cfg.detections_3d_topic}', "
            f"segmentation_topic='{cfg.segmentation_topic}', "
            f"zone_summary_topic='{cfg.zone_summary_topic}', "
            f"jsonl_logging={cfg.jsonl_logging_enabled}, "
            f"skip_device_validation={cfg.skip_device_validation})"
        )

        # Warn if blob paths are empty (validation skipped in this case).
        if not cfg.yolo_blob_path or not cfg.seg_blob_path:
            self.get_logger().warn(
                "Blob paths not fully configured. Running in skeleton mode without validation. "
                "Set yolo_blob_path and seg_blob_path to enable full pipeline."
            )

    def log_detection(self, message: dict) -> None:
        """Write a detection message to JSONL if logging is enabled."""
        if self._jsonl_writer is not None:
            self._jsonl_writer.write_detection(message)

    def log_segmentation(self, message: dict) -> None:
        """Write a segmentation message to JSONL if logging is enabled."""
        if self._jsonl_writer is not None:
            self._jsonl_writer.write_segmentation(message)

    def destroy_node(self) -> None:
        """Clean up resources before node shutdown."""
        if self._jsonl_writer is not None:
            self._jsonl_writer.close()
            self._jsonl_writer = None
        super().destroy_node()


def main(args: list[str] | None = None) -> int:
    """Main entry point for the ai_camera_vision node.

    Returns:
        Exit code: 0 for success, 1 for validation failure.
    """
    rclpy.init(args=args)
    node: AiCameraVisionNode | None = None
    exit_code = 0
    try:
        node = AiCameraVisionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Normal shutdown path.
        pass
    except SystemExit as e:
        # Validation failure - exit with non-zero code.
        exit_code = e.code if isinstance(e.code, int) else 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())


