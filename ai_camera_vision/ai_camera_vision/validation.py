"""Startup validation for OAK device and model files.

This module provides validation functions that run before the DepthAI pipeline
is created. Per FR-008, the system should fail fast with actionable errors.

Usage:
    from ai_camera_vision.validation import (
        validate_oak_device,
        validate_blob_file,
        validate_startup,
        ValidationError,
    )

    try:
        validate_startup(yolo_blob_path="/path/to/yolo.blob", seg_blob_path="/path/to/seg.blob")
    except ValidationError as e:
        print(f"ERROR: {e}")
        sys.exit(1)
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from typing import Sequence


class ValidationError(Exception):
    """Raised when startup validation fails.

    Attributes:
        message: Human-readable error message with actionable guidance.
    """

    def __init__(self, message: str) -> None:
        self.message = message
        super().__init__(message)


def validate_oak_device(timeout_sec: float = 3.0) -> dict:
    """Check that an OAK device is connected and accessible.

    Args:
        timeout_sec: How long to wait for device discovery.

    Returns:
        Device info dict with 'mxid' and 'name' keys.

    Raises:
        ValidationError: If no OAK device is found.
    """
    try:
        import depthai as dai
    except ImportError as e:
        raise ValidationError(
            "DepthAI library not installed. Install with: pip install depthai"
        ) from e

    # Get available devices
    devices = dai.Device.getAllAvailableDevices()

    if not devices:
        raise ValidationError(
            "OAK device not found. Please connect an OAK-D Lite camera via USB 3.0 "
            "and ensure no other application is using it."
        )

    # Return info about the first device
    device_info = devices[0]
    return {
        "mxid": device_info.getDeviceId(),
        "name": device_info.name,
        "state": str(device_info.state),
    }


def validate_blob_file(path: str | Path, name: str = "Blob") -> Path:
    """Check that a blob file exists and is readable.

    Args:
        path: Path to the .blob file.
        name: Human-readable name for error messages (e.g., "YOLO blob").

    Returns:
        Resolved Path object.

    Raises:
        ValidationError: If the file doesn't exist or isn't readable.
    """
    if not path:
        raise ValidationError(
            f"{name} file path is empty. Please provide a valid path to the .blob file."
        )

    blob_path = Path(path)

    if not blob_path.exists():
        raise ValidationError(
            f"{name} file not found: {blob_path.absolute()}\n"
            f"Please ensure the file exists or provide the correct path."
        )

    if not blob_path.is_file():
        raise ValidationError(
            f"{name} path is not a file: {blob_path.absolute()}\n"
            f"Expected a .blob file, got a directory or invalid path."
        )

    # Check if readable by attempting to open
    try:
        with open(blob_path, "rb") as f:
            # Read first few bytes to verify it's accessible
            header = f.read(16)
            if len(header) == 0:
                raise ValidationError(
                    f"{name} file is empty: {blob_path.absolute()}\n"
                    f"Please provide a valid compiled neural network blob."
                )
    except PermissionError:
        raise ValidationError(
            f"{name} file is not readable (permission denied): {blob_path.absolute()}\n"
            f"Please check file permissions."
        )
    except OSError as e:
        raise ValidationError(
            f"{name} file cannot be read: {blob_path.absolute()}\n"
            f"Error: {e}"
        )

    return blob_path.resolve()


def validate_startup(
    *,
    yolo_blob_path: str | Path,
    seg_blob_path: str | Path,
    check_device: bool = True,
) -> dict:
    """Run all startup validations.

    This is the main entry point for validation. Call this before creating
    the DepthAI pipeline.

    Args:
        yolo_blob_path: Path to the YOLO detection model blob.
        seg_blob_path: Path to the segmentation model blob.
        check_device: Whether to check for OAK device (can be disabled for testing).

    Returns:
        Dict with validation results:
        - 'device': Device info dict (if check_device=True)
        - 'yolo_blob': Resolved path to YOLO blob
        - 'seg_blob': Resolved path to segmentation blob

    Raises:
        ValidationError: If any validation fails.
    """
    result: dict = {}

    # Device check (optional, can be skipped for testing without hardware)
    if check_device:
        result["device"] = validate_oak_device()

    # Blob file checks
    result["yolo_blob"] = validate_blob_file(yolo_blob_path, "YOLO blob")
    result["seg_blob"] = validate_blob_file(seg_blob_path, "Segmentation blob")

    return result


def format_validation_success(result: dict) -> str:
    """Format validation results for logging.

    Args:
        result: Dict returned by validate_startup().

    Returns:
        Human-readable success message.
    """
    lines = ["Startup validation passed:"]

    if "device" in result:
        device = result["device"]
        lines.append(f"  - OAK device: {device.get('name', 'unknown')} ({device.get('mxid', 'unknown')})")

    if "yolo_blob" in result:
        lines.append(f"  - YOLO blob: {result['yolo_blob']}")

    if "seg_blob" in result:
        lines.append(f"  - Segmentation blob: {result['seg_blob']}")

    return "\n".join(lines)

