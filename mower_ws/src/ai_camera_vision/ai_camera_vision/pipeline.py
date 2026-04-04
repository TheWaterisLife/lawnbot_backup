"""DepthAI pipeline for OAK-D Lite camera.

PURPOSE:
    Creates and manages the DepthAI pipeline that runs YOLO detection,
    DeepLabV3+ segmentation, and stereo depth concurrently on OAK-D Lite.

WHAT THIS FILE DOES:
    - create_pipeline(): Builds the DepthAI pipeline with all nodes
    - PipelineRunner: Context manager to run the pipeline and get results
    - InferenceResult/ImageResult: Dataclasses for timing metadata
    - BackpressureMonitor: Tracks dropped frames

KEY CONFIGURATION (lines to edit):
    - YOLO_INPUT_SIZE (line ~418): Change if using different YOLO model size
    - SEG_INPUT_SIZE (line ~419): Change if using different segmentation model
    - CAM_PREVIEW_SIZE (line ~416): Camera preview resolution
    - nn_yolo.setConfidenceThreshold() (line ~504): Detection confidence
    - nn_yolo.setAnchors() (line ~500): YOLO anchor boxes (model-specific)

HOW TO EDIT:
    - To change YOLO model: Update YOLO_INPUT_SIZE and anchor configuration
    - To change segmentation model: Update SEG_INPUT_SIZE
    - To disable depth: Set enable_depth=False in create_pipeline()
    - To adjust queue sizes: Modify setQueueSize() calls

DEPENDENCIES (imports from):
    - depthai: The OAK-D SDK
    - logging, dataclasses, pathlib

USED BY:
    - demo_live_view.py: Creates pipeline and uses PipelineRunner
    - node.py: ROS2 node wrapper (for Raspberry Pi deployment)
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from datetime import timedelta
from pathlib import Path
from typing import TYPE_CHECKING, Any, Callable

import depthai as dai

if TYPE_CHECKING:
    pass

# Module logger
logger = logging.getLogger(__name__)


# =============================================================================
# Timing Metadata (Story 2.3)
# =============================================================================

# Default frame ID per schema v1
DEFAULT_FRAME_ID = "oak_rgb_optical_frame"


def timedelta_to_ns(td: timedelta) -> int:
    """Convert a timedelta to nanoseconds.

    Args:
        td: A datetime.timedelta object.

    Returns:
        Integer nanoseconds.
    """
    return int(td.total_seconds() * 1_000_000_000)


@dataclass(frozen=True)
class InferenceResult:
    """Inference result with timing metadata.

    Attributes:
        raw_data: The original dai.NNData object for accessing tensors.
        timestamp_ns: Device timestamp in nanoseconds since device boot.
        sequence_num: Monotonically increasing sequence number per stream.
        timestamp_source: Source of the timestamp ("device" or "host").
        frame_id: ROS-style frame identifier.

    Stream Correlation:
        Detection and segmentation streams may run at different rates.
        To find results from the same input frame, match by sequence_num.

        Example:
            det = runner.get_detection_result()
            seg = runner.get_segmentation_result()

            if det and seg and det.sequence_num == seg.sequence_num:
                # Same input frame
                process_paired(det, seg)
    """

    raw_data: Any  # dai.NNData - use Any to allow mocking in tests
    timestamp_ns: int
    sequence_num: int
    timestamp_source: str = "device"
    frame_id: str = DEFAULT_FRAME_ID

    @classmethod
    def from_nn_data(
        cls,
        nn_data: Any,
        *,
        frame_id: str = DEFAULT_FRAME_ID,
    ) -> "InferenceResult":
        """Create an InferenceResult from a dai.NNData object.

        Args:
            nn_data: A dai.NNData object from the DepthAI queue.
            frame_id: ROS-style frame identifier.

        Returns:
            An InferenceResult with extracted timing metadata.
        """
        timestamp_td = nn_data.getTimestamp()
        timestamp_ns = timedelta_to_ns(timestamp_td)
        sequence_num = nn_data.getSequenceNum()

        return cls(
            raw_data=nn_data,
            timestamp_ns=timestamp_ns,
            sequence_num=sequence_num,
            timestamp_source="device",
            frame_id=frame_id,
        )


@dataclass(frozen=True)
class ImageResult:
    """Image frame result with timing metadata.

    Attributes:
        raw_frame: The original dai.ImgFrame object.
        timestamp_ns: Device timestamp in nanoseconds since device boot.
        sequence_num: Monotonically increasing sequence number.
        timestamp_source: Source of the timestamp ("device" or "host").
        frame_id: ROS-style frame identifier.
    """

    raw_frame: Any  # dai.ImgFrame - use Any to allow mocking in tests
    timestamp_ns: int
    sequence_num: int
    timestamp_source: str = "device"
    frame_id: str = DEFAULT_FRAME_ID

    @classmethod
    def from_img_frame(
        cls,
        img_frame: Any,
        *,
        frame_id: str = DEFAULT_FRAME_ID,
    ) -> "ImageResult":
        """Create an ImageResult from a dai.ImgFrame object.

        Args:
            img_frame: A dai.ImgFrame object from the DepthAI queue.
            frame_id: ROS-style frame identifier.

        Returns:
            An ImageResult with extracted timing metadata.
        """
        timestamp_td = img_frame.getTimestamp()
        timestamp_ns = timedelta_to_ns(timestamp_td)
        sequence_num = img_frame.getSequenceNum()

        return cls(
            raw_frame=img_frame,
            timestamp_ns=timestamp_ns,
            sequence_num=sequence_num,
            timestamp_source="device",
            frame_id=frame_id,
        )


# =============================================================================
# Backpressure Monitoring (Story 2.4)
# =============================================================================

# Default log interval to avoid spam (log every N drops)
DEFAULT_DROP_LOG_INTERVAL = 10


@dataclass
class QueueStats:
    """Statistics for a single queue stream.

    Tracks frame reception and drop detection via sequence number gaps.

    Attributes:
        stream_name: Name of the stream (e.g., "detections", "segmentation").
        total_received: Total frames successfully received.
        total_dropped: Total frames dropped (detected via sequence gaps).
        last_sequence: Last sequence number seen (-1 if none yet).
        drops_since_last_log: Drops accumulated since last log message.
    """

    stream_name: str
    total_received: int = 0
    total_dropped: int = 0
    last_sequence: int = -1
    drops_since_last_log: int = 0

    @property
    def drop_rate(self) -> float:
        """Calculate drop rate as a percentage.

        Returns:
            Drop rate in range [0.0, 100.0], or 0.0 if no frames processed.
        """
        total = self.total_received + self.total_dropped
        if total == 0:
            return 0.0
        return (self.total_dropped / total) * 100.0

    def update(self, sequence_num: int) -> int:
        """Update stats with a new frame's sequence number.

        Args:
            sequence_num: The sequence number of the received frame.

        Returns:
            Number of frames dropped (0 if no gap detected).
        """
        dropped = 0

        if self.last_sequence >= 0:
            # Detect gaps in sequence numbers
            expected = self.last_sequence + 1
            if sequence_num > expected:
                dropped = sequence_num - expected
                self.total_dropped += dropped
                self.drops_since_last_log += dropped

        self.last_sequence = sequence_num
        self.total_received += 1

        return dropped

    def reset_log_counter(self) -> int:
        """Reset the drops-since-last-log counter.

        Returns:
            The count that was reset.
        """
        count = self.drops_since_last_log
        self.drops_since_last_log = 0
        return count


class BackpressureMonitor:
    """Monitors backpressure across multiple queue streams.

    Tracks drop statistics per stream and logs warnings when drops occur.

    Backpressure Strategy (Story 2.4):
        - All queues use "drop oldest, keep latest" policy
        - Queue sizes are bounded (1 on device, 4 on host)
        - Drops are detected via sequence number gaps
        - Warnings logged periodically (not per-frame) to avoid spam

    Usage:
        monitor = BackpressureMonitor()

        # When receiving a frame:
        result = runner.get_detection_result()
        if result:
            monitor.record_frame("detections", result.sequence_num)

        # Get statistics:
        stats = monitor.get_stats("detections")
        print(f"Drop rate: {stats.drop_rate:.1f}%")
    """

    def __init__(
        self,
        *,
        log_interval: int = DEFAULT_DROP_LOG_INTERVAL,
        log_callback: Callable[[str], None] | None = None,
    ) -> None:
        """Initialize the backpressure monitor.

        Args:
            log_interval: Log a warning every N accumulated drops per stream.
                         Set to 0 to disable logging.
            log_callback: Optional callback for log messages. If None, uses
                         the module logger at WARNING level.
        """
        self._stats: dict[str, QueueStats] = {}
        self._log_interval = log_interval
        self._log_callback = log_callback

    def record_frame(self, stream_name: str, sequence_num: int) -> int:
        """Record a received frame and detect drops.

        Args:
            stream_name: Name of the stream (e.g., "detections").
            sequence_num: Sequence number of the received frame.

        Returns:
            Number of frames dropped since last frame (0 if none).
        """
        if stream_name not in self._stats:
            self._stats[stream_name] = QueueStats(stream_name=stream_name)

        stats = self._stats[stream_name]
        dropped = stats.update(sequence_num)

        # Log if we've accumulated enough drops
        if self._log_interval > 0 and stats.drops_since_last_log >= self._log_interval:
            self._log_drops(stats)

        return dropped

    def _log_drops(self, stats: QueueStats) -> None:
        """Log accumulated drops for a stream."""
        count = stats.reset_log_counter()
        if count == 0:
            return

        msg = (
            f"Backpressure: {count} {stats.stream_name} frame(s) dropped "
            f"(total: {stats.total_dropped}, rate: {stats.drop_rate:.1f}%)"
        )

        if self._log_callback:
            self._log_callback(msg)
        else:
            logger.warning(msg)

    def get_stats(self, stream_name: str) -> QueueStats | None:
        """Get statistics for a specific stream.

        Args:
            stream_name: Name of the stream.

        Returns:
            QueueStats for the stream, or None if not tracked.
        """
        return self._stats.get(stream_name)

    def get_all_stats(self) -> dict[str, QueueStats]:
        """Get statistics for all tracked streams.

        Returns:
            Dictionary mapping stream names to their QueueStats.
        """
        return dict(self._stats)

    def get_total_drops(self) -> int:
        """Get total drops across all streams.

        Returns:
            Sum of total_dropped across all streams.
        """
        return sum(s.total_dropped for s in self._stats.values())

    def get_summary(self) -> str:
        """Get a summary string of all stream statistics.

        Returns:
            Human-readable summary of drop statistics.
        """
        if not self._stats:
            return "No streams tracked"

        lines = ["Backpressure Summary:"]
        for name, stats in sorted(self._stats.items()):
            lines.append(
                f"  {name}: {stats.total_received} received, "
                f"{stats.total_dropped} dropped ({stats.drop_rate:.1f}%)"
            )
        return "\n".join(lines)

    def flush_logs(self) -> None:
        """Force log any accumulated drops (e.g., at shutdown)."""
        for stats in self._stats.values():
            if stats.drops_since_last_log > 0:
                self._log_drops(stats)


# Stream names for XLink outputs
STREAM_DETECTIONS = "detections"
STREAM_SEGMENTATION = "segmentation"
STREAM_RGB_PREVIEW = "rgb_preview"
STREAM_DEPTH = "depth"

# Model input sizes
YOLO_INPUT_SIZE = (416, 416)
SEG_INPUT_SIZE = (256, 256)

# Camera settings
CAM_RESOLUTION = dai.ColorCameraProperties.SensorResolution.THE_1080_P
CAM_FPS = 30
# Use wider aspect ratio for preview (16:9) to maximize horizontal FOV
# The camera will scale down from 1920x1080 without center-cropping
CAM_PREVIEW_SIZE = (640, 360)  # 16:9 aspect ratio, same as native sensor

# Letterboxing: When feeding wider preview to fixed-size NN inputs,
# black bars are added as needed to preserve aspect ratio.
# This ensures full horizontal FOV is processed by the AI models.

# Depth settings (Story 2.5)
DEPTH_RESOLUTION = dai.MonoCameraProperties.SensorResolution.THE_400_P
DEPTH_FPS = 30
DEPTH_MEDIAN_FILTER = dai.StereoDepthProperties.MedianFilter.KERNEL_7x7


def create_pipeline(
    yolo_blob_path: str | Path,
    seg_blob_path: str | Path,
    *,
    enable_rgb_preview: bool = False,
    enable_depth: bool = False,
) -> dai.Pipeline:
    """Create a DepthAI pipeline with dual-model inference.

    Args:
        yolo_blob_path: Path to the YOLOv8n .blob file.
        seg_blob_path: Path to the DeepLabV3+ .blob file.
        enable_rgb_preview: If True, also output RGB preview for visualization.
        enable_depth: If True, enable stereo depth output (Story 2.5).

    Returns:
        Configured dai.Pipeline ready to be started on a device.

    Pipeline Architecture (with depth enabled):
        MonoCamera (Left)  ─┐
                            ├──► StereoDepth ──► XLinkOut (depth)
        MonoCamera (Right) ─┘        │
                                     │ (aligned to RGB)
        ColorCamera (RGB) ──► ImageManip ──► NeuralNetwork (YOLO) ──► XLinkOut
                 │
                 └──► ImageManip ──► NeuralNetwork (Seg) ──► XLinkOut
    """
    pipeline = dai.Pipeline()

    # =========================================================================
    # Color Camera
    # =========================================================================
    cam_rgb = pipeline.create(dai.node.ColorCamera)
    cam_rgb.setResolution(CAM_RESOLUTION)
    cam_rgb.setFps(CAM_FPS)
    cam_rgb.setInterleaved(False)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    cam_rgb.setPreviewSize(*CAM_PREVIEW_SIZE)

    # Set board socket for depth alignment
    cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)

    # =========================================================================
    # YOLO Detection Branch
    # =========================================================================
    # ImageManip to resize for YOLO input with letterboxing
    # This preserves the full horizontal FOV by adding black bars top/bottom
    manip_yolo = pipeline.create(dai.node.ImageManip)
    manip_yolo.initialConfig.setResize(*YOLO_INPUT_SIZE)
    manip_yolo.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888p)
    manip_yolo.initialConfig.setKeepAspectRatio(True)  # Letterbox, don't crop
    manip_yolo.setMaxOutputFrameSize(YOLO_INPUT_SIZE[0] * YOLO_INPUT_SIZE[1] * 3)

    # YOLO Detection Network (on-device decoding)
    # Using YoloDetectionNetwork instead of NeuralNetwork for proper anchor-based decoding
    nn_yolo = pipeline.create(dai.node.YoloDetectionNetwork)
    nn_yolo.setBlobPath(str(yolo_blob_path))
    nn_yolo.setNumInferenceThreads(2)
    nn_yolo.input.setBlocking(False)
    nn_yolo.input.setQueueSize(1)
    
    # YOLO-v3-tiny configuration (COCO 80 classes)
    nn_yolo.setNumClasses(80)
    nn_yolo.setCoordinateSize(4)
    # Anchors for YOLO-v3-tiny (from model config)
    nn_yolo.setAnchors([10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319])
    nn_yolo.setAnchorMasks({"side26": [0, 1, 2], "side13": [3, 4, 5]})
    nn_yolo.setIouThreshold(0.5)
    nn_yolo.setConfidenceThreshold(0.5)

    # Link camera -> manip -> YOLO
    cam_rgb.preview.link(manip_yolo.inputImage)
    manip_yolo.out.link(nn_yolo.input)

    # YOLO output to host (now outputs ImgDetections, not NNData)
    xout_yolo = pipeline.create(dai.node.XLinkOut)
    xout_yolo.setStreamName(STREAM_DETECTIONS)
    xout_yolo.input.setBlocking(False)
    xout_yolo.input.setQueueSize(1)
    nn_yolo.out.link(xout_yolo.input)

    # =========================================================================
    # Segmentation Branch
    # =========================================================================
    # ImageManip to resize for segmentation input with letterboxing
    # This preserves the full horizontal FOV by adding black bars top/bottom
    manip_seg = pipeline.create(dai.node.ImageManip)
    manip_seg.initialConfig.setResize(*SEG_INPUT_SIZE)
    manip_seg.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888p)
    manip_seg.initialConfig.setKeepAspectRatio(True)  # Letterbox, don't crop
    manip_seg.setMaxOutputFrameSize(SEG_INPUT_SIZE[0] * SEG_INPUT_SIZE[1] * 3)

    # Segmentation Neural Network
    nn_seg = pipeline.create(dai.node.NeuralNetwork)
    nn_seg.setBlobPath(str(seg_blob_path))
    nn_seg.setNumInferenceThreads(2)
    nn_seg.input.setBlocking(False)
    nn_seg.input.setQueueSize(1)

    # Link camera -> manip -> Segmentation
    cam_rgb.preview.link(manip_seg.inputImage)
    manip_seg.out.link(nn_seg.input)

    # Segmentation output to host
    xout_seg = pipeline.create(dai.node.XLinkOut)
    xout_seg.setStreamName(STREAM_SEGMENTATION)
    xout_seg.input.setBlocking(False)
    xout_seg.input.setQueueSize(1)
    nn_seg.out.link(xout_seg.input)

    # =========================================================================
    # Optional RGB Preview (for visualization)
    # =========================================================================
    if enable_rgb_preview:
        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName(STREAM_RGB_PREVIEW)
        xout_rgb.input.setBlocking(False)
        xout_rgb.input.setQueueSize(1)
        cam_rgb.preview.link(xout_rgb.input)

    # =========================================================================
    # Stereo Depth (Story 2.5)
    # =========================================================================
    if enable_depth:
        # Left mono camera (CAM_B)
        mono_left = pipeline.create(dai.node.MonoCamera)
        mono_left.setResolution(DEPTH_RESOLUTION)
        mono_left.setFps(DEPTH_FPS)
        mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)

        # Right mono camera (CAM_C)
        mono_right = pipeline.create(dai.node.MonoCamera)
        mono_right.setResolution(DEPTH_RESOLUTION)
        mono_right.setFps(DEPTH_FPS)
        mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)

        # Stereo depth node
        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.initialConfig.setMedianFilter(DEPTH_MEDIAN_FILTER)
        stereo.setLeftRightCheck(True)
        stereo.setExtendedDisparity(False)
        stereo.setSubpixel(False)

        # Align depth to RGB camera
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)

        # Link mono cameras to stereo depth
        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        # Depth output to host
        xout_depth = pipeline.create(dai.node.XLinkOut)
        xout_depth.setStreamName(STREAM_DEPTH)
        xout_depth.input.setBlocking(False)
        xout_depth.input.setQueueSize(1)
        stereo.depth.link(xout_depth.input)

    return pipeline


def create_yolo_only_pipeline(
    yolo_blob_path: str | Path,
    *,
    enable_rgb_preview: bool = False,
    enable_depth: bool = True,
    confidence_threshold: float = 0.5,
) -> dai.Pipeline:
    """Create a lightweight pipeline with YOLO detection only (no segmentation).

    Approximately 2x faster than the dual-model pipeline since all OAK-D SHAVE
    cores are dedicated to a single model.  Designed for real-time obstacle
    avoidance where segmentation data is not needed.

    Args:
        yolo_blob_path: Path to a YOLO .blob file (v3-tiny, v4-tiny, etc.).
                        The model must output COCO 80-class detections at 416x416.
        enable_rgb_preview: If True, output RGB preview for visualization.
        enable_depth: If True, enable stereo depth (needed for distance estimation).
        confidence_threshold: Minimum confidence for YOLO detections (0.0–1.0).

    Returns:
        Configured dai.Pipeline ready to be started on a device.

    Pipeline Architecture:
        MonoCamera (Left)  ─┐
                            ├──► StereoDepth ──► XLinkOut (depth)
        MonoCamera (Right) ─┘
        ColorCamera (RGB) ──► ImageManip ──► YoloDetectionNetwork ──► XLinkOut (detections)

    Streams produced:
        "detections"  — always
        "rgb_preview" — only if enable_rgb_preview=True
        "depth"       — only if enable_depth=True

    Note:
        This pipeline does NOT create a "segmentation" stream.  Do not use
        PipelineRunner (which expects segmentation); use dai.Device directly.
    """
    pipeline = dai.Pipeline()

    # =========================================================================
    # Color Camera
    # =========================================================================
    cam_rgb = pipeline.create(dai.node.ColorCamera)
    cam_rgb.setResolution(CAM_RESOLUTION)
    cam_rgb.setFps(CAM_FPS)
    cam_rgb.setInterleaved(False)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    cam_rgb.setPreviewSize(*CAM_PREVIEW_SIZE)
    cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)

    # =========================================================================
    # YOLO Detection (single model, all shaves available)
    # =========================================================================
    manip_yolo = pipeline.create(dai.node.ImageManip)
    manip_yolo.initialConfig.setResize(*YOLO_INPUT_SIZE)
    manip_yolo.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888p)
    manip_yolo.initialConfig.setKeepAspectRatio(True)
    manip_yolo.setMaxOutputFrameSize(YOLO_INPUT_SIZE[0] * YOLO_INPUT_SIZE[1] * 3)

    nn_yolo = pipeline.create(dai.node.YoloDetectionNetwork)
    nn_yolo.setBlobPath(str(yolo_blob_path))
    nn_yolo.setNumInferenceThreads(2)
    nn_yolo.input.setBlocking(False)
    nn_yolo.input.setQueueSize(1)
    nn_yolo.setNumClasses(80)
    nn_yolo.setCoordinateSize(4)
    nn_yolo.setAnchors([10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319])
    nn_yolo.setAnchorMasks({"side26": [0, 1, 2], "side13": [3, 4, 5]})
    nn_yolo.setIouThreshold(0.5)
    nn_yolo.setConfidenceThreshold(confidence_threshold)

    cam_rgb.preview.link(manip_yolo.inputImage)
    manip_yolo.out.link(nn_yolo.input)

    xout_yolo = pipeline.create(dai.node.XLinkOut)
    xout_yolo.setStreamName(STREAM_DETECTIONS)
    xout_yolo.input.setBlocking(False)
    xout_yolo.input.setQueueSize(1)
    nn_yolo.out.link(xout_yolo.input)

    # =========================================================================
    # Optional RGB Preview
    # =========================================================================
    if enable_rgb_preview:
        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName(STREAM_RGB_PREVIEW)
        xout_rgb.input.setBlocking(False)
        xout_rgb.input.setQueueSize(1)
        cam_rgb.preview.link(xout_rgb.input)

    # =========================================================================
    # Stereo Depth
    # =========================================================================
    if enable_depth:
        mono_left = pipeline.create(dai.node.MonoCamera)
        mono_left.setResolution(DEPTH_RESOLUTION)
        mono_left.setFps(DEPTH_FPS)
        mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)

        mono_right = pipeline.create(dai.node.MonoCamera)
        mono_right.setResolution(DEPTH_RESOLUTION)
        mono_right.setFps(DEPTH_FPS)
        mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)

        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.initialConfig.setMedianFilter(DEPTH_MEDIAN_FILTER)
        stereo.setLeftRightCheck(True)
        stereo.setExtendedDisparity(False)
        stereo.setSubpixel(False)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)

        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        xout_depth = pipeline.create(dai.node.XLinkOut)
        xout_depth.setStreamName(STREAM_DEPTH)
        xout_depth.input.setBlocking(False)
        xout_depth.input.setQueueSize(1)
        stereo.depth.link(xout_depth.input)

    return pipeline


class PipelineRunner:
    """Manages the DepthAI device and pipeline lifecycle.

    Backpressure Strategy (Story 2.4):
        All queues are bounded to prevent unbounded memory growth.
        Drop policy: "drop oldest, keep latest" — real-time systems need
        fresh data, not stale queued data.

        Queue sizes:
        - Device NN input queues: 1 (minimize latency)
        - Device XLinkOut queues: 1 (minimize latency)
        - Host output queues: 4 (small buffer for consumer jitter)

        Drops are detected via sequence number gaps and logged at WARNING
        level. Use get_backpressure_stats() to check drop rates.

    Usage:
        with PipelineRunner(pipeline) as runner:
            while running:
                det = runner.get_detection_result(timeout_ms=100)
                seg = runner.get_segmentation_result(timeout_ms=100)

            # Check drop statistics at end
            print(runner.get_backpressure_summary())
    """

    def __init__(
        self,
        pipeline: dai.Pipeline,
        *,
        force_usb2: bool = False,
        enable_backpressure_logging: bool = True,
        drop_log_interval: int = DEFAULT_DROP_LOG_INTERVAL,
    ) -> None:
        """Initialize the pipeline runner.

        Args:
            pipeline: A configured dai.Pipeline (from create_pipeline).
            force_usb2: Force USB2 (High Speed) mode. Default is False (USB3).
                       USB3 provides higher bandwidth (~10 FPS vs ~3 FPS).
                       Set to True if brownout occurs (camera keeps disconnecting)
                       or when deploying to Raspberry Pi.
            enable_backpressure_logging: If True, log warnings when drops occur.
            drop_log_interval: Log every N accumulated drops (0 to disable).
        """
        self._pipeline = pipeline
        self._force_usb2 = force_usb2
        self._device: dai.Device | None = None
        self._q_detections: Any = None
        self._q_segmentation: Any = None
        self._q_rgb_preview: Any = None
        self._q_depth: Any = None  # Story 2.5: Depth queue

        # Backpressure monitoring (Story 2.4)
        self._backpressure = BackpressureMonitor(
            log_interval=drop_log_interval if enable_backpressure_logging else 0,
        )

    def start(self) -> None:
        """Start the pipeline on the connected OAK device."""
        if self._device is not None:
            return  # Already started

        # Force USB2 mode to avoid brownout reset issues on OAK-D Lite
        if self._force_usb2:
            self._device = dai.Device(self._pipeline, maxUsbSpeed=dai.UsbSpeed.HIGH)
        else:
            self._device = dai.Device(self._pipeline)

        # Get output queues
        self._q_detections = self._device.getOutputQueue(
            name=STREAM_DETECTIONS, maxSize=4, blocking=False
        )
        self._q_segmentation = self._device.getOutputQueue(
            name=STREAM_SEGMENTATION, maxSize=4, blocking=False
        )

        # RGB preview queue (may not exist if not enabled)
        try:
            self._q_rgb_preview = self._device.getOutputQueue(
                name=STREAM_RGB_PREVIEW, maxSize=4, blocking=False
            )
        except Exception:
            self._q_rgb_preview = None

        # Depth queue (Story 2.5, may not exist if not enabled)
        try:
            self._q_depth = self._device.getOutputQueue(
                name=STREAM_DEPTH, maxSize=4, blocking=False
            )
        except Exception:
            self._q_depth = None

    def stop(self) -> None:
        """Stop the pipeline and release the device."""
        # Flush any accumulated drop logs before shutdown
        self._backpressure.flush_logs()

        if self._device is not None:
            self._device.close()
            self._device = None
        self._q_detections = None
        self._q_segmentation = None
        self._q_rgb_preview = None
        self._q_depth = None

    def is_running(self) -> bool:
        """Check if the pipeline is running."""
        return self._device is not None

    def get_detections(self, timeout_ms: int = 100) -> dai.NNData | None:
        """Get detection results from the YOLO queue.

        Args:
            timeout_ms: Timeout in milliseconds. Use 0 for non-blocking.

        Returns:
            NNData object with detection results, or None if timeout.
        """
        if self._q_detections is None:
            return None
        try:
            if timeout_ms == 0:
                return self._q_detections.tryGet()
            return self._q_detections.get()
        except Exception:
            return None

    def get_segmentation(self, timeout_ms: int = 100) -> dai.NNData | None:
        """Get segmentation results from the segmentation queue.

        Args:
            timeout_ms: Timeout in milliseconds. Use 0 for non-blocking.

        Returns:
            NNData object with segmentation results, or None if timeout.
        """
        if self._q_segmentation is None:
            return None
        try:
            if timeout_ms == 0:
                return self._q_segmentation.tryGet()
            return self._q_segmentation.get()
        except Exception:
            return None

    def get_rgb_preview(self, timeout_ms: int = 100) -> dai.ImgFrame | None:
        """Get RGB preview frame (if enabled).

        Args:
            timeout_ms: Timeout in milliseconds.

        Returns:
            ImgFrame object, or None if not available or timeout.
        """
        if self._q_rgb_preview is None:
            return None
        try:
            if timeout_ms == 0:
                return self._q_rgb_preview.tryGet()
            return self._q_rgb_preview.get()
        except Exception:
            return None

    def get_depth_frame(self, timeout_ms: int = 100) -> dai.ImgFrame | None:
        """Get depth frame (if enabled, Story 2.5).

        Returns a depth frame aligned to the RGB camera. Pixel values
        are depth in millimeters.

        Args:
            timeout_ms: Timeout in milliseconds. Use 0 for non-blocking.

        Returns:
            ImgFrame object with depth data (mm), or None if not available.
        """
        if self._q_depth is None:
            return None
        try:
            if timeout_ms == 0:
                return self._q_depth.tryGet()
            return self._q_depth.get()
        except Exception:
            return None

    # =========================================================================
    # Methods with timing metadata (Story 2.3)
    # =========================================================================

    def get_detection_result(
        self,
        timeout_ms: int = 100,
        *,
        frame_id: str = DEFAULT_FRAME_ID,
    ) -> InferenceResult | None:
        """Get detection results with timing metadata.

        This is the preferred method for getting detection data, as it includes
        timestamps and sequence numbers for stream correlation.

        Backpressure: Drops are tracked via sequence number gaps and logged.

        Args:
            timeout_ms: Timeout in milliseconds. Use 0 for non-blocking.
            frame_id: ROS-style frame identifier.

        Returns:
            InferenceResult with detection data and timing, or None if timeout.

        Example:
            result = runner.get_detection_result()
            if result:
                print(f"Sequence: {result.sequence_num}")
                print(f"Timestamp: {result.timestamp_ns} ns")
                # Access raw tensors via result.raw_data
        """
        nn_data = self.get_detections(timeout_ms)
        if nn_data is None:
            return None
        result = InferenceResult.from_nn_data(nn_data, frame_id=frame_id)

        # Track for backpressure monitoring (Story 2.4)
        self._backpressure.record_frame(STREAM_DETECTIONS, result.sequence_num)

        return result

    def get_segmentation_result(
        self,
        timeout_ms: int = 100,
        *,
        frame_id: str = DEFAULT_FRAME_ID,
    ) -> InferenceResult | None:
        """Get segmentation results with timing metadata.

        This is the preferred method for getting segmentation data, as it includes
        timestamps and sequence numbers for stream correlation.

        Backpressure: Drops are tracked via sequence number gaps and logged.

        Args:
            timeout_ms: Timeout in milliseconds. Use 0 for non-blocking.
            frame_id: ROS-style frame identifier.

        Returns:
            InferenceResult with segmentation data and timing, or None if timeout.

        Example:
            result = runner.get_segmentation_result()
            if result:
                print(f"Sequence: {result.sequence_num}")
                print(f"Timestamp: {result.timestamp_ns} ns")
        """
        nn_data = self.get_segmentation(timeout_ms)
        if nn_data is None:
            return None
        result = InferenceResult.from_nn_data(nn_data, frame_id=frame_id)

        # Track for backpressure monitoring (Story 2.4)
        self._backpressure.record_frame(STREAM_SEGMENTATION, result.sequence_num)

        return result

    def get_rgb_preview_result(
        self,
        timeout_ms: int = 100,
        *,
        frame_id: str = DEFAULT_FRAME_ID,
    ) -> ImageResult | None:
        """Get RGB preview frame with timing metadata.

        Backpressure: Drops are tracked via sequence number gaps and logged.

        Args:
            timeout_ms: Timeout in milliseconds.
            frame_id: ROS-style frame identifier.

        Returns:
            ImageResult with frame data and timing, or None if not available.
        """
        img_frame = self.get_rgb_preview(timeout_ms)
        if img_frame is None:
            return None
        result = ImageResult.from_img_frame(img_frame, frame_id=frame_id)

        # Track for backpressure monitoring (Story 2.4)
        self._backpressure.record_frame(STREAM_RGB_PREVIEW, result.sequence_num)

        return result

    def get_depth_result(
        self,
        timeout_ms: int = 100,
        *,
        frame_id: str = DEFAULT_FRAME_ID,
    ) -> ImageResult | None:
        """Get depth frame with timing metadata (Story 2.5).

        Returns depth frame aligned to RGB with timing metadata.
        Pixel values are depth in millimeters (uint16).

        Backpressure: Drops are tracked via sequence number gaps and logged.

        Args:
            timeout_ms: Timeout in milliseconds. Use 0 for non-blocking.
            frame_id: ROS-style frame identifier.

        Returns:
            ImageResult with depth data and timing, or None if not available.

        Example:
            depth_result = runner.get_depth_result()
            if depth_result:
                depth_mm = depth_result.raw_frame.getFrame()  # numpy array in mm
                print(f"Sequence: {depth_result.sequence_num}")
        """
        img_frame = self.get_depth_frame(timeout_ms)
        if img_frame is None:
            return None
        result = ImageResult.from_img_frame(img_frame, frame_id=frame_id)

        # Track for backpressure monitoring (Story 2.4)
        self._backpressure.record_frame(STREAM_DEPTH, result.sequence_num)

        return result

    # =========================================================================
    # Backpressure Statistics (Story 2.4)
    # =========================================================================

    def get_backpressure_stats(self, stream_name: str) -> QueueStats | None:
        """Get backpressure statistics for a specific stream.

        Args:
            stream_name: Name of the stream (e.g., "detections", "segmentation").

        Returns:
            QueueStats for the stream, or None if not tracked.
        """
        return self._backpressure.get_stats(stream_name)

    def get_all_backpressure_stats(self) -> dict[str, QueueStats]:
        """Get backpressure statistics for all streams.

        Returns:
            Dictionary mapping stream names to their QueueStats.
        """
        return self._backpressure.get_all_stats()

    def get_backpressure_summary(self) -> str:
        """Get a human-readable summary of backpressure statistics.

        Returns:
            Multi-line string summarizing drop rates for all streams.
        """
        return self._backpressure.get_summary()

    def get_total_drops(self) -> int:
        """Get total frames dropped across all streams.

        Returns:
            Total number of dropped frames.
        """
        return self._backpressure.get_total_drops()

    def __enter__(self) -> "PipelineRunner":
        self.start()
        return self

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> None:
        self.stop()
