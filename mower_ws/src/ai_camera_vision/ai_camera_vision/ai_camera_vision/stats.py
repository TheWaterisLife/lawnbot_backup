"""Runtime statistics collection and logging.

PURPOSE:
    Tracks FPS, latency, and frame drop rates for all streams.
    Used for performance monitoring and health status.

WHAT THIS FILE DOES:
    - StreamStats: Stats for a single stream (FPS, drops, latency)
    - RuntimeStats: Aggregates all stream stats with periodic logging
    - Detects frame drops via sequence number gaps

KEY CONFIGURATION:
    - log_interval_sec (default 10.0): How often to log stats
    - Frame drop detection: Checks sequence number gaps

HOW TO EDIT:
    - To change log interval: Modify log_interval_sec in RuntimeStats.__init__()
    - To add new stream: Call runtime_stats.record_frame("new_stream", ...)
    - To change FPS window: Modify start_time tracking logic

DEPENDENCIES (imports from):
    - logging: Python standard logging
    - time: For timestamps
    - dataclasses: For StreamStats/RuntimeStats

USED BY:
    - demo_live_view.py: Tracks FPS and logs stats
    - node.py: Reports stats to ROS2 diagnostics
    - status.py: Uses stats to compute health status
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import Callable

logger = logging.getLogger(__name__)


@dataclass
class StreamStats:
    """Statistics for a single data stream.
    
    Tracks frame counts, FPS, and latency for streams like
    detection, segmentation, depth, preview.
    """
    name: str
    frame_count: int = 0
    start_time: float = field(default_factory=time.time)
    total_latency_ms: float = 0.0
    last_timestamp_ns: int = 0
    last_sequence: int = -1
    drops: int = 0
    
    @property
    def fps(self) -> float:
        """Calculate frames per second."""
        elapsed = time.time() - self.start_time
        if elapsed <= 0:
            return 0.0
        return self.frame_count / elapsed
    
    @property
    def avg_latency_ms(self) -> float:
        """Calculate average latency in milliseconds."""
        if self.frame_count <= 0:
            return 0.0
        return self.total_latency_ms / self.frame_count
    
    @property
    def drop_rate(self) -> float:
        """Calculate drop rate as percentage."""
        total = self.frame_count + self.drops
        if total <= 0:
            return 0.0
        return (self.drops / total) * 100.0
    
    def record(
        self,
        timestamp_ns: int,
        host_time: float | None = None,
        sequence_num: int | None = None,
    ) -> int:
        """Record a frame reception.
        
        Args:
            timestamp_ns: Device timestamp in nanoseconds
            host_time: Host receive time (time.time() value)
            sequence_num: Optional sequence number for drop detection
        
        Returns:
            Number of drops detected (0 if none)
        """
        if self.frame_count == 0:
            self.start_time = time.time()
        
        self.frame_count += 1
        self.last_timestamp_ns = timestamp_ns
        
        # Calculate latency if host time provided
        if host_time is not None and timestamp_ns > 0:
            # Convert device timestamp to seconds
            device_sec = timestamp_ns / 1_000_000_000
            # Estimate latency (rough - device and host clocks not synced)
            # This gives relative timing trends, not absolute latency
            self.total_latency_ms += (host_time - device_sec) * 1000
        
        # Detect drops via sequence number gaps
        dropped = 0
        if sequence_num is not None and self.last_sequence >= 0:
            expected = self.last_sequence + 1
            if sequence_num > expected:
                dropped = sequence_num - expected
                self.drops += dropped
        
        if sequence_num is not None:
            self.last_sequence = sequence_num
        
        return dropped
    
    def reset(self) -> None:
        """Reset all statistics."""
        self.frame_count = 0
        self.start_time = time.time()
        self.total_latency_ms = 0.0
        self.last_timestamp_ns = 0
        self.last_sequence = -1
        self.drops = 0


class RuntimeStats:
    """Aggregate runtime statistics manager.
    
    Collects FPS and latency data for all streams, with periodic
    logging and summary generation.
    
    Attributes:
        log_interval_sec: Seconds between automatic log outputs
        log_callback: Optional callback for log messages
    """
    
    STREAM_NAMES = ("preview", "detection", "segmentation", "depth")
    
    def __init__(
        self,
        log_interval_sec: float = 10.0,
        log_callback: Callable[[str], None] | None = None,
    ) -> None:
        """Initialize runtime stats.
        
        Args:
            log_interval_sec: Seconds between automatic log outputs.
                             Set to 0 to disable automatic logging.
            log_callback: Optional callback for log messages.
                         If None, uses module logger.
        """
        self._streams: dict[str, StreamStats] = {}
        for name in self.STREAM_NAMES:
            self._streams[name] = StreamStats(name=name)
        
        self.log_interval_sec = log_interval_sec
        self._log_callback = log_callback
        self._last_log_time = time.time()
        self._start_time = time.time()
    
    def record_frame(
        self,
        stream: str,
        timestamp_ns: int,
        host_time: float | None = None,
        sequence_num: int | None = None,
    ) -> int:
        """Record a frame for a stream.
        
        Args:
            stream: Stream name (preview, detection, segmentation, depth)
            timestamp_ns: Device timestamp in nanoseconds
            host_time: Host receive time (time.time())
            sequence_num: Optional sequence number for drop detection
        
        Returns:
            Number of drops detected
        """
        if stream not in self._streams:
            self._streams[stream] = StreamStats(name=stream)
        
        return self._streams[stream].record(timestamp_ns, host_time, sequence_num)
    
    def get_stream(self, stream: str) -> StreamStats | None:
        """Get stats for a specific stream."""
        return self._streams.get(stream)
    
    def maybe_log(self) -> bool:
        """Log stats if interval has elapsed.
        
        Returns:
            True if logged, False otherwise
        """
        if self.log_interval_sec <= 0:
            return False
        
        now = time.time()
        if now - self._last_log_time >= self.log_interval_sec:
            self.log_summary()
            self._last_log_time = now
            return True
        return False
    
    def log_summary(self) -> None:
        """Log current statistics summary."""
        lines = ["Runtime Stats:"]
        for name in self.STREAM_NAMES:
            stats = self._streams.get(name)
            if stats and stats.frame_count > 0:
                line = f"  {name}: {stats.fps:.1f} FPS"
                if stats.drops > 0:
                    line += f" (drops: {stats.drop_rate:.1f}%)"
                lines.append(line)
        
        msg = "\n".join(lines)
        if self._log_callback:
            self._log_callback(msg)
        else:
            logger.info(msg)
    
    def get_summary(self) -> str:
        """Get formatted summary string."""
        lines = ["Runtime Statistics:"]
        lines.append(f"  Uptime: {time.time() - self._start_time:.1f}s")
        
        for name in self.STREAM_NAMES:
            stats = self._streams.get(name)
            if stats and stats.frame_count > 0:
                lines.append(
                    f"  {name}: {stats.fps:.1f} FPS, "
                    f"{stats.frame_count} frames, "
                    f"{stats.drop_rate:.1f}% drops"
                )
        
        return "\n".join(lines)
    
    def get_dict(self) -> dict:
        """Get statistics as dictionary for JSON serialization."""
        result = {
            "uptime_sec": time.time() - self._start_time,
            "streams": {},
        }
        
        for name in self.STREAM_NAMES:
            stats = self._streams.get(name)
            if stats:
                result["streams"][name] = {
                    "fps": round(stats.fps, 1),
                    "frame_count": stats.frame_count,
                    "drop_rate": round(stats.drop_rate, 1),
                    "avg_latency_ms": round(stats.avg_latency_ms, 1),
                }
        
        return result
    
    def reset(self) -> None:
        """Reset all statistics."""
        for stats in self._streams.values():
            stats.reset()
        self._start_time = time.time()
        self._last_log_time = time.time()
    
    @property
    def uptime_sec(self) -> float:
        """Get uptime in seconds."""
        return time.time() - self._start_time
    
    @property
    def detection_fps(self) -> float:
        """Get detection stream FPS."""
        stats = self._streams.get("detection")
        return stats.fps if stats else 0.0
    
    @property
    def segmentation_fps(self) -> float:
        """Get segmentation stream FPS."""
        stats = self._streams.get("segmentation")
        return stats.fps if stats else 0.0
    
    @property
    def depth_fps(self) -> float:
        """Get depth stream FPS."""
        stats = self._streams.get("depth")
        return stats.fps if stats else 0.0
    
    @property
    def preview_fps(self) -> float:
        """Get preview stream FPS."""
        stats = self._streams.get("preview")
        return stats.fps if stats else 0.0
