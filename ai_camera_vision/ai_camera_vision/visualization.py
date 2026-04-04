"""Visualization utilities for RGB preview and overlays.

PURPOSE:
    Provides OpenCV-based visualization for the live demo, including
    preview window, detection boxes, segmentation overlay, and stats panel.

WHAT THIS FILE DOES:
    - PreviewWindow: Manages OpenCV window with keyboard exit handling
    - draw_detections(): Draws bounding boxes and labels on frame
    - overlay_segmentation(): Overlays colored segmentation mask
    - draw_zone_legend(): Draws green/yellow/red zone legend
    - draw_stats_panel(): Draws FPS and statistics panel

KEY CONFIGURATION:
    - SAFETY_COLORS: Box colors per safety category (BGR format)
    - ZONE_COLORS: Segmentation overlay colors (green/yellow/red)
    - PreviewWindow.EXIT_KEYS: Keys that close the window

HOW TO EDIT:
    - To change box colors: Edit SAFETY_COLORS dictionary
    - To change overlay transparency: Modify alpha parameter in overlay_segmentation()
    - To change exit keys: Edit PreviewWindow.EXIT_KEYS tuple

DEPENDENCIES (imports from):
    - cv2: OpenCV for drawing
    - numpy: Array operations
    - ai_camera_vision.detections: Detection2D type hint

USED BY:
    - demo_live_view.py: All functions used in visualization loop
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import cv2
import numpy as np

if TYPE_CHECKING:
    from ai_camera_vision.detections import Detection2D


# =============================================================================
# Color Definitions
# =============================================================================

# Safety category colors (BGR format for OpenCV)
SAFETY_COLORS = {
    "human": (0, 0, 255),       # Red - DANGER
    "animal": (0, 165, 255),    # Orange - CAUTION
    "vehicle": (255, 0, 0),     # Blue - OBSTACLE
    "static_obstacle": (0, 255, 255),  # Yellow
    "unknown": (128, 128, 128), # Gray
}

# Zone colors for segmentation overlay (BGR)
ZONE_COLORS = {
    0: (0, 0, 0),        # Unknown - transparent (black for mask)
    1: (0, 255, 0),      # Green - safe
    2: (0, 255, 255),    # Yellow - caution
    3: (0, 0, 255),      # Red - no-go
}

# PASCAL VOC class → Zone mapping
# Background (0) → Green, Person (15) → Red, Others → Yellow
PASCAL_TO_ZONE = {
    0: 0,   # Background → Unknown (won't show overlay)
    15: 3,  # Person → Red (no-go)
}


# =============================================================================
# Preview Window (Story 3.1)
# =============================================================================

class PreviewWindow:
    """Manages an OpenCV preview window with clean exit handling.
    
    Story 3.1: Render RGB preview with clean exit.
    
    Attributes:
        title: Window title
        is_open: Whether window is currently open
    """
    
    EXIT_KEYS = (ord('q'), 27)  # 'q' and ESC
    
    def __init__(
        self,
        title: str = "AI Camera Vision",
        *,
        scale: float = 1.0,
    ) -> None:
        """Initialize preview window.
        
        Args:
            title: Window title
            scale: Display scale factor (1.0 = original size)
        """
        self.title = title
        self.scale = scale
        self._window_created = False
        self._is_open = True
    
    @property
    def is_open(self) -> bool:
        """Check if window should remain open."""
        return self._is_open
    
    def show(self, frame: np.ndarray) -> bool:
        """Display frame and check for exit key.
        
        Args:
            frame: BGR image to display
        
        Returns:
            True if should continue, False if user requested exit
        """
        if not self._is_open:
            return False
        
        try:
            if not self._window_created:
                cv2.namedWindow(self.title, cv2.WINDOW_AUTOSIZE)
                self._window_created = True
            
            # Apply scale if needed
            if self.scale != 1.0:
                h, w = frame.shape[:2]
                new_size = (int(w * self.scale), int(h * self.scale))
                frame = cv2.resize(frame, new_size)
            
            cv2.imshow(self.title, frame)
            
            # Check for exit key
            key = cv2.waitKey(1) & 0xFF
            if key in self.EXIT_KEYS:
                self._is_open = False
                return False
            
            return True
            
        except cv2.error:
            # No display available
            self._is_open = False
            return False
    
    def close(self) -> None:
        """Close the window and release resources."""
        self._is_open = False
        if self._window_created:
            try:
                cv2.destroyWindow(self.title)
            except cv2.error:
                pass
            self._window_created = False
    
    def __enter__(self) -> "PreviewWindow":
        return self
    
    def __exit__(self, *args) -> None:
        self.close()


# =============================================================================
# Detection Overlay (Story 3.2)
# =============================================================================

def draw_detections(
    frame: np.ndarray,
    detections: list[Detection2D],
    *,
    confidence_threshold: float = 0.5,
    show_distance: bool = True,
    depth_frame: np.ndarray | None = None,
) -> np.ndarray:
    """Draw detection bounding boxes and labels on frame.
    
    Story 3.2: Overlay YOLO detections (boxes + labels).
    
    Args:
        frame: BGR image (modified in place)
        detections: List of Detection2D objects
        confidence_threshold: Skip detections below this
        show_distance: Show distance label if depth available
        depth_frame: Optional depth frame for distance labels
    
    Returns:
        Frame with detection overlays drawn
    """
    h, w = frame.shape[:2]
    
    for det in detections:
        if det.confidence < confidence_threshold:
            continue
        
        # Get pixel coordinates
        x1, y1, x2, y2 = det.to_pixels(w, h)
        
        # Get color based on safety category
        color = SAFETY_COLORS.get(det.safety_category, SAFETY_COLORS["unknown"])
        
        # Draw bounding box
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        
        # Build label text
        label = f"{det.class_label}: {det.confidence:.0%}"
        
        # Add distance if depth available
        if show_distance and depth_frame is not None:
            distance_m = _get_detection_distance(det, depth_frame)
            if distance_m is not None and distance_m > 0:
                label += f" [{distance_m:.2f}m]"
        
        # Draw label
        _draw_label(frame, label, (x1, y1), color)
    
    return frame


def _get_detection_distance(
    det: Detection2D,
    depth_frame: np.ndarray,
) -> float | None:
    """Get distance to detection center from depth frame."""
    try:
        dh, dw = depth_frame.shape[:2]
        cx = int(det.center_x * dw)
        cy = int(det.center_y * dh)
        
        if 0 <= cx < dw and 0 <= cy < dh:
            depth_mm = int(depth_frame[cy, cx])
            if depth_mm > 0:
                return depth_mm / 1000.0
    except Exception:
        pass
    return None


def _draw_label(
    frame: np.ndarray,
    text: str,
    position: tuple[int, int],
    color: tuple[int, int, int],
) -> None:
    """Draw a label with background rectangle."""
    x, y = position
    h, w = frame.shape[:2]
    
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.5
    thickness = 1
    
    (text_w, text_h), baseline = cv2.getTextSize(text, font, font_scale, thickness)
    
    # Position label above box, or inside if near top edge
    label_y = y - 5 if y > 25 else y + text_h + 5
    
    # Draw background rectangle
    cv2.rectangle(
        frame,
        (x, label_y - text_h - 5),
        (x + text_w + 4, label_y + 2),
        color,
        -1,
    )
    
    # Draw text
    cv2.putText(
        frame,
        text,
        (x + 2, label_y - 2),
        font,
        font_scale,
        (255, 255, 255),
        thickness,
    )


# =============================================================================
# Segmentation Overlay (Story 3.3)
# =============================================================================

def overlay_segmentation(
    frame: np.ndarray,
    mask: np.ndarray,
    *,
    alpha: float = 0.4,
    use_zone_colors: bool = False,
) -> np.ndarray:
    """Overlay segmentation mask on RGB frame.
    
    Story 3.3: Overlay segmentation mask.
    
    Args:
        frame: BGR image (H, W, 3)
        mask: Class ID mask (H_mask, W_mask) - will be resized
        alpha: Transparency (0=invisible, 1=opaque)
        use_zone_colors: If True, use green/yellow/red zone colors.
                        If False, use per-class colors.
    
    Returns:
        Frame with segmentation overlay
    """
    if mask is None or mask.size == 0:
        return frame
    
    h, w = frame.shape[:2]
    
    try:
        # Resize mask to frame size (use INTER_NEAREST to preserve class IDs)
        mask_resized = cv2.resize(
            mask.astype(np.uint8),
            (w, h),
            interpolation=cv2.INTER_NEAREST,
        )
    except Exception:
        return frame
    
    # Create color overlay
    overlay = np.zeros_like(frame)
    
    if use_zone_colors:
        # Map PASCAL VOC classes to zone colors
        # Background (0) → no overlay
        # Person (15) → Red (danger)
        # Other → Yellow (caution)
        overlay[mask_resized == 15] = (0, 0, 255)  # Red for person
        
        # All other non-background classes get yellow
        other_mask = (mask_resized > 0) & (mask_resized != 15)
        overlay[other_mask] = (0, 255, 255)  # Yellow
    else:
        # Per-class coloring
        overlay[mask_resized == 15] = (0, 0, 255)  # Red for person
        
        # Generate colors for other classes
        for class_id in range(1, 21):
            if class_id == 15:
                continue
            class_mask = mask_resized == class_id
            if np.any(class_mask):
                # Generate deterministic color from class ID
                color = (
                    (class_id * 37) % 255,
                    (class_id * 73) % 255,
                    (class_id * 113) % 255,
                )
                overlay[class_mask] = color
    
    # Create alpha mask (only blend where class > 0)
    has_overlay = mask_resized > 0
    if not np.any(has_overlay):
        return frame
    
    # Blend overlay with original frame
    alpha_mask = has_overlay.astype(np.float32) * alpha
    alpha_mask_3ch = np.stack([alpha_mask] * 3, axis=-1)
    
    result = (
        frame.astype(np.float32) * (1 - alpha_mask_3ch) +
        overlay.astype(np.float32) * alpha_mask_3ch
    )
    
    return result.astype(np.uint8)


def draw_zone_legend(
    frame: np.ndarray,
    position: str = "top-right",
) -> np.ndarray:
    """Draw zone color legend on frame.
    
    Args:
        frame: BGR image
        position: "top-left", "top-right", "bottom-left", "bottom-right"
    
    Returns:
        Frame with legend overlay
    """
    h, w = frame.shape[:2]
    
    legend_items = [
        ("Green: Safe", (0, 255, 0)),
        ("Yellow: Caution", (0, 255, 255)),
        ("Red: No-go", (0, 0, 255)),
    ]
    
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.5
    thickness = 1
    line_height = 20
    padding = 10
    
    # Calculate legend size
    max_width = 0
    for text, _ in legend_items:
        (tw, _), _ = cv2.getTextSize(text, font, font_scale, thickness)
        max_width = max(max_width, tw)
    
    legend_w = max_width + 30 + padding * 2  # 30 for color square
    legend_h = len(legend_items) * line_height + padding * 2
    
    # Position
    if "right" in position:
        x = w - legend_w - 5
    else:
        x = 5
    if "bottom" in position:
        y = h - legend_h - 5
    else:
        y = 5
    
    # Draw background
    cv2.rectangle(frame, (x, y), (x + legend_w, y + legend_h), (0, 0, 0), -1)
    cv2.rectangle(frame, (x, y), (x + legend_w, y + legend_h), (255, 255, 255), 1)
    
    # Draw items
    text_y = y + padding + 15
    for text, color in legend_items:
        # Color square
        cv2.rectangle(
            frame,
            (x + padding, text_y - 12),
            (x + padding + 15, text_y),
            color,
            -1,
        )
        # Text
        cv2.putText(
            frame,
            text,
            (x + padding + 20, text_y),
            font,
            font_scale,
            (255, 255, 255),
            thickness,
        )
        text_y += line_height
    
    return frame


# =============================================================================
# Stats Overlay (Story 3.4)
# =============================================================================

def draw_stats_panel(
    frame: np.ndarray,
    stats: dict[str, str],
    position: str = "top-left",
) -> np.ndarray:
    """Draw statistics panel on frame.
    
    Args:
        frame: BGR image
        stats: Dictionary of label → value strings
        position: "top-left", "top-right", "bottom-left", "bottom-right"
    
    Returns:
        Frame with stats overlay
    """
    if not stats:
        return frame
    
    h, w = frame.shape[:2]
    
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.5
    thickness = 1
    line_height = 22
    padding = 10
    
    # Calculate panel size
    max_width = 0
    for label, value in stats.items():
        text = f"{label}: {value}"
        (tw, _), _ = cv2.getTextSize(text, font, font_scale, thickness)
        max_width = max(max_width, tw)
    
    panel_w = max_width + padding * 2
    panel_h = len(stats) * line_height + padding * 2
    
    # Position
    if "right" in position:
        x = w - panel_w - 5
    else:
        x = 5
    if "bottom" in position:
        y = h - panel_h - 5
    else:
        y = 5
    
    # Draw background
    cv2.rectangle(frame, (x, y), (x + panel_w, y + panel_h), (0, 0, 0), -1)
    cv2.rectangle(frame, (x, y), (x + panel_w, y + panel_h), (0, 255, 0), 1)
    
    # Draw stats
    text_y = y + padding + 15
    for label, value in stats.items():
        text = f"{label}: {value}"
        cv2.putText(frame, text, (x + padding, text_y), font, font_scale, (255, 255, 255), thickness)
        text_y += line_height
    
    return frame
