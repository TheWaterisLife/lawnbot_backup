"""Zone summary computation for segmentation masks.

PURPOSE:
    Computes green/yellow/red zone ratios from segmentation masks.
    These ratios tell the mower brain how safe the scene is.

WHAT THIS FILE DOES:
    - ZoneSummary: Dataclass with green/yellow/red ratios
    - compute_zone_ratios(): Computes ratios from mask
    - decode_segmentation_mask(): Converts raw NN output to class IDs

ZONE MEANING:
    - GREEN: Safe to cut and drive (vegetation, terrain)
    - YELLOW: Safe to drive only (road, sidewalk)
    - RED: No-go zone (humans, vehicles, obstacles)

KEY CONFIGURATION:
    - ADAS_GREEN_CLASSES: Classes considered "safe" (8=vegetation, 9=terrain)
    - ADAS_RED_CLASSES: Classes considered "danger" (11=person, vehicles, etc.)
    - ZoneSummary.recommendation thresholds (line ~120)

HOW TO EDIT:
    - To change zone classification: Edit ADAS_GREEN/RED/YELLOW_CLASSES sets
    - To change action thresholds: Edit ZoneSummary.recommendation property
    - To support new model: Add new *_CLASSES sets and update compute_zone_ratios()

DEPENDENCIES (imports from):
    - numpy: Array operations

USED BY:
    - demo_live_view.py: compute_zone_ratios() for decision indicator
    - node.py: Publishes zone ratios to ROS2 topics
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    pass


# =============================================================================
# Zone Definitions (per schema-v1.md)
# =============================================================================

# PASCAL VOC classes from DeepLabV3+ (legacy model)
PASCAL_VOC_BACKGROUND = 0
PASCAL_VOC_PERSON = 15

# PASCAL VOC zone mappings
PASCAL_RED_ZONE_CLASSES = frozenset({15})  # Person
PASCAL_YELLOW_ZONE_CLASSES = frozenset(range(1, 21)) - PASCAL_RED_ZONE_CLASSES

# =============================================================================
# ADAS Segmentation Classes (semantic-segmentation-adas-0001)
# =============================================================================
# 20 classes for outdoor/driving scenes:
#   0: road, 1: sidewalk, 2: building, 3: wall, 4: fence, 5: pole,
#   6: traffic light, 7: traffic sign, 8: vegetation, 9: terrain,
#   10: sky, 11: person, 12: rider, 13: car, 14: truck, 15: bus,
#   16: train, 17: motorcycle, 18: bicycle, 19: ego-vehicle

# GREEN zone - safe to cut and drive
ADAS_GREEN_CLASSES = frozenset({8, 9})  # vegetation, terrain

# RED zone - no-go zone (humans, vehicles, obstacles)
ADAS_RED_CLASSES = frozenset({
    2, 3, 4, 5, 6, 7,  # building, wall, fence, pole, traffic light/sign
    11, 12,             # person, rider
    13, 14, 15, 16, 17, 18,  # car, truck, bus, train, motorcycle, bicycle
})

# YELLOW zone - safe to drive, not to cut
ADAS_YELLOW_CLASSES = frozenset({0, 1})  # road, sidewalk

# IGNORED classes - excluded from ratio calculations
ADAS_IGNORED_CLASSES = frozenset({10, 19})  # sky, ego-vehicle

# Model type enum
MODEL_PASCAL_VOC = "PASCAL_VOC"
MODEL_ADAS = "ADAS"


# =============================================================================
# Data Classes
# =============================================================================

@dataclass(frozen=True)
class ZoneSummary:
    """Zone summary from segmentation mask.
    
    Attributes:
        green_ratio: Fraction of pixels that are safe (background)
        yellow_ratio: Fraction of pixels that are caution (obstacles)
        red_ratio: Fraction of pixels that are no-go (person)
        total_pixels: Total number of pixels analyzed
        classified_pixels: Number of non-background pixels
    """
    green_ratio: float
    yellow_ratio: float
    red_ratio: float
    total_pixels: int = 0
    classified_pixels: int = 0
    
    def to_array(self) -> list[float]:
        """Convert to Float32MultiArray data format [green, yellow, red]."""
        return [self.green_ratio, self.yellow_ratio, self.red_ratio]
    
    def to_dict(self) -> dict:
        """Convert to dictionary for JSON serialization."""
        return {
            "green_ratio": self.green_ratio,
            "yellow_ratio": self.yellow_ratio,
            "red_ratio": self.red_ratio,
            "total_pixels": self.total_pixels,
            "classified_pixels": self.classified_pixels,
        }
    
    @property
    def is_safe(self) -> bool:
        """Check if scene is predominantly safe (green)."""
        return self.red_ratio < 0.01 and self.yellow_ratio < 0.1
    
    @property
    def has_person(self) -> bool:
        """Check if a person is detected (any red pixels)."""
        return self.red_ratio > 0.001
    
    @property
    def recommendation(self) -> str:
        """Get mower action recommendation based on zones."""
        if self.red_ratio > 0.01:
            return "STOP"
        elif self.yellow_ratio > 0.1:
            return "SLOW"
        else:
            return "GO"


# =============================================================================
# Computation Functions
# =============================================================================

def compute_zone_ratios(
    mask: np.ndarray,
    roi: tuple[int, int, int, int] | None = None,
    model_type: str = MODEL_PASCAL_VOC,
) -> ZoneSummary:
    """Compute zone ratios from segmentation mask.
    
    Supports two model types:
    - MODEL_PASCAL_VOC: DeepLabV3+ (background=GREEN, person=RED, others=YELLOW)
      This is the default for deeplab_v3_mnv2_256x256.blob
    - MODEL_ADAS: semantic-segmentation-adas-0001 (vegetation/terrain=GREEN,
                  obstacles/humans=RED, road/sidewalk=YELLOW)
    
    Args:
        mask: 2D numpy array of class IDs
        roi: Optional (x, y, width, height) region of interest.
             If None, uses full mask.
        model_type: MODEL_PASCAL_VOC (default) or MODEL_ADAS
    
    Returns:
        ZoneSummary with green/yellow/red ratios
    """
    if mask is None or mask.size == 0:
        return ZoneSummary(0.0, 0.0, 0.0, 0, 0)
    
    # Apply ROI if specified
    if roi is not None:
        x, y, w, h = roi
        mask = mask[y:y+h, x:x+w]
    
    if mask.size == 0:
        return ZoneSummary(0.0, 0.0, 0.0, 0, 0)
    
    if model_type == MODEL_ADAS:
        # ADAS model: count pixels per zone using class sets
        green_pixels = int(np.sum(np.isin(mask, list(ADAS_GREEN_CLASSES))))
        red_pixels = int(np.sum(np.isin(mask, list(ADAS_RED_CLASSES))))
        yellow_pixels = int(np.sum(np.isin(mask, list(ADAS_YELLOW_CLASSES))))
        ignored_pixels = int(np.sum(np.isin(mask, list(ADAS_IGNORED_CLASSES))))
        
        # Total excludes ignored pixels (sky, ego-vehicle)
        total_pixels = int(mask.size) - ignored_pixels
        classified_pixels = red_pixels + yellow_pixels
    else:
        # PASCAL VOC model: background=green, person=red, rest=yellow
        total_pixels = int(mask.size)
        green_pixels = int(np.sum(mask == PASCAL_VOC_BACKGROUND))
        red_pixels = int(np.sum(mask == PASCAL_VOC_PERSON))
        yellow_pixels = total_pixels - green_pixels - red_pixels
        classified_pixels = red_pixels + yellow_pixels
    
    # Compute ratios
    if total_pixels > 0:
        green_ratio = green_pixels / total_pixels
        yellow_ratio = yellow_pixels / total_pixels
        red_ratio = red_pixels / total_pixels
    else:
        green_ratio = yellow_ratio = red_ratio = 0.0
    
    return ZoneSummary(
        green_ratio=green_ratio,
        yellow_ratio=yellow_ratio,
        red_ratio=red_ratio,
        total_pixels=total_pixels,
        classified_pixels=classified_pixels,
    )


def compute_zone_ratios_roi(
    mask: np.ndarray,
    roi_policy: str = "full",
    frame_height: int | None = None,
    model_type: str = MODEL_PASCAL_VOC,
) -> ZoneSummary:
    """Compute zone ratios with configurable ROI policy.
    
    Args:
        mask: 2D numpy array of class IDs
        roi_policy: One of "full", "bottom_half", "center_strip"
        frame_height: Original frame height (for ROI calculation)
        model_type: MODEL_PASCAL_VOC (default) or MODEL_ADAS
    
    Returns:
        ZoneSummary for the specified ROI
    """
    if mask is None or mask.size == 0:
        return ZoneSummary(0.0, 0.0, 0.0, 0, 0)
    
    h, w = mask.shape[:2]
    
    if roi_policy == "full":
        roi = None
    elif roi_policy == "bottom_half":
        # Bottom half of frame (closer to mower)
        roi = (0, h // 2, w, h // 2)
    elif roi_policy == "center_strip":
        # Center 50% width, bottom 60% height
        roi = (w // 4, int(h * 0.4), w // 2, int(h * 0.6))
    else:
        roi = None
    
    return compute_zone_ratios(mask, roi, model_type=model_type)


def decode_segmentation_mask(
    seg_data: np.ndarray,
    expected_shape: tuple[int, int] = (256, 256),
) -> np.ndarray | None:
    """Decode raw segmentation output to class ID mask.
    
    DeepLabV3+ can output:
    - Direct class IDs: shape (H, W)
    - Softmax probabilities: shape (num_classes, H, W)
    
    Args:
        seg_data: Raw segmentation output
        expected_shape: Expected mask dimensions
    
    Returns:
        2D mask of class IDs, or None if decoding fails
    """
    if seg_data is None or len(seg_data) == 0:
        return None
    
    arr = np.array(seg_data)
    h, w = expected_shape
    expected_pixels = h * w
    
    # Case 1: Direct class IDs (H*W values)
    if len(arr) == expected_pixels:
        return arr.reshape(h, w).astype(np.uint8)
    
    # Case 2: Softmax output (num_classes * H * W values)
    # PASCAL VOC has 21 classes (0-20)
    if len(arr) == 21 * expected_pixels:
        arr = arr.reshape(21, h, w)
        return np.argmax(arr, axis=0).astype(np.uint8)
    
    # Case 3: Try to infer number of channels
    if len(arr) % expected_pixels == 0:
        num_channels = len(arr) // expected_pixels
        arr = arr.reshape(num_channels, h, w)
        return np.argmax(arr, axis=0).astype(np.uint8)
    
    return None
