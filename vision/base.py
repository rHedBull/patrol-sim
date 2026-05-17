from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class Detection:
    label: str
    confidence: float
    bbox: tuple[float, float, float, float]  # [x1, y1, x2, y2]


@dataclass
class ProcessorResult:
    detections: list[Detection] = field(default_factory=list)
    annotated_frame: bytes = b""


class VisionProcessor:
    """Base class for vision processors."""

    def process(self, frame: np.ndarray) -> ProcessorResult:
        raise NotImplementedError
