from __future__ import annotations

import io

import numpy as np
from PIL import Image
from ultralytics import YOLO

from vision.base import Detection, ProcessorResult, VisionProcessor


class YOLOProcessor(VisionProcessor):
    """Vision processor using YOLOv8 for object detection."""

    def __init__(self, model_name: str = "yolov8n.pt", confidence: float = 0.25):
        self.model = YOLO(model_name)
        self.confidence = confidence

    def process(self, frame: np.ndarray) -> ProcessorResult:
        results = self.model(frame, conf=self.confidence, verbose=False)
        result = results[0]

        detections: list[Detection] = []
        for box in result.boxes:
            detections.append(
                Detection(
                    label=result.names[int(box.cls[0])],
                    confidence=float(box.conf[0]),
                    bbox=box.xyxy[0].tolist(),
                )
            )

        annotated = result.plot()
        img = Image.fromarray(annotated)
        buf = io.BytesIO()
        img.save(buf, format="JPEG")
        annotated_bytes = buf.getvalue()

        return ProcessorResult(detections=detections, annotated_frame=annotated_bytes)
