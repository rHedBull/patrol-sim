from __future__ import annotations

import io

import numpy as np
import torch
from PIL import Image, ImageDraw, ImageFont
from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection

from vision.base import Detection, ProcessorResult, VisionProcessor


class GroundingDINOProcessor(VisionProcessor):
    """Vision processor using Grounding DINO for open-set object detection."""

    def __init__(
        self,
        model_id: str = "IDEA-Research/grounding-dino-tiny",
        text_prompt: str = "a pipe. a pump. a valve.",
        confidence: float = 0.3,
        text_threshold: float = 0.25,
    ):
        self.processor = AutoProcessor.from_pretrained(model_id)
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = AutoModelForZeroShotObjectDetection.from_pretrained(
            model_id
        ).to(self.device)
        self.text_prompt = text_prompt
        self.confidence = confidence
        self.text_threshold = text_threshold

    def set_text_prompt(self, text_prompt: str) -> None:
        """Update the detection prompt at runtime."""
        self.text_prompt = text_prompt

    def process(self, frame: np.ndarray) -> ProcessorResult:
        image = Image.fromarray(frame)

        inputs = self.processor(
            images=image, text=self.text_prompt, return_tensors="pt"
        ).to(self.device)

        with torch.no_grad():
            outputs = self.model(**inputs)

        results = self.processor.post_process_grounded_object_detection(
            outputs,
            inputs.input_ids,
            threshold=self.confidence,
            text_threshold=self.text_threshold,
            target_sizes=[image.size[::-1]],
        )[0]

        detections: list[Detection] = []
        draw = ImageDraw.Draw(image)

        labels_key = "text_labels" if "text_labels" in results else "labels"
        for box, score, label in zip(
            results["boxes"], results["scores"], results[labels_key]
        ):
            bbox = [round(v, 1) for v in box.tolist()]
            det = Detection(
                label=label,
                confidence=round(float(score), 3),
                bbox=bbox,
            )
            detections.append(det)

            # Draw annotation
            draw.rectangle(bbox, outline="red", width=2)
            text = f"{label} {det.confidence:.2f}"
            draw.text((bbox[0] + 2, bbox[1] + 2), text, fill="red")

        buf = io.BytesIO()
        image.save(buf, format="JPEG")

        return ProcessorResult(
            detections=detections, annotated_frame=buf.getvalue()
        )
