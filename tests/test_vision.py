import numpy as np
import pytest

from vision.base import Detection, ProcessorResult, VisionProcessor


def test_processor_result_structure():
    det = Detection(label="person", confidence=0.95, bbox=[10.0, 20.0, 100.0, 200.0])
    assert det.label == "person"
    assert det.confidence == 0.95
    assert det.bbox == [10.0, 20.0, 100.0, 200.0]

    result = ProcessorResult(detections=[det], annotated_frame=b"fake-jpeg")
    assert len(result.detections) == 1
    assert result.detections[0] is det
    assert result.annotated_frame == b"fake-jpeg"


def test_base_processor_raises():
    processor = VisionProcessor()
    with pytest.raises(NotImplementedError):
        processor.process(np.zeros((480, 640, 3), dtype=np.uint8))
