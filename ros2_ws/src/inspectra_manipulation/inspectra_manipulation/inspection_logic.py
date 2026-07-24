"""
inspection_logic: PASS/FAIL decision rule for Inspectra's sorting cycle.

PLACEHOLDER BUSINESS LOGIC: this checks the YOLOv8-detected class name
against a small allow-list, as a stand-in for real quality inspection.
Genuine defect detection would need a custom-trained model on actual
Inspectra parts (surface defects, dimensional tolerances, etc.) — a
natural "future work" item, not something this placeholder claims to do.
"""

ACCEPTABLE_CLASSES = {"bottle", "cup", "book"}
MIN_CONFIDENCE = 0.5


def decide_pass_fail(class_name: str, confidence: float) -> str:
    """Returns 'PASS' or 'FAIL' for a single detected object.

    Args:
        class_name: YOLOv8 class label (e.g. 'bottle', 'person')
        confidence: detection confidence score, 0.0-1.0
    """
    if confidence < MIN_CONFIDENCE:
        return "FAIL"
    if class_name in ACCEPTABLE_CLASSES:
        return "PASS"
    return "FAIL"
