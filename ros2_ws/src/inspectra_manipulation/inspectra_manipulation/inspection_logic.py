"""
inspection_logic: GOOD/BAD PCB decision rule, based on a custom YOLOv8
model trained on the PKU-Market-PCB defect dataset (missing_hole,
mouse_bite, open_circuit, short, spur, spurious_copper).

Design: the dataset only labels DEFECTS (no "good" class exists), so a
PCB is GOOD if no defect class is detected above confidence threshold
in the current frame, and BAD if any defect is found.
"""

DEFECT_CLASSES = {
    "missing_hole", "mouse_bite", "open_circuit",
    "short", "spur", "spurious_copper",
}
MIN_CONFIDENCE = 0.5


def get_sorting_target(classification: str) -> tuple[str, str]:
    """Return the display color and bin name for a PCB classification."""
    if classification == "BAD":
        return "red", "FAIL_BIN"
    return "green", "PASS_BIN"


def classify_pcb(detections: list) -> str:
    """Classify a PCB as GOOD or BAD from a list of (class_name, confidence) tuples.

    Args:
        detections: all detections for the current frame, e.g.
            [("mouse_bite", 0.81), ("short", 0.63)]
            An empty list means no defects were detected -> GOOD.
    """
    for class_name, confidence in detections:
        if class_name in DEFECT_CLASSES and confidence >= MIN_CONFIDENCE:
            return "BAD"
    return "GOOD"
