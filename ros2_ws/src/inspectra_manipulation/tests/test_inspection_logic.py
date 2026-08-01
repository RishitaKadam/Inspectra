import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from inspectra_manipulation import inspection_logic


def test_good_pcb_targets_green_bin():
    assert inspection_logic.get_sorting_target("GOOD") == ("green", "PASS_BIN")


def test_bad_pcb_targets_red_bin():
    assert inspection_logic.get_sorting_target("BAD") == ("red", "FAIL_BIN")


def test_classify_pcb_marks_defect_when_confidence_is_high():
    detections = [("mouse_bite", 0.81)]
    assert inspection_logic.classify_pcb(detections) == "BAD"
