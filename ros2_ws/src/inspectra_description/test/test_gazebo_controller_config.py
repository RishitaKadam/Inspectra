import pathlib


def test_hand_controller_uses_only_primary_finger_joint():
    config_path = pathlib.Path(__file__).resolve().parents[1] / "config" / "inspectra_controllers.yaml"
    text = config_path.read_text()

    assert "panda_finger_joint1" in text
    assert "panda_finger_joint2" not in text.split("panda_hand_controller:", 1)[1].split("panda_arm_controller:", 1)[0]
