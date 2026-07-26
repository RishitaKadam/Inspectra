"""
train_pcb_detector.py: fine-tunes YOLOv8n on the PCB Dataset Defect
(693 images, 6 defect classes: missing_hole, mouse_bite, open_circuit,
short, spur, spurious_copper — sourced from Peking University's
PKU-Market-PCB dataset via Roboflow).

Run manually (not a ROS node) — this is an offline training step:
    python3 train_pcb_detector.py
"""

from ultralytics import YOLO

DATA_YAML = "/home/rk/inspectra/PCB-Dataset-Defect-1/data.yaml"
EPOCHS = 50
IMG_SIZE = 640

def main():
    model = YOLO("yolov8n.pt")  # start from COCO-pretrained weights (transfer learning)

    results = model.train(
        data=DATA_YAML,
        epochs=EPOCHS,
        imgsz=IMG_SIZE,
        project="/home/rk/inspectra/training_runs",
        name="pcb_defect_detector",
        patience=15,  # early stop if val loss plateaus
    )

    print("Training complete.")
    print("Best weights saved to: /home/rk/inspectra/training_runs/pcb_defect_detector/weights/best.pt")


if __name__ == "__main__":
    main()
