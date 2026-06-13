"""
train_signs.py
--------------
Fine-tune YOLOv8-nano on your own turn-sign images.

FOLDER STRUCTURE expected:
    dataset/
        images/
            train/   ← your training images (.jpg / .png)
            val/     ← validation images (10-20 % of total)
        labels/
            train/   ← YOLO-format .txt files (one per image)
            val/

YOLO label format (one line per object in the image):
    <class_id> <x_center> <y_center> <width> <height>
    (all values normalised 0-1 relative to image size)

    Example for a "turn_left" sign covering the centre of a 640x480 image:
        0 0.5 0.5 0.3 0.4

LABELLING TOOLS (free):
    - LabelImg  →  https://github.com/heartexlabs/labelImg
      (select YOLO format, draw boxes, auto-saves .txt files)
    - Roboflow  →  https://roboflow.com  (web-based, exports YOLO format)

Requirements:
    pip install ultralytics
"""

from ultralytics import YOLO
import yaml, os

# ─── Edit these ───────────────────────────────────────────────────────────────

DATASET_ROOT = "dataset"        # path to your dataset folder

# Class names — must match the order of your label IDs (0, 1, 2, ...)
CLASS_NAMES = [
    "turn_left",
    "turn_right",
    "stop",
]

EPOCHS      = 60          # more epochs = better accuracy (diminishing returns past ~100)
IMAGE_SIZE  = 416         # 416 is a good balance of speed vs accuracy for small signs
BATCH_SIZE  = 16          # reduce to 8 if you run out of GPU memory

OUTPUT_DIR  = "signs_model"   # where weights/results are saved

# ──────────────────────────────────────────────────────────────────────────────

def build_yaml(root: str, names: list[str]) -> str:
    """Write a dataset.yaml file that Ultralytics expects."""
    cfg = {
        "path":  os.path.abspath(root),
        "train": "images/train",
        "val":   "images/val",
        "names": {i: n for i, n in enumerate(names)},
    }
    yaml_path = os.path.join(root, "dataset.yaml")
    with open(yaml_path, "w") as f:
        yaml.dump(cfg, f, default_flow_style=False)
    print(f"[TRAIN] Wrote {yaml_path}")
    return yaml_path


def train():
    yaml_path = build_yaml(DATASET_ROOT, CLASS_NAMES)

    # yolov8n = nano (fastest, smallest — good for a Raspberry Pi / Jetson Nano)
    # swap for yolov8s if you have more GPU and want higher accuracy
    model = YOLO("yolo26n.pt")   # downloads pretrained weights automatically

    results = model.train(
        data      = yaml_path,
        epochs    = EPOCHS,
        imgsz     = IMAGE_SIZE,
        batch     = BATCH_SIZE,
        project   = OUTPUT_DIR,
        name      = "weights",
        exist_ok  = True,

        # Augmentations — help the model generalise to different lighting / angles
        hsv_h     = 0.015,   # colour hue shift
        hsv_s     = 0.5,     # saturation
        hsv_v     = 0.4,     # brightness
        fliplr    = 0.0,     # don't flip left/right — a left-turn sign IS different from right
        degrees   = 10,      # small rotation
        translate = 0.1,
        scale     = 0.3,
    )

    print("\n[TRAIN] Done.")
    print(f"[TRAIN] Best weights → {OUTPUT_DIR}/weights/weights/best.pt")
    print("[TRAIN] Update YOLO_MODEL_PATH in turtlebot_autonomous.py to that path.")


if __name__ == "__main__":
    train()