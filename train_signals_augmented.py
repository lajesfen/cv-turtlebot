"""Prepare the local signal images and fine-tune a YOLO detector.

The source images are expected in ``signals/`` with the numbering currently
used by this repository.  The script:

1. excludes QR images (they are decoded with OpenCV, not YOLO),
2. finds the circular sign in each image and creates an initial bounding box,
3. creates a reproducible train/validation dataset,
4. writes labelled previews for visual verification, and
5. trains a small YOLO detector with safe augmentations for directional signs.

Run preparation and verify ``dataset_signals/label_previews`` first:

    python train_signals_augmented.py --prepare-only

Then train:

    python train_signals_augmented.py --epochs 60
"""

from __future__ import annotations

import argparse
import math
import shutil
from pathlib import Path

import cv2
import numpy as np
import yaml


ROOT = Path(__file__).resolve().parent
SOURCE_DIR = ROOT / "signals"
DATASET_DIR = ROOT / "dataset_signals"
RUNS_DIR = ROOT / "signs_model"

# Keep this order aligned with turtlebot_autonomous.py.
CLASS_NAMES = ["turn_left", "turn_right", "stop"]

# Numbering after the repository images were normalised to 001.jpeg, etc.
SOURCE_GROUPS = {
    "turn_right": [1, *range(5, 26)],
    "turn_left": [2, *range(26, 47)],
    "stop": [4, *range(47, 62)],
}
QR_IMAGE_IDS = [3, *range(62, 72)]

# Validation samples are spread through each capture sequence.  Adjacent
# frames remain similar, so metrics from this small first dataset are only a
# development signal, not a final generalisation measurement.
VALIDATION_IDS = {
    10,
    15,
    20,
    25,
    30,
    35,
    40,
    45,
    50,
    55,
    60,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Prepare and train a YOLO detector for TurtleBot signs."
    )
    parser.add_argument("--prepare-only", action="store_true")
    parser.add_argument("--epochs", type=int, default=60)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument("--model", default="yolo26n.pt")
    parser.add_argument(
        "--device",
        default="auto",
        help="auto, cpu, mps, or a CUDA device such as 0",
    )
    parser.add_argument(
        "--rebuild",
        action="store_true",
        help="delete and recreate dataset_signals before preparing it",
    )
    return parser.parse_args()


def source_path(image_id: int) -> Path:
    return SOURCE_DIR / f"{image_id:03d}.jpeg"


def validate_sources() -> None:
    expected_ids = {
        image_id
        for image_ids in SOURCE_GROUPS.values()
        for image_id in image_ids
    } | set(QR_IMAGE_IDS)
    expected = {source_path(image_id) for image_id in expected_ids}
    actual = set(SOURCE_DIR.glob("*.jpeg"))

    missing = sorted(path.name for path in expected - actual)
    unexpected = sorted(path.name for path in actual - expected)
    if missing or unexpected:
        raise RuntimeError(
            "The numbering in signals/ does not match this script. "
            f"Missing={missing}, unexpected={unexpected}"
        )


def detect_sign_box(image: np.ndarray) -> tuple[int, int, int, int]:
    """Detect the outer dark circle and return an expanded XYXY sign box."""
    height, width = image.shape[:2]
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)

    # The physical signs are black symbols on white paper.  Otsu adapts to the
    # brightness changes already present in the laboratory captures.
    _, dark = cv2.threshold(
        gray, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU
    )
    dark = cv2.morphologyEx(
        dark, cv2.MORPH_CLOSE, np.ones((5, 5), dtype=np.uint8), iterations=1
    )

    contours, _ = cv2.findContours(dark, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    image_area = float(width * height)
    candidates: list[tuple[float, tuple[int, int, int, int]]] = []

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < image_area * 0.001 or area > image_area * 0.35:
            continue

        x, y, box_width, box_height = cv2.boundingRect(contour)
        if box_width < 45 or box_height < 45:
            continue

        # In the robot-height captures every sign is mounted in the lower
        # two-thirds of the frame.  This rejects circular chair components and
        # wheels above the actual target.
        center_y = y + box_height / 2.0
        if center_y < height * 0.35:
            continue

        aspect = box_width / float(box_height)
        if not 0.72 <= aspect <= 1.38:
            continue

        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 0:
            continue
        circularity = 4.0 * math.pi * area / (perimeter * perimeter)
        extent = area / float(box_width * box_height)
        if circularity < 0.42 or extent < 0.45:
            continue

        # Prefer a large, square, circular contour.  The target ring has high
        # circularity; chair wheels and table details are usually much smaller.
        size_score = math.sqrt(area / image_area)
        square_score = 1.0 - abs(1.0 - aspect)
        score = 4.0 * size_score + 2.0 * circularity + square_score
        candidates.append((score, (x, y, x + box_width, y + box_height)))

    if not candidates:
        raise RuntimeError("No circular sign candidate found")

    _, (x1, y1, x2, y2) = max(candidates, key=lambda item: item[0])
    box_size = max(x2 - x1, y2 - y1)
    # Some stop-sign contours follow the inner white disk rather than the outer
    # black ring.  A generous margin keeps the full physical sign in the box.
    padding = max(10, round(box_size * 0.22))
    return (
        max(0, x1 - padding),
        max(0, y1 - padding),
        min(width - 1, x2 + padding),
        min(height - 1, y2 + padding),
    )


def yolo_line(
    class_id: int,
    box: tuple[int, int, int, int],
    width: int,
    height: int,
) -> str:
    x1, y1, x2, y2 = box
    center_x = ((x1 + x2) / 2.0) / width
    center_y = ((y1 + y2) / 2.0) / height
    box_width = (x2 - x1) / width
    box_height = (y2 - y1) / height
    return (
        f"{class_id} {center_x:.6f} {center_y:.6f} "
        f"{box_width:.6f} {box_height:.6f}\n"
    )


def prepare_dataset(rebuild: bool) -> Path:
    validate_sources()
    if rebuild and DATASET_DIR.exists():
        shutil.rmtree(DATASET_DIR)

    for split in ("train", "val"):
        (DATASET_DIR / "images" / split).mkdir(parents=True, exist_ok=True)
        (DATASET_DIR / "labels" / split).mkdir(parents=True, exist_ok=True)
    preview_dir = DATASET_DIR / "label_previews"
    preview_dir.mkdir(parents=True, exist_ok=True)

    counts = {name: {"train": 0, "val": 0} for name in CLASS_NAMES}
    failures: list[str] = []

    for class_name, image_ids in SOURCE_GROUPS.items():
        class_id = CLASS_NAMES.index(class_name)
        for image_id in image_ids:
            path = source_path(image_id)
            image = cv2.imread(str(path))
            if image is None:
                failures.append(f"{path.name}: unreadable image")
                continue

            # These are reference close-ups rather than robot-height captures,
            # so the complete frame is the object by construction.
            if image_id in {1, 2, 4}:
                height, width = image.shape[:2]
                margin_x = round(width * 0.01)
                margin_y = round(height * 0.01)
                box = (
                    margin_x,
                    margin_y,
                    width - margin_x - 1,
                    height - margin_y - 1,
                )
            else:
                try:
                    box = detect_sign_box(image)
                except RuntimeError as error:
                    failures.append(f"{path.name}: {error}")
                    continue

            split = "val" if image_id in VALIDATION_IDS else "train"
            image_target = DATASET_DIR / "images" / split / path.name
            label_target = DATASET_DIR / "labels" / split / f"{path.stem}.txt"
            shutil.copy2(path, image_target)
            height, width = image.shape[:2]
            label_target.write_text(
                yolo_line(class_id, box, width, height), encoding="utf-8"
            )

            preview = image.copy()
            x1, y1, x2, y2 = box
            cv2.rectangle(preview, (x1, y1), (x2, y2), (0, 255, 0), 5)
            cv2.putText(
                preview,
                class_name,
                (x1, max(35, y1 - 12)),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.2,
                (0, 255, 0),
                3,
                cv2.LINE_AA,
            )
            cv2.imwrite(str(preview_dir / path.name), preview)
            counts[class_name][split] += 1

    if failures:
        raise RuntimeError("Automatic labelling failed:\n" + "\n".join(failures))

    yaml_path = DATASET_DIR / "dataset.yaml"
    config = {
        "path": str(DATASET_DIR.resolve()),
        "train": "images/train",
        "val": "images/val",
        "names": {index: name for index, name in enumerate(CLASS_NAMES)},
    }
    yaml_path.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")

    print("[DATASET] Initial automatic labels created.")
    for class_name in CLASS_NAMES:
        print(
            f"[DATASET] {class_name}: "
            f"train={counts[class_name]['train']} val={counts[class_name]['val']}"
        )
    print(f"[DATASET] Verify every box in: {preview_dir}")
    print(f"[DATASET] QR images excluded: {len(QR_IMAGE_IDS)}")
    return yaml_path


def choose_device(requested: str) -> str:
    if requested != "auto":
        return requested

    import torch

    if torch.cuda.is_available():
        return "0"
    if torch.backends.mps.is_available():
        return "mps"
    return "cpu"


def train(yaml_path: Path, args: argparse.Namespace) -> None:
    from ultralytics import YOLO

    device = choose_device(args.device)
    print(f"[TRAIN] device={device} model={args.model}")
    model = YOLO(args.model)
    results = model.train(
        data=str(yaml_path),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=device,
        workers=0,
        project=str(RUNS_DIR),
        name="signals_augmented",
        exist_ok=True,
        patience=20,
        seed=42,
        deterministic=True,
        # Lighting and colour variation.
        hsv_h=0.0,
        hsv_s=0.25,
        hsv_v=0.50,
        # Size, position, camera angle, and mild geometric distortion.
        degrees=15.0,
        translate=0.15,
        scale=0.50,
        shear=5.0,
        perspective=0.0005,
        # Never mirror directional signs: left would become right.
        fliplr=0.0,
        flipud=0.0,
        mosaic=0.60,
        mixup=0.0,
        close_mosaic=10,
    )

    run_best = Path(results.save_dir) / "weights" / "best.pt"
    expected_best = RUNS_DIR / "weights" / "best.pt"
    expected_best.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(run_best, expected_best)
    print(f"[TRAIN] Best model copied to: {expected_best}")
    print("[TRAIN] This path already matches turtlebot_autonomous.py.")


def main() -> None:
    args = parse_args()
    yaml_path = prepare_dataset(rebuild=args.rebuild)
    if args.prepare_only:
        return
    train(yaml_path, args)


if __name__ == "__main__":
    main()
