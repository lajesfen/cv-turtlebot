#!/usr/bin/env python3
"""Prepara dataset YOLO Clasificación desde Fotos Señales Visión."""

from __future__ import annotations

import argparse
import random
import shutil
import subprocess
import sys
from pathlib import Path

import cv2

CLASS_MAP = {
    "Izquierda": "turn_left",
    "Derecha": "turn_right",
    "Bloqueo": "stop",
}
IMAGE_EXTS = {".png", ".jpg", ".jpeg", ".heic", ".HEIC"}


def convert_to_jpg(src: Path, dst: Path) -> bool:
    dst.parent.mkdir(parents=True, exist_ok=True)
    if src.suffix.lower() == ".heic":
        result = subprocess.run(
            ["sips", "-s", "format", "jpeg", str(src), "--out", str(dst)],
            capture_output=True,
            text=True,
        )
        return result.returncode == 0 and dst.exists()
    img = cv2.imread(str(src))
    if img is None:
        return False
    return cv2.imwrite(str(dst), img, [cv2.IMWRITE_JPEG_QUALITY, 95])


def collect_samples(source: Path) -> list[tuple[Path, str]]:
    samples: list[tuple[Path, str]] = []
    for folder_name, class_name in CLASS_MAP.items():
        folder = source / folder_name
        if not folder.is_dir():
            continue
        for path in sorted(folder.iterdir()):
            if path.suffix in IMAGE_EXTS and path.is_file():
                samples.append((path, class_name))
    return samples


def write_yaml(output_dir: Path, yaml_path: Path) -> None:
    yaml_path.parent.mkdir(parents=True, exist_ok=True)
    content = f"""# Dataset clasificación señales TurtleBot
path: {output_dir.resolve()}
train: train
val: val

names:
  0: turn_left
  1: turn_right
  2: stop
"""
    yaml_path.write_text(content, encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--source",
        type=Path,
        default=Path(__file__).resolve().parents[2] / "Fotos Señales Visión",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "dataset_classify",
    )
    parser.add_argument("--val-ratio", type=float, default=0.2)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    samples = collect_samples(args.source)
    if not samples:
        print("[ERROR] No se encontraron imágenes.")
        return 1

    rng = random.Random(args.seed)
    by_class: dict[str, list[tuple[Path, str]]] = {v: [] for v in CLASS_MAP.values()}
    for item in samples:
        by_class[item[1]].append(item)

    if args.output.exists():
        shutil.rmtree(args.output)

    stats = {"train": 0, "val": 0, "failed": 0}
    for class_name, class_items in by_class.items():
        rng.shuffle(class_items)
        n_val = max(1, int(len(class_items) * args.val_ratio)) if len(class_items) > 1 else 0
        splits = [("val", class_items[:n_val]), ("train", class_items[n_val:])]
        for split, batch in splits:
            for idx, (src, _) in enumerate(batch):
                dst = args.output / split / class_name / f"{class_name}_{idx:03d}.jpg"
                if convert_to_jpg(src, dst):
                    stats[split] += 1
                else:
                    stats["failed"] += 1

    yaml_path = Path(__file__).resolve().parents[1] / "data" / "signs_classify.yaml"
    write_yaml(args.output, yaml_path)

    print("\n=== Dataset clasificación listo ===")
    print(f"Train: {stats['train']} | Val: {stats['val']} | Fallos: {stats['failed']}")
    print(f"YAML:  {yaml_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
