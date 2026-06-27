#!/usr/bin/env python3
"""Convierte Fotos Señales Visión a dataset YOLO (detección).

- HEIC/PNG/JPG -> JPG
- Auto-anotación de bounding boxes en señales (Izquierda, Derecha, Bloqueo)
- Imágenes en Negativas (y sample_images del robot) sin labels → fondo
- Split train/val estratificado
- Genera data/signs.yaml y visualizaciones de revisión
"""

from __future__ import annotations

import argparse
import random
import shutil
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np

CLASS_MAP = {
    "Izquierda": ("turn_left", 0),
    "Derecha": ("turn_right", 1),
    "Bloqueo": ("stop", 2),
}

NEGATIVE_FOLDERS = ("Negativas",)
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


def auto_bbox(img: np.ndarray) -> tuple[tuple[int, int, int, int] | None, float]:
    """Detecta la señal blanca cuadrada. Devuelve (bbox, score) o (None, 0)."""
    h, w = img.shape[:2]
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    y0 = int(h * 0.15)
    roi = gray[y0:, :]
    rh, rw = roi.shape

    blurred = cv2.GaussianBlur(roi, (5, 5), 0)
    _, mask = cv2.threshold(blurred, 160, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best = None
    best_score = -1.0
    for contour in contours:
        x, y, bw, bh = cv2.boundingRect(contour)
        area = bw * bh
        if area < 0.002 * rh * rw or area > 0.15 * rh * rw:
            continue
        aspect = bw / bh if bh else 0.0
        if not (0.6 < aspect < 1.6):
            continue

        patch = roi[y : y + bh, x : x + bw]
        if patch.size == 0:
            continue
        contrast = float(patch.std())
        if contrast < 25:
            continue

        cx = x + bw / 2
        center_penalty = abs(cx - rw / 2) / (rw / 2)
        score = area * (1 - 0.5 * center_penalty) * (contrast / 50)
        if score > best_score:
            best_score = score
            best = (x, y + y0, bw, bh)

    if best is None:
        return None, 0.0

    x, y, bw, bh = best
    pad = int(max(bw, bh) * 0.15)
    x = max(0, x - pad)
    y = max(0, y - pad)
    bw = min(w - x, bw + 2 * pad)
    bh = min(h - y, bh + 2 * pad)
    return (x, y, bw, bh), best_score


def to_yolo_line(cls_id: int, bbox: tuple[int, int, int, int], w: int, h: int) -> str:
    x, y, bw, bh = bbox
    cx = (x + bw / 2) / w
    cy = (y + bh / 2) / h
    nw = bw / w
    nh = bh / h
    return f"{cls_id} {cx:.6f} {cy:.6f} {nw:.6f} {nh:.6f}"


def collect_positive(source: Path) -> list[tuple[Path, str, int]]:
    samples: list[tuple[Path, str, int]] = []
    for folder_name, (_, cls_id) in CLASS_MAP.items():
        folder = source / folder_name
        if not folder.is_dir():
            print(f"[WARN] Carpeta no encontrada: {folder}")
            continue
        for path in sorted(folder.iterdir()):
            if path.suffix in IMAGE_EXTS and path.is_file():
                samples.append((path, folder_name, cls_id))
    return samples


def collect_negatives(source: Path, extra_dirs: list[Path]) -> list[Path]:
    negatives: list[Path] = []
    for folder_name in NEGATIVE_FOLDERS:
        folder = source / folder_name
        if folder.is_dir():
            for path in sorted(folder.iterdir()):
                if path.suffix.lower() in {e.lower() for e in IMAGE_EXTS} and path.is_file():
                    negatives.append(path)
        else:
            print(f"[WARN] Carpeta negativas no encontrada: {folder}")

    for extra in extra_dirs:
        if extra.is_dir():
            for path in sorted(extra.glob("*")):
                if path.suffix.lower() in {".png", ".jpg", ".jpeg"} and path.is_file():
                    negatives.append(path)
    return negatives


def split_items(items: list, val_ratio: float, rng: random.Random) -> tuple[list, list]:
    shuffled = items.copy()
    rng.shuffle(shuffled)
    if len(shuffled) <= 1:
        return shuffled, []
    n_val = max(1, int(len(shuffled) * val_ratio))
    return shuffled[n_val:], shuffled[:n_val]


def write_yaml(output_dir: Path, yaml_path: Path) -> None:
    yaml_path.parent.mkdir(parents=True, exist_ok=True)
    content = f"""# Dataset de señales TurtleBot — generado por prepare_dataset.py
path: {output_dir.resolve()}
train: images/train
val: images/val

names:
  0: turn_left
  1: turn_right
  2: stop
"""
    yaml_path.write_text(content, encoding="utf-8")


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description="Preparar dataset YOLO desde Fotos Señales Visión")
    parser.add_argument(
        "--source",
        type=Path,
        default=root.parent / "Fotos Señales Visión",
    )
    parser.add_argument("--output", type=Path, default=root / "dataset")
    parser.add_argument(
        "--extra-negatives",
        type=Path,
        default=root / "sample_images",
        help="Carpeta adicional de imágenes sin señales (p.ej. sample_images)",
    )
    parser.add_argument("--val-ratio", type=float, default=0.2)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--preview", action="store_true")
    args = parser.parse_args()

    if not args.source.exists():
        print(f"[ERROR] No existe la carpeta fuente: {args.source}")
        return 1

    positives = collect_positive(args.source)
    negatives = collect_negatives(args.source, [args.extra_negatives])
    if not positives:
        print("[ERROR] No se encontraron imágenes con señales.")
        return 1

    rng = random.Random(args.seed)

    by_class: dict[str, list[tuple[Path, str, int]]] = {k: [] for k in CLASS_MAP}
    for item in positives:
        by_class[item[1]].append(item)

    train_pos: list[tuple[Path, str, int]] = []
    val_pos: list[tuple[Path, str, int]] = []
    for class_items in by_class.values():
        tr, va = split_items(class_items, args.val_ratio, rng)
        train_pos.extend(tr)
        val_pos.extend(va)

    train_neg, val_neg = split_items(negatives, args.val_ratio, rng)

    if args.output.exists():
        shutil.rmtree(args.output)
    for split in ("train", "val"):
        (args.output / "images" / split).mkdir(parents=True, exist_ok=True)
        (args.output / "labels" / split).mkdir(parents=True, exist_ok=True)
    if args.preview:
        (args.output / "preview").mkdir(parents=True, exist_ok=True)

    stats = {
        "train_pos": 0, "val_pos": 0,
        "train_neg": 0, "val_neg": 0,
        "failed": 0, "bbox_fallback": 0,
    }

    def process_positives(batch: list[tuple[Path, str, int]], split: str) -> None:
        for src, folder_name, cls_id in batch:
            stem = f"{folder_name.lower()}_{src.stem}"[:80]
            jpg_path = args.output / "images" / split / f"{stem}.jpg"
            label_path = args.output / "labels" / split / f"{stem}.txt"

            if not convert_to_jpg(src, jpg_path):
                print(f"[FAIL] No se pudo convertir: {src}")
                stats["failed"] += 1
                continue

            img = cv2.imread(str(jpg_path))
            if img is None:
                stats["failed"] += 1
                continue

            h, w = img.shape[:2]
            bbox, score = auto_bbox(img)
            if bbox is None:
                bw = int(w * 0.22)
                bh = int(h * 0.22)
                x = (w - bw) // 2
                y = int(h * 0.35)
                bbox = (x, y, bw, bh)
                stats["bbox_fallback"] += 1
                print(f"[WARN] bbox fallback: {src.name}")

            label_path.write_text(to_yolo_line(cls_id, bbox, w, h) + "\n", encoding="utf-8")
            stats[f"{split}_pos"] += 1

            if args.preview:
                vis = img.copy()
                x, y, bw, bh = bbox
                cv2.rectangle(vis, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
                label_name = CLASS_MAP[folder_name][0]
                cv2.putText(vis, label_name, (x, max(20, y - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.imwrite(str(args.output / "preview" / f"{split}_{stem}.jpg"), vis)

    def process_negatives(batch: list[Path], split: str) -> None:
        for src in batch:
            stem = f"neg_{src.stem}"[:80]
            jpg_path = args.output / "images" / split / f"{stem}.jpg"
            if not convert_to_jpg(src, jpg_path):
                print(f"[FAIL] Negativa no convertida: {src}")
                stats["failed"] += 1
                continue
            # Sin archivo .txt → imagen de fondo para YOLO
            if split == "train":
                stats["train_neg"] += 1
            else:
                stats["val_neg"] += 1

    process_positives(train_pos, "train")
    process_positives(val_pos, "val")
    process_negatives(train_neg, "train")
    process_negatives(val_neg, "val")

    yaml_path = root / "data" / "signs.yaml"
    write_yaml(args.output, yaml_path)

    print("\n=== Dataset detección preparado ===")
    print(f"Fuente:        {args.source}")
    print(f"Salida:        {args.output}")
    print(f"YAML:          {yaml_path}")
    print(f"Train:         {stats['train_pos']} con señal + {stats['train_neg']} negativas")
    print(f"Val:           {stats['val_pos']} con señal + {stats['val_neg']} negativas")
    print(f"Bbox fallback: {stats['bbox_fallback']}")
    print(f"Fallos:        {stats['failed']}")
    if args.preview:
        print(f"Previews:      {args.output / 'preview'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
