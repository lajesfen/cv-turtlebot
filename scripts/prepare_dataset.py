#!/usr/bin/env python3
"""Convierte Fotos Señales Visión a dataset YOLO (detección).

- HEIC/PNG -> JPG
- Auto-anotación de bounding boxes (señal blanca con símbolo negro)
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


def auto_bbox(img: np.ndarray) -> tuple[int, int, int, int]:
  """Detecta la señal blanca cuadrada en la parte media-baja del frame."""
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

  if best:
    x, y, bw, bh = best
    pad = int(max(bw, bh) * 0.15)
    x = max(0, x - pad)
    y = max(0, y - pad)
    bw = min(w - x, bw + 2 * pad)
    bh = min(h - y, bh + 2 * pad)
    return x, y, bw, bh

  bw = int(w * 0.22)
  bh = int(h * 0.22)
  x = (w - bw) // 2
  y = int(h * 0.35)
  return x, y, bw, bh


def to_yolo_line(cls_id: int, bbox: tuple[int, int, int, int], w: int, h: int) -> str:
  x, y, bw, bh = bbox
  cx = (x + bw / 2) / w
  cy = (y + bh / 2) / h
  nw = bw / w
  nh = bh / h
  return f"{cls_id} {cx:.6f} {cy:.6f} {nw:.6f} {nh:.6f}"


def collect_samples(source: Path) -> list[tuple[Path, str, int]]:
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
  parser = argparse.ArgumentParser(description="Preparar dataset YOLO desde Fotos Señales Visión")
  parser.add_argument(
    "--source",
    type=Path,
    default=Path(__file__).resolve().parents[2] / "Fotos Señales Visión",
    help="Carpeta con subcarpetas Izquierda, Derecha, Bloqueo",
  )
  parser.add_argument(
    "--output",
    type=Path,
    default=Path(__file__).resolve().parents[1] / "dataset",
    help="Directorio de salida del dataset YOLO",
  )
  parser.add_argument("--val-ratio", type=float, default=0.2, help="Fracción para validación")
  parser.add_argument("--seed", type=int, default=42)
  parser.add_argument("--preview", action="store_true", help="Guardar imágenes con bbox dibujado")
  args = parser.parse_args()

  if not args.source.exists():
    print(f"[ERROR] No existe la carpeta fuente: {args.source}")
    return 1

  samples = collect_samples(args.source)
  if not samples:
    print("[ERROR] No se encontraron imágenes.")
    return 1

  rng = random.Random(args.seed)
  by_class: dict[str, list[tuple[Path, str, int]]] = {k: [] for k in CLASS_MAP}
  for item in samples:
    by_class[item[1]].append(item)

  train_samples: list[tuple[Path, str, int]] = []
  val_samples: list[tuple[Path, str, int]] = []
  for class_items in by_class.values():
    rng.shuffle(class_items)
    n_val = max(1, int(len(class_items) * args.val_ratio)) if len(class_items) > 1 else 0
    val_samples.extend(class_items[:n_val])
    train_samples.extend(class_items[n_val:])

  if args.output.exists():
    shutil.rmtree(args.output)
  for split in ("train", "val"):
    (args.output / "images" / split).mkdir(parents=True, exist_ok=True)
    (args.output / "labels" / split).mkdir(parents=True, exist_ok=True)
  if args.preview:
    (args.output / "preview").mkdir(parents=True, exist_ok=True)

  stats = {"converted": 0, "failed": 0, "train": 0, "val": 0}

  def process_batch(batch: list[tuple[Path, str, int]], split: str) -> None:
    for idx, (src, folder_name, cls_id) in enumerate(batch):
      stem = f"{folder_name.lower()}_{idx:03d}"
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
      bbox = auto_bbox(img)
      label_path.write_text(to_yolo_line(cls_id, bbox, w, h) + "\n", encoding="utf-8")
      stats["converted"] += 1
      stats[split] += 1

      if args.preview:
        vis = img.copy()
        x, y, bw, bh = bbox
        cv2.rectangle(vis, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
        label_name = CLASS_MAP[folder_name][0]
        cv2.putText(
          vis,
          label_name,
          (x, max(20, y - 8)),
          cv2.FONT_HERSHEY_SIMPLEX,
          0.6,
          (0, 255, 0),
          2,
        )
        cv2.imwrite(str(args.output / "preview" / f"{split}_{stem}.jpg"), vis)

  process_batch(train_samples, "train")
  process_batch(val_samples, "val")

  yaml_path = Path(__file__).resolve().parents[1] / "data" / "signs.yaml"
  write_yaml(args.output, yaml_path)

  print("\n=== Dataset preparado ===")
  print(f"Fuente:     {args.source}")
  print(f"Salida:     {args.output}")
  print(f"YAML:       {yaml_path}")
  print(f"Train:      {stats['train']}")
  print(f"Val:        {stats['val']}")
  print(f"Fallos:     {stats['failed']}")
  if args.preview:
    print(f"Previews:   {args.output / 'preview'}")
  print("\nRevisa las previews y corrige labels manualmente si algún bbox está mal.")
  return 0


if __name__ == "__main__":
  sys.exit(main())
