#!/usr/bin/env python3
"""Entrena YOLOv8n para detección de señales del TurtleBot."""

from __future__ import annotations

import argparse
from pathlib import Path

from ultralytics import YOLO


def main() -> None:
  root = Path(__file__).resolve().parents[1]
  parser = argparse.ArgumentParser(description="Entrenar detector de señales YOLO")
  parser.add_argument(
    "--data",
    type=Path,
    default=root / "data" / "signs.yaml",
    help="Ruta al YAML del dataset",
  )
  parser.add_argument(
    "--model",
    default="yolov8n.pt",
    help="Checkpoint base (yolov8n.pt, yolo11n.pt, etc.)",
  )
  parser.add_argument("--epochs", type=int, default=150)
  parser.add_argument("--imgsz", type=int, default=640)
  parser.add_argument("--batch", type=int, default=8)
  parser.add_argument("--device", default="", help="cuda, cpu, mps o vacío para auto")
  parser.add_argument("--project", type=Path, default=root / "runs")
  parser.add_argument("--name", default="signs_detect")
  args = parser.parse_args()

  if not args.data.exists():
    raise FileNotFoundError(
      f"No existe {args.data}. Ejecuta primero: python scripts/prepare_dataset.py --preview"
    )

  dataset_dir = root / "dataset"
  if not (dataset_dir / "images" / "train").exists():
    raise FileNotFoundError(
      f"No existe {dataset_dir}. Ejecuta primero: python scripts/prepare_dataset.py --preview"
    )

  model = YOLO(args.model)
  results = model.train(
    data=str(args.data),
    epochs=args.epochs,
    imgsz=args.imgsz,
    batch=args.batch,
    device=args.device or None,
    project=str(args.project),
    name=args.name,
    exist_ok=True,
    patience=25,
    save=True,
    plots=True,
    # Augmentación para dataset pequeño y señales pequeñas en frame
    hsv_h=0.015,
    hsv_s=0.5,
    hsv_v=0.4,
    degrees=10,
    translate=0.1,
    scale=0.4,
    shear=2.0,
    perspective=0.0005,
    flipud=0.0,
    fliplr=0.5,
    mosaic=0.5,
    mixup=0.0,
    copy_paste=0.0,
    close_mosaic=15,
  )

  best = Path(results.save_dir) / "weights" / "best.pt"
  target = root / "best.pt"
  if best.exists():
    target.write_bytes(best.read_bytes())
    print(f"\n[OK] Mejor modelo copiado a: {target}")
    print("\nProbar en imágenes:")
    print("  python scripts/predict.py --save")
    print("En el robot:")
    print("  python turtlebot.py")
  else:
    print(f"\n[WARN] No se encontró best.pt en {results.save_dir}")


if __name__ == "__main__":
  main()
