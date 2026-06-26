#!/usr/bin/env python3
"""Entrena YOLOv8n-cls para clasificación de señales."""

from __future__ import annotations

import argparse
from pathlib import Path

from ultralytics import YOLO


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", type=Path, default=root / "dataset_classify")
    parser.add_argument("--model", default="yolov8n-cls.pt")
    parser.add_argument("--epochs", type=int, default=80)
    parser.add_argument("--imgsz", type=int, default=224)
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--device", default="")
    parser.add_argument("--project", type=Path, default=root / "runs")
    parser.add_argument("--name", default="signs_classify")
    args = parser.parse_args()

    if not args.data.exists():
        raise FileNotFoundError("Ejecuta: python scripts/prepare_dataset_classify.py")

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
        patience=15,
        save=True,
        plots=True,
        # Augmentación para cámara en movimiento
        hsv_h=0.02,
        hsv_s=0.5,
        hsv_v=0.4,
        degrees=15,
        translate=0.1,
        scale=0.6,
        fliplr=0.5,
        erasing=0.2,
    )

    best = Path(results.save_dir) / "weights" / "best.pt"
    target = root / "best.pt"
    if best.exists():
        target.write_bytes(best.read_bytes())
        print(f"\n[OK] Modelo copiado a: {target}")


if __name__ == "__main__":
    main()
