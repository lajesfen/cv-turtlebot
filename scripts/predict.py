#!/usr/bin/env python3
"""Ejecuta el modelo entrenado en modo predict (detección o clasificación).

Uso:
  python scripts/predict.py                          # sample_images + negativas
  python scripts/predict.py --source path/to/img.jpg
  python scripts/predict.py --source Fotos\\ Señales\\ Visión/Izquierda
  python scripts/predict.py --conf 0.4 --save
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
from ultralytics import YOLO

IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def collect_images(source: Path) -> list[Path]:
    if source.is_file():
        return [source]
    return sorted(
        p for p in source.rglob("*")
        if p.is_file() and p.suffix.lower() in IMAGE_EXTS
    )


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description="Predict con best.pt")
    parser.add_argument(
        "--model",
        type=Path,
        default=root / "best.pt",
        help="Checkpoint entrenado (default: best.pt)",
    )
    parser.add_argument(
        "--source",
        type=Path,
        nargs="*",
        default=None,
        help="Imagen(es) o carpeta(s). Por defecto: sample_images + Negativas",
    )
    parser.add_argument("--conf", type=float, default=0.40)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--save", action="store_true", help="Guardar imágenes anotadas")
    parser.add_argument(
        "--project",
        type=Path,
        default=root / "runs" / "predict",
    )
    parser.add_argument("--name", default="exp")
    args = parser.parse_args()

    if not args.model.exists():
        raise FileNotFoundError(f"No existe {args.model}. Entrena primero: python scripts/train.py")

    model = YOLO(str(args.model))
    is_classify = model.task == "classify"
    imgsz = 224 if is_classify else args.imgsz

    if args.source:
        sources = []
        for s in args.source:
            sources.extend(collect_images(s))
    else:
        sources = []
        for default in [
            root / "sample_images",
            root.parent / "Fotos Señales Visión" / "Negativas",
        ]:
            if default.exists():
                sources.extend(collect_images(default))

    if not sources:
        raise FileNotFoundError("No se encontraron imágenes. Usa --source.")

    print(f"Modelo: {args.model.name} | task={model.task} | conf≥{args.conf} | imgsz={imgsz}")
    print(f"Imágenes: {len(sources)}")

    if args.save:
        out_dir = args.project / args.name
        out_dir.mkdir(parents=True, exist_ok=True)
        print(f"Guardando en: {out_dir}")

    detected = 0
    for path in sources:
        if is_classify:
            result = model.predict(str(path), imgsz=imgsz, verbose=False)[0]
            if result.probs is None:
                label, conf = "—", 0.0
            else:
                conf = float(result.probs.top1conf)
                label = model.names[int(result.probs.top1)] if conf >= args.conf else "—"
            line = f"{path.name}: {label} ({conf:.2f})"
            if label != "—":
                detected += 1
            print(line)
            if args.save:
                img = cv2.imread(str(path))
                if img is not None:
                    cv2.putText(
                        img, f"{label} {conf:.2f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2,
                    )
                    cv2.imwrite(str(out_dir / path.name), img)
        else:
            result = model.predict(
                str(path), imgsz=imgsz, conf=args.conf, verbose=False
            )[0]
            if len(result.boxes) == 0:
                print(f"{path.name}: sin detección")
            else:
                detected += 1
                for box in result.boxes:
                    cls_id = int(box.cls)
                    conf = float(box.conf)
                    name = model.names[cls_id]
                    print(f"{path.name}: {name} ({conf:.2f})")
            if args.save:
                annotated = result.plot()
                cv2.imwrite(str(out_dir / path.name), annotated)

    print(f"\nCon detección/clase válida: {detected}/{len(sources)}")
    if args.save:
        print(f"Anotaciones: {args.project / args.name}")


if __name__ == "__main__":
    main()
