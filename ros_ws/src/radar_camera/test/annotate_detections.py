#!/usr/bin/env python3
from pathlib import Path

import cv2
import numpy as np
import openvino as ov

MODEL = Path(__file__).resolve().parents[1] / "config" / "camera_inference_model.onnx"
IMG_DIR = Path(__file__).resolve().parent / "fixtures" / "tets"
OUT_DIR = Path(__file__).resolve().parent / "fixtures" / "annotated"

INPUT_W = INPUT_H = 1280
CONF_TH = 0.6
MIN_AR, MAX_AR = 0.5, 3.0
DRONE_MIN_AR, DRONE_MAX_AR = 2.0, 10.0
DRONE_CLASS_IDS = {5, 11}

NAMES = {
    0: "hero-red",
    1: "engineer-red",
    2: "infantry3-red",
    3: "infantry4-red",
    4: "sentry-red",
    5: "drone-red",
    6: "hero-blue",
    7: "engineer-blue",
    8: "infantry3-blue",
    9: "infantry4-blue",
    10: "sentry-blue",
    11: "drone-blue",
}


def color_for(cls: int) -> tuple[int, int, int]:
    return (40, 40, 220) if cls < 6 else (220, 120, 40)


def preprocess(bgr: np.ndarray) -> np.ndarray:
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    resized = cv2.resize(rgb, (INPUT_W, INPUT_H), interpolation=cv2.INTER_LINEAR)
    x = resized.astype(np.float32) / 255.0
    return np.transpose(x, (2, 0, 1))[None, ...]


def postprocess(raw: np.ndarray, src_w: int, src_h: int) -> list[dict]:
    rows = raw[0] if raw.ndim == 3 else raw
    scale_x = src_w / INPUT_W
    scale_y = src_h / INPUT_H
    dets: list[dict] = []
    for row in rows:
        x1, y1, x2, y2, conf, cls_f = row[:6]
        conf = float(conf)
        if conf < CONF_TH:
            continue
        cls = int(cls_f)
        bw = (float(x2) - float(x1)) * scale_x
        bh = (float(y2) - float(y1)) * scale_y
        if bw < 1.0 or bh < 1.0:
            continue
        ar = max(bw, bh) / min(bw, bh)
        if cls in DRONE_CLASS_IDS:
            if ar < DRONE_MIN_AR or ar > DRONE_MAX_AR:
                continue
        elif ar < MIN_AR or ar > MAX_AR:
            continue
        dets.append(
            {
                "x1": float(x1) * scale_x,
                "y1": float(y1) * scale_y,
                "x2": float(x2) * scale_x,
                "y2": float(y2) * scale_y,
                "conf": conf,
                "cls": cls,
                "name": NAMES.get(cls, f"id{cls}"),
            }
        )
    return dets


def draw(bgr: np.ndarray, dets: list[dict]) -> np.ndarray:
    vis = bgr.copy()
    for d in dets:
        c = color_for(d["cls"])
        p1 = (int(d["x1"]), int(d["y1"]))
        p2 = (int(d["x2"]), int(d["y2"]))
        cv2.rectangle(vis, p1, p2, c, 3)
        label = f'{d["name"]} {d["conf"]:.2f}'
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
        y0 = max(0, p1[1] - th - 8)
        cv2.rectangle(vis, (p1[0], y0), (p1[0] + tw + 6, y0 + th + 8), c, -1)
        cv2.putText(
            vis,
            label,
            (p1[0] + 3, y0 + th + 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
    return vis


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    core = ov.Core()
    compiled = core.compile_model(str(MODEL), "CPU")
    infer = compiled.create_infer_request()

    images = sorted(IMG_DIR.glob("*.jpg"))
    print(f"model={MODEL}")
    print(
        f"images={len(images)} conf>={CONF_TH} "
        f"ar=[{MIN_AR},{MAX_AR}] drone_ar=[{DRONE_MIN_AR},{DRONE_MAX_AR}]"
    )
    total = 0
    for img_path in images:
        bgr = cv2.imread(str(img_path), cv2.IMREAD_COLOR)
        if bgr is None:
            print("FAIL read", img_path)
            continue
        h, w = bgr.shape[:2]
        tensor = preprocess(bgr)
        infer.set_input_tensor(ov.Tensor(tensor))
        infer.infer()
        raw = np.array(infer.get_output_tensor().data)
        dets = postprocess(raw, w, h)
        vis = draw(bgr, dets)
        out_path = OUT_DIR / f"{img_path.stem}_vis.jpg"
        cv2.imwrite(str(out_path), vis, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
        total += len(dets)
        print(f"{img_path.name}: {w}x{h} -> {len(dets)} dets -> {out_path.name}")
        for d in dets:
            print(
                f"  {d['name']:16s} conf={d['conf']:.3f} "
                f"box=({d['x1']:.0f},{d['y1']:.0f})-({d['x2']:.0f},{d['y2']:.0f})"
            )
    print(f"wrote annotated images to {OUT_DIR}")
    print(f"total detections: {total}")


if __name__ == "__main__":
    main()
