#!/usr/bin/env python3
"""Validate the L1->L2->L3 three-layer fusion on a folder of frames.

Mirrors the C++ radar_camera::armor_refine::ArmorRefiner decode logic:
  - L1 (best_fixed_names_1280): YOLO detect, end2end NMS, [1,300,6] xyxy+conf+cls
      class ids: BLUE 0-5, RED 6-11 (0 hero .. 4 sentry, 5 drone; +6 for red)
  - L2 (shenzhen-0708): [1,25200,22] raw grid
      col 0-7 = 4 corner pts, col 8 = obj(sigmoid), col 9-12 = color, col 13-21 = genre
  - L3 (armor-number): classify [1,9] -> B1,B2,B3,B4,BS,R1,R2,R3,R4

Fusion is pure priority: L3>=thr overrides L2>=thr overrides L1. Drones skip L2/L3.
Also reports per-layer inference latency (CPU / onnxruntime).
"""
import argparse
import csv
import time
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--model-dir", default="/workspace/ros_ws/src/radar_camera/model",
                    help="directory holding the L1/L2/L3 .onnx models")
parser.add_argument("--frames", default="/workspace/tools/verify_frames",
                    help="directory of input frames (*.jpg)")
parser.add_argument("--out-dir", default="/workspace/tools/verify_output/l1l2l3_verify",
                    help="directory for annotated images + CSV")
args = parser.parse_args()

MODEL_DIR = Path(args.model_dir)
FRAMES = Path(args.frames)
OUT_DIR = Path(args.out_dir)
OUT_DIR.mkdir(parents=True, exist_ok=True)

L1_CONF = 0.30
L2_CONF = 0.80
L2_NMS = 0.30
L3_CONF = 0.80

DRONE_IDS = {5, 11}

L1_NAMES = {0: "hero-b", 1: "eng-b", 2: "inf3-b", 3: "inf4-b", 4: "sentry-b", 5: "drone-b",
            6: "hero-r", 7: "eng-r", 8: "inf3-r", 9: "inf4-r", 10: "sentry-r", 11: "drone-r"}
L3_NAMES = ["B1", "B2", "B3", "B4", "BS", "R1", "R2", "R3", "R4"]


def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


def l3_idx_to_class_id(idx):
    # 0..4 -> blue 0..4 ; 5..8 -> red 6..9
    return {0: 0, 1: 1, 2: 2, 3: 3, 4: 4, 5: 6, 6: 7, 7: 8, 8: 9}.get(idx)


def l2_to_class_id(genre, color):  # color: 1=red, 2=blue
    blue = {1: 0, 2: 1, 3: 2, 4: 3, 6: 4}.get(genre)
    if blue is None or color == 0:
        return None
    return blue if color == 2 else blue + 6


def load(name):
    return ort.InferenceSession(str(MODEL_DIR / name), providers=["CPUExecutionProvider"])


def letterbox(img, side, center):
    h, w = img.shape[:2]
    s = min(side / w, side / h)
    nw, nh = max(1, round(w * s)), max(1, round(h * s))
    r = cv2.resize(img, (nw, nh))
    canvas = np.zeros((side, side, 3), np.uint8)
    px = (side - nw) // 2 if center else 0
    py = (side - nh) // 2 if center else 0
    canvas[py:py + nh, px:px + nw] = r
    return canvas, s, px, py


def blob(img_rgb, side):
    x = img_rgb.astype(np.float32) / 255.0
    x = np.transpose(x, (2, 0, 1))[None]
    return np.ascontiguousarray(x)


def run_l1(sess, frame_rgb):
    side = 1280
    lb, s, _, _ = letterbox(frame_rgb, side, center=False)
    t = time.perf_counter()
    out = sess.run(None, {sess.get_inputs()[0].name: blob(lb, side)})[0]
    dt = (time.perf_counter() - t) * 1000
    out = out[0]  # [300,6]
    dets = []
    for x1, y1, x2, y2, conf, cls in out:
        if conf < L1_CONF:
            continue
        dets.append((int(cls), float(conf),
                     x1 / s, y1 / s, x2 / s, y2 / s))
    return dets, dt


def run_l2(sess, frame_rgb, box):
    x1, y1, x2, y2 = box
    x1 = max(0, int(x1)); y1 = max(0, int(y1))
    x2 = min(frame_rgb.shape[1], int(x2)); y2 = min(frame_rgb.shape[0], int(y2))
    if x2 - x1 < 2 or y2 - y1 < 2:
        return None, 0.0
    crop = frame_rgb[y1:y2, x1:x2]
    side = 640
    lb, s, px, py = letterbox(crop, side, center=False)
    t = time.perf_counter()
    out = sess.run(None, {sess.get_inputs()[0].name: blob(lb, side)})[0]
    dt = (time.perf_counter() - t) * 1000
    out = out[0]  # [25200,22]
    conf = sigmoid(out[:, 8])
    keep = conf >= L2_CONF
    out = out[keep]; conf = conf[keep]
    if len(out) == 0:
        return None, dt
    corners = out[:, 0:8].reshape(-1, 4, 2)
    xs = corners[:, :, 0]; ys = corners[:, :, 1]
    bx1 = (xs.min(1) - px) / s + x1
    by1 = (ys.min(1) - py) / s + y1
    bx2 = (xs.max(1) - px) / s + x1
    by2 = (ys.max(1) - py) / s + y1
    boxes = np.stack([bx1, by1, bx2 - bx1, by2 - by1], 1)
    idxs = cv2.dnn.NMSBoxes(boxes.tolist(), conf.tolist(), L2_CONF, L2_NMS)
    if len(idxs) == 0:
        return None, dt
    best = int(np.array(idxs).flatten()[0])
    color = int(out[best, 9:13].argmax())
    genre = int(out[best, 13:22].argmax())
    return (float(bx1[best]), float(by1[best]), float(bx2[best]), float(by2[best]),
            genre, color, float(conf[best])), dt


def run_l3(sess, frame_rgb, box):
    x1, y1, x2, y2 = box
    x1 = max(0, int(x1)); y1 = max(0, int(y1))
    x2 = min(frame_rgb.shape[1], int(x2)); y2 = min(frame_rgb.shape[0], int(y2))
    if x2 - x1 < 2 or y2 - y1 < 2:
        return None, 0.0
    crop = frame_rgb[y1:y2, x1:x2]
    side = 224
    lb, _, _, _ = letterbox(crop, side, center=True)
    t = time.perf_counter()
    out = sess.run(None, {sess.get_inputs()[0].name: blob(lb, side)})[0]
    dt = (time.perf_counter() - t) * 1000
    # Model output is already softmax probabilities — use directly.
    p = out[0]
    idx = int(p.argmax())
    return (idx, float(p[idx])), dt


def main():
    l1 = load("best_fixed_names_1280.onnx")
    l2 = load("shenzhen-0708.onnx")
    l3 = load("armor-number.onnx")

    lat = {"l1": [], "l2": [], "l3": []}
    stat = {"robots": 0, "drones": 0, "l3_hit": 0, "l2_hit": 0, "l1_fallback": 0}
    rows = []

    files = sorted(FRAMES.glob("*.jpg"))
    for fi, fp in enumerate(files):
        bgr = cv2.imread(str(fp))
        if bgr is None:
            continue
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        dets, d1 = run_l1(l1, rgb)
        lat["l1"].append(d1)
        vis = bgr.copy()
        for cls, conf, x1, y1, x2, y2 in dets:
            layer = "L1"
            final = cls
            box = (x1, y1, x2, y2)
            if cls in DRONE_IDS:
                stat["drones"] += 1
            else:
                stat["robots"] += 1
                r3, d3 = run_l3(l3, rgb, box)
                lat["l3"].append(d3)
                decided = False
                if r3 and r3[1] >= L3_CONF:
                    cid = l3_idx_to_class_id(r3[0])
                    if cid is not None:
                        final = cid; layer = "L3"; stat["l3_hit"] += 1; decided = True
                if not decided:
                    r2, d2 = run_l2(l2, rgb, box)
                    lat["l2"].append(d2)
                    if r2:
                        pb = (r2[0], r2[1], r2[2], r2[3])
                        r3b, d3b = run_l3(l3, rgb, pb)
                        lat["l3"].append(d3b)
                        if r3b and r3b[1] >= L3_CONF:
                            cid = l3_idx_to_class_id(r3b[0])
                            if cid is not None:
                                final = cid; layer = "L3"; stat["l3_hit"] += 1; decided = True
                        if not decided:
                            cid = l2_to_class_id(r2[4], r2[5])
                            if cid is not None:
                                final = cid; layer = "L2"; stat["l2_hit"] += 1; decided = True
                if not decided:
                    stat["l1_fallback"] += 1
            color = (0, 255, 0) if layer == "L3" else (0, 165, 255) if layer == "L2" else (200, 200, 200)
            cv2.rectangle(vis, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
            label = f"{L1_NAMES.get(final, final)} [{layer}] L1={L1_NAMES.get(cls, cls)}"
            cv2.putText(vis, label, (int(x1), max(0, int(y1) - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            rows.append([fp.name, L1_NAMES.get(cls, cls), layer, L1_NAMES.get(final, final),
                         f"{conf:.2f}"])
        cv2.imwrite(str(OUT_DIR / fp.name), vis)

    with open(OUT_DIR / "l1l2l3_verify.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame", "l1_class", "decision_layer", "final_class", "l1_conf"])
        w.writerows(rows)

    def stats(a):
        a = np.array(a) if a else np.array([0.0])
        return f"mean={a.mean():.1f}ms p50={np.percentile(a,50):.1f} p95={np.percentile(a,95):.1f} n={len(a)}"

    print("=== latency (CPU onnxruntime, NOT deployment GPU) ===")
    print("L1", stats(lat["l1"]))
    print("L2", stats(lat["l2"]))
    print("L3", stats(lat["l3"]))
    print("=== decision stats ===")
    print(stat)
    total = stat["l3_hit"] + stat["l2_hit"] + stat["l1_fallback"]
    if total:
        print(f"L3={stat['l3_hit']}/{total}={stat['l3_hit']/total*100:.0f}%  "
              f"L2={stat['l2_hit']}/{total}={stat['l2_hit']/total*100:.0f}%  "
              f"L1fb={stat['l1_fallback']}/{total}={stat['l1_fallback']/total*100:.0f}%")
    print("out:", OUT_DIR)


if __name__ == "__main__":
    main()
