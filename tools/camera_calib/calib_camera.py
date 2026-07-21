#!/usr/bin/env python3
"""相机内参标定 — 先采图后标定，SHM 直读。

用法:
    python3 calib_camera.py capture              # 采图（空格保存, q退出）
    python3 calib_camera.py calibrate --cols 11 --rows 8 --square-size 15.0  # 离线标定
"""
import sys, os, argparse, json, struct
from pathlib import Path
import cv2, numpy as np

SHM_PATH      = "/dev/shm/hikcamera_shm"
WIDTH         = 5472
HEIGHT        = 3648
CHANNELS      = 3
SLOT_SIZE     = 60000000
HEADER        = 128
OFF_WRITE_IDX = 80
CAPTURE_DIR   = Path("./calib")


def read_frame() -> np.ndarray | None:
    try:
        fd = os.open(SHM_PATH, os.O_RDONLY)
        idx = struct.unpack_from('<I', os.pread(fd, 4, OFF_WRITE_IDX))[0]
        off = HEADER + (idx % 4) * SLOT_SIZE
        data = os.pread(fd, WIDTH * HEIGHT * CHANNELS, off)
        os.close(fd)
        return np.frombuffer(data, dtype=np.uint8).reshape(HEIGHT, WIDTH, CHANNELS)
    except Exception:
        return None


def do_capture(args):
    CAPTURE_DIR.mkdir(parents=True, exist_ok=True)
    title = "capture — SPACE:save  Q:quit"
    cv2.namedWindow(title, cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_EXPANDED)
    cv2.resizeWindow(title, 960, 540)
    cv2.startWindowThread()
    saved = 0
    prev_idx = -1
    print(f"保存到 {CAPTURE_DIR}/ | 空格=保存  q=退出\n")
    try:
        while True:
            fd = os.open(SHM_PATH, os.O_RDONLY)
            idx = struct.unpack_from('<I', os.pread(fd, 4, OFF_WRITE_IDX))[0]
            if idx == prev_idx:
                os.close(fd)
                if cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
                    break
                cv2.waitKey(5)
                continue
            prev_idx = idx
            off = HEADER + (idx % 4) * SLOT_SIZE
            data = os.pread(fd, WIDTH * HEIGHT * CHANNELS, off)
            os.close(fd)
            frame = np.frombuffer(data, dtype=np.uint8).reshape(HEIGHT, WIDTH, CHANNELS)
            cv2.putText(frame, f"Saved: {saved}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 3)
            cv2.imshow(title, cv2.resize(frame, (960, 540)))
            key = cv2.waitKey(1) & 0xFF
            if key == ord(' '):
                saved += 1
                path = CAPTURE_DIR / f"calib_{saved:04d}.png"
                cv2.imwrite(str(path), frame)
                print(f"  [{saved}] {path}")
            elif key == ord('q'):
                break
    except KeyboardInterrupt:
        print("\n中断")
    finally:
        cv2.destroyAllWindows()
    print(f"共采集 {saved} 张 → {CAPTURE_DIR}/")


def do_calibrate(args):
    imgs = sorted(CAPTURE_DIR.glob("*.png"))
    if not imgs:
        print(f"无图片: {CAPTURE_DIR}/")
        sys.exit(1)

    objp = np.zeros((args.cols * args.rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square_size
    objpoints, imgpoints = [], []
    h = w = 0
    for p in imgs:
        img = cv2.imread(str(p))
        if img is None:
            continue
        h, w = img.shape[:2]
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCornersSB(gray, (args.cols, args.rows),
            cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE)
        if not found:
            found, corners = cv2.findChessboardCorners(gray, (args.cols, args.rows), None)
            if found:
                corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1),
                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        if not found:
            print(f"  x {p.name}")
            continue
        objpoints.append(objp)
        imgpoints.append(corners)
        print(f"  v {p.name}")

    if len(objpoints) < 10:
        print(f"有效图片不足 ({len(objpoints)} < 10)")
        sys.exit(1)

    ret, mtx, dist, _, _ = cv2.calibrateCamera(objpoints, imgpoints, (w, h), None, None)
    r = {"image_width": w, "image_height": h,
         "camera_matrix": mtx.flatten().tolist(),
         "dist_coeffs": dist.tolist()[0],
         "rms_reprojection_error": float(ret),
         "num_samples": len(objpoints)}
    out = "camera_calib.json"
    with open(out, "w") as f:
        json.dump(r, f, indent=2)
    print(f"\n标定完成 -> {out}")
    mtx2 = np.array(r["camera_matrix"]).reshape(3, 3)
    print(f"  内参: fx={mtx2[0,0]:.1f} fy={mtx2[1,1]:.1f} cx={mtx2[0,2]:.1f} cy={mtx2[1,2]:.1f}")
    print(f"  畸变: {[round(v,6) for v in r['dist_coeffs']]}")
    print(f"  RMS: {r['rms_reprojection_error']:.4f}  样本: {r['num_samples']}")


def main():
    p = argparse.ArgumentParser()
    sp = p.add_subparsers(dest="cmd", required=True)
    sp.add_parser("capture", help="采集图片到 ./calib/")
    cp = sp.add_parser("calibrate", help="离线标定 ./calib/ 中的图片")
    cp.add_argument("--cols", type=int, default=11)
    cp.add_argument("--rows", type=int, default=8)
    cp.add_argument("--square-size", type=float, default=15.0, help="棋盘格方格边长 (mm)")
    args = p.parse_args()
    if args.cmd == "capture":
        do_capture(args)
    elif args.cmd == "calibrate":
        do_calibrate(args)

if __name__ == "__main__":
    main()
