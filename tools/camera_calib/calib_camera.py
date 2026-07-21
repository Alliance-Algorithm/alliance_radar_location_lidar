#!/usr/bin/env python3
"""相机内参标定 — SHM 直读采图 + 离线标定。

用法:
    python3 calib_camera.py capture [-d ./calib]
    python3 calib_camera.py calibrate [-d ./calib] --cols 11 --rows 8 --square-size 15.0
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


def do_capture(args):
    d = Path(args.image_dir)
    d.mkdir(parents=True, exist_ok=True)
    title = "capture — SPACE:save  Q:quit"
    cv2.namedWindow(title, cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_EXPANDED)
    cv2.resizeWindow(title, 960, 540)
    cv2.startWindowThread()
    saved, prev_idx = 0, -1
    print(f"保存到 {d}/ | 空格=保存  q=退出\n")
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
                path = d / f"calib_{saved:04d}.png"
                cv2.imwrite(str(path), frame)
                print(f"  [{saved}] {path}", flush=True)
            elif key == ord('q'):
                break
    except KeyboardInterrupt:
        print("\n中断", flush=True)
    finally:
        cv2.destroyAllWindows()
    print(f"共采集 {saved} 张 → {d}/", flush=True)


def do_calibrate(args):
    d = Path(args.image_dir)
    imgs = sorted(d.glob("*.png"))
    if not imgs:
        print(f"无图片: {d}/", flush=True)
        sys.exit(1)
    print(f"找到 {len(imgs)} 张图片, 开始检测棋盘格...\n", flush=True)

    objp = np.zeros((args.cols * args.rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square_size
    objpoints, imgpoints = [], []
    h = w = 0
    for i, p in enumerate(imgs):
        img = cv2.imread(str(p))
        if img is None:
            continue
        h, w = img.shape[:2]
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCornersSB(gray, (args.cols, args.rows))
        if not found:
            found, corners = cv2.findChessboardCorners(gray, (args.cols, args.rows), None)
        if found:
            corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        status = "v" if found else "x"
        print(f"  [{i+1}/{len(imgs)}] {status} {p.name}", flush=True)
        if found:
            objpoints.append(objp)
            imgpoints.append(corners)

    ok = len(objpoints)
    print(f"\n有效: {ok}/{len(imgs)}", flush=True)
    if ok < 10:
        print(f"不足 10 张, 跳过标定", flush=True)
        sys.exit(1)

    print("标定中...", flush=True)
    ret, mtx, dist, _, _ = cv2.calibrateCamera(objpoints, imgpoints, (w, h), None, None)
    r = {"image_width": w, "image_height": h,
         "camera_matrix": mtx.flatten().tolist(),
         "dist_coeffs": dist.tolist()[0],
         "rms_reprojection_error": float(ret),
         "num_samples": ok}
    out = "camera_calib.json"
    with open(out, "w") as f:
        json.dump(r, f, indent=2)
    print(f"\n标定完成 → {out}", flush=True)
    mtx2 = np.array(r["camera_matrix"]).reshape(3, 3)
    print(f"  内参: fx={mtx2[0,0]:.1f} fy={mtx2[1,1]:.1f} cx={mtx2[0,2]:.1f} cy={mtx2[1,2]:.1f}", flush=True)
    print(f"  畸变: {[round(v,6) for v in r['dist_coeffs']]}", flush=True)
    print(f"  RMS: {r['rms_reprojection_error']:.4f}  样本: {ok}", flush=True)


def main():
    p = argparse.ArgumentParser()
    sp = p.add_subparsers(dest="cmd")
    sp.required = True

    cap = sp.add_parser("capture", help="采集图片")
    cap.add_argument("-d", "--image-dir", default="./calib", help="图片保存目录 (默认 ./calib)")

    cal = sp.add_parser("calibrate", help="离线标定")
    cal.add_argument("-d", "--image-dir", default="./calib", help="图片目录 (默认 ./calib)")
    cal.add_argument("--cols", type=int, default=11)
    cal.add_argument("--rows", type=int, default=8)
    cal.add_argument("--square-size", type=float, default=15.0, help="方格边长 mm")

    args = p.parse_args()
    if args.cmd == "capture":
        do_capture(args)
    elif args.cmd == "calibrate":
        do_calibrate(args)


if __name__ == "__main__":
    main()
