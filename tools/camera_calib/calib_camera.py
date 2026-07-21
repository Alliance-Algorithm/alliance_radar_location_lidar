#!/usr/bin/env python3
"""相机内参标定脚本 — SHM 直读，cv2 实时显示，内存标定。

用法:
    ros2 launch radar_bringup hikcamera.launch.py  # 终端1: 启动相机
    python3 calib_camera.py --cols 11 --rows 8 --square-size 15.0  # 终端2: 标定

操作: 空格=保存当前帧  q=退出并标定  Ctrl+C=中断
"""
import sys, os, argparse, json, struct, mmap
import cv2, numpy as np

SHM_PATH  = "/dev/shm/hikcamera_shm"
WIDTH     = 5472
HEIGHT    = 3648
CHANNELS  = 3
SLOT_SIZE = 60000000
HEADER    = 128
OFF_WRITE_IDX = 80


def read_frame() -> np.ndarray | None:
    """从 SHM 读取最新帧，返回 BGR8 numpy 数组."""
    try:
        fd = os.open(SHM_PATH, os.O_RDONLY)
        write_idx = struct.unpack_from('<I',
            os.pread(fd, 4, OFF_WRITE_IDX))[0]
        slot = write_idx % 4
        offset = HEADER + slot * SLOT_SIZE
        data = os.pread(fd, WIDTH * HEIGHT * CHANNELS, offset)
        os.close(fd)
        return np.frombuffer(data, dtype=np.uint8).reshape(HEIGHT, WIDTH, CHANNELS)
    except Exception:
        return None


def calibrate(images: list, cols: int, rows: int, square_mm: float) -> dict:
    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_mm
    objpoints, imgpoints = [], []
    h, w = images[0].shape[:2]
    for i, img in enumerate(images):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCornersSB(gray, (cols, rows),
            cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE)
        if not found:
            found, corners = cv2.findChessboardCorners(gray, (cols, rows), None)
            if found:
                corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1),
                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        if not found:
            print(f"  ✗ 第{i+1}张 无法检测棋盘格, 跳过")
            continue
        objpoints.append(objp)
        imgpoints.append(corners)
        print(f"  ✓ 第{i+1}张")
    if len(objpoints) < 10:
        print(f"有效图片不足 ({len(objpoints)} < 10)")
        return {}
    ret, mtx, dist, _, _ = cv2.calibrateCamera(objpoints, imgpoints, (w, h), None, None)
    return {"image_width": w, "image_height": h,
            "camera_matrix": mtx.flatten().tolist(),
            "dist_coeffs": dist.tolist()[0],
            "rms_reprojection_error": float(ret),
            "num_samples": len(objpoints)}


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--cols", type=int, default=11)
    p.add_argument("--rows", type=int, default=8)
    p.add_argument("--square-size", type=float, default=15.0, help="棋盘格方格边长 (mm)")
    args = p.parse_args()

    title = "calib — SPACE:save  Q:quit"
    cv2.namedWindow(title, cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_EXPANDED)
    cv2.resizeWindow(title, 960, 540)
    cv2.startWindowThread()

    snapshots, saved, last_key = [], 0, -1
    print("空格=保存  q=退出标定  Ctrl+C=中断\n")
    try:
        while True:
            frame = read_frame()
            if frame is None:
                if cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
                    break
                cv2.waitKey(20)
                continue

            display = frame.copy()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCornersSB(gray, (args.cols, args.rows),
                cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE)
            if not found:
                found, corners = cv2.findChessboardCorners(gray, (args.cols, args.rows), None)
            if found:
                cv2.drawChessboardCorners(display, (args.cols, args.rows), corners, found)
            cv2.putText(display, f"Saved: {saved}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0) if found else (0, 0, 255), 2)
            cv2.imshow(title, cv2.resize(display, (960, 540)))

            key = cv2.waitKey(1) & 0xFF
            if key == 255:
                key = last_key
            else:
                last_key = key
            if key == ord(' '):
                saved += 1
                snapshots.append(frame.copy())
                print(f"  ✓ [{saved}]  {'(棋盘格已检测)' if found else '(未检测到棋盘格!)'}")
            elif key == ord('q'):
                break
    except KeyboardInterrupt:
        print("\n中断")
    finally:
        cv2.destroyAllWindows()

    if len(snapshots) < 10:
        print(f"图片不足 ({len(snapshots)} < 10), 跳过标定")
        sys.exit(0)

    r = calibrate(snapshots, args.cols, args.rows, args.square_size)
    if r:
        out = "camera_calib.json"
        with open(out, "w") as f:
            json.dump(r, f, indent=2)
        print(f"\n标定完成 → {out}")
        mtx = np.array(r["camera_matrix"]).reshape(3, 3)
        print(f"  内参: fx={mtx[0,0]:.1f} fy={mtx[1,1]:.1f} cx={mtx[0,2]:.1f} cy={mtx[1,2]:.1f}")
        print(f"  畸变: {[round(v,6) for v in r['dist_coeffs']]}")
        print(f"  RMS: {r['rms_reprojection_error']:.4f}  样本: {r['num_samples']}")


if __name__ == "__main__":
    main()
