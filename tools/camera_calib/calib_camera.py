#!/usr/bin/env python3
"""相机内参标定脚本 — 终端交互采集图像，OpenCV 标定。

用法:
    # 终端1: 相机
    ros2 launch radar_bringup hikcamera.launch.py
    # 终端2: 看图
    ros2 run rqt_image_view rqt_image_view /camera/image_raw
    # 终端3: 标定采集
    python3 calib_camera.py --cols 11 --rows 8 --square-size 15.0

操作: Enter=保存当前帧  q=退出并标定
"""
import sys, time, argparse, json, select, termios, tty
from pathlib import Path
import cv2, numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def _get_key(timeout: float = 0.05) -> str:
    """非阻塞读取终端单字符."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if r else ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

class ManualCalib(Node):
    def __init__(self, output_dir: Path, chess_cols: int, chess_rows: int, square_mm: float):
        super().__init__("manual_calib")
        self.bridge = CvBridge()
        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.chess_size = (chess_cols, chess_rows)
        self.square_mm = square_mm
        self.frame = None
        self.saved = 0
        self.sub = self.create_subscription(Image, "/camera/image_raw", self._on_image, 10)
        self.get_logger().info("终端交互模式: Enter=保存  q=退出标定  (用 rqt_image_view 看图)")

    def _on_image(self, msg: Image):
        self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def run(self):
        snapshots = []
        last_status = ""
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.frame is None:
                continue
            gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCornersSB(gray, self.chess_size,
                cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE)
            if not found:
                found, corners = cv2.findChessboardCorners(gray, self.chess_size, None)

            status = f"\r  棋盘格: {'✓ 检测到' if found else '✗ 未检测'} | 已保存: {self.saved} | Enter=保存  q=退出 "
            if status != last_status:
                sys.stdout.write(status)
                sys.stdout.flush()
                last_status = status

            key = _get_key(0.02)
            if key in ('\r', '\n', ' '):
                self.saved += 1
                path = self.output_dir / f"calib_{self.saved:04d}.png"
                cv2.imwrite(str(path), self.frame)
                snapshots.append(path)
                print(f"\n  ✓ [{self.saved}] {path}  {'(棋盘格已检测)' if found else '(未检测到棋盘格!)'}")
                last_status = ""
            elif key in ('q', 'Q', '\x1b'):
                print()
                break
        print(f"\n共采集 {self.saved} 张图像")
        return snapshots


def calibrate_from_images(paths, cols, rows, square_mm):
    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_mm
    objpoints, imgpoints, good_paths = [], [], []
    h = w = 0
    for p in paths:
        img = cv2.imread(str(p))
        if img is None: continue
        h, w = img.shape[:2]
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCornersSB(gray, (cols, rows),
            cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE)
        if not found:
            found, corners = cv2.findChessboardCorners(gray, (cols, rows), None)
            if found:
                corners = cv2.cornerSubPix(gray, corners, (5,5), (-1,-1),
                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        if not found:
            print(f"  ✗ {p.name} 无法检测棋盘格, 跳过")
            continue
        objpoints.append(objp)
        imgpoints.append(corners)
        good_paths.append(p)
        print(f"  ✓ {p.name}")
    if len(objpoints) < 10:
        print(f"有效图片不足 ({len(objpoints)} < 10)")
        return {}
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, (w, h), None, None)
    return {"image_width": w, "image_height": h,
            "camera_matrix": mtx.flatten().tolist(), "dist_coeffs": dist.tolist()[0],
            "rms_reprojection_error": float(ret), "num_samples": len(objpoints)}

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--image-dir", default="./calib_data")
    p.add_argument("--cols", type=int, default=11)
    p.add_argument("--rows", type=int, default=8)
    p.add_argument("--square-size", type=float, default=15.0)
    p.add_argument("--only-calibrate", action="store_true", help="只标定已有图片")
    args = p.parse_args()
    d = Path(args.image_dir)

    if args.only_calibrate:
        imgs = sorted(d.glob("*.png"))
        if not imgs:
            print(f"无图片: {d}")
            sys.exit(1)
        r = calibrate_from_images(imgs, args.cols, args.rows, args.square_size)
    else:
        rclpy.init()
        node = ManualCalib(d, args.cols, args.rows, args.square_size)
        imgs = node.run()
        node.destroy_node()
        rclpy.shutdown()
        if len(imgs) < 10:
            print(f"图片不足 ({len(imgs)} < 10), 跳过标定")
            sys.exit(0)
        r = calibrate_from_images(imgs, args.cols, args.rows, args.square_size)

    if r:
        out = d / "camera_calib.json"
        out.write_text(json.dumps(r, indent=2))
        print(f"\n标定完成 → {out}")
        mtx = np.array(r["camera_matrix"]).reshape(3, 3)
        print(f"  内参: fx={mtx[0,0]:.1f} fy={mtx[1,1]:.1f} cx={mtx[0,2]:.1f} cy={mtx[1,2]:.1f}")
        print(f"  畸变: {[round(v,6) for v in r['dist_coeffs']]}")
        print(f"  RMS: {r['rms_reprojection_error']:.4f}  样本: {r['num_samples']}")

if __name__ == "__main__":
    main()
