#!/usr/bin/env python3
"""Project L1 detections onto two meshes (field_zup.obj vs walls_v2.obj).

投影公式与 ros_ws/src/radar_camera/src/projector.cpp 一致:
  x_norm=(u-cx)/fx, y_norm=(v-cy)/fy, dir_cam=normalize(x_norm,y_norm,1)
  R = Rz(yaw)*Ry(pitch)*Rx(roll); dir_world = R*dir_cam; origin = translation
Möller–Trumbore 最近命中(projector.cpp:132-165)。
仅输出坐标对比 CSV 与 stdout 汇总,不做图像可视化。
"""
import argparse, os, sys
import numpy as np

REPO = "/home/yukikaze/Documents/workspace/alliance_radar_location_lidar"
FX, FY, CX, CY = 6753.698616, 6737.450110, 2620.748274, 1924.062270
ROT_DEFAULT = [0.0, 1.8159, 3.14159]
TRANS_DEFAULT = [13.9965, 0.0800, 3.9803]
DELTA_THRESH = 0.5  # m
# 类别顺序与部署一致(radar_camera.yaml): red 0-5, blue 6-11。
# 注意 validate_3layer.cpp 的 CLASS_NAMES 是旧顺序(蓝 0-5),只影响图上标注,不影响 id 与 u,v。
CLASS_IDS = ["hero_r","eng_r","inf3_r","inf4_r","sentry_r","drone_r",
             "hero_b","eng_b","inf3_b","inf4_b","sentry_b","drone_b"]


def R_from_rpy(roll, pitch, yaw):
    rx = np.array([[1,0,0],[0,np.cos(roll),-np.sin(roll)],[0,np.sin(roll),np.cos(roll)]])
    ry = np.array([[np.cos(pitch),0,np.sin(pitch)],[0,1,0],[-np.sin(pitch),0,np.cos(pitch)]])
    rz = np.array([[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1]])
    return rz @ ry @ rx


def pixel_to_ray(u, v, R, fx=FX, fy=FY, cx=CX, cy=CY):
    dcam = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
    dcam /= np.linalg.norm(dcam)
    return R @ dcam


def mt_intersect(origin, direction, tris, eps=1e-8):
    e1 = tris[:, 1] - tris[:, 0]
    e2 = tris[:, 2] - tris[:, 0]
    pvec = np.cross(direction[None, :], e2)
    det = np.einsum("ij,ij->i", e1, pvec)
    tvec = origin - tris[:, 0]
    uu = np.einsum("ij,ij->i", tvec, pvec) / det
    qvec = np.cross(tvec, e1)
    vv = np.einsum("ij,ij->i", direction[None, :], qvec) / det
    tt = np.einsum("ij,ij->i", tris[:, 2] - tris[:, 0], qvec) / det
    ok = (np.abs(det) > eps) & (uu >= 0) & (uu <= 1) & (vv >= 0) & (uu + vv <= 1) & (tt > eps)
    if not ok.any():
        return None
    k = np.argmin(np.where(ok, tt, np.inf))
    return origin + direction * tt[k]


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("detections_csv")
    p.add_argument("--mesh-old", default=f"{REPO}/model/generated/field_zup.obj")
    p.add_argument("--mesh-new", default=f"{REPO}/model/generated/jinan_field_map_reg_walls_v2.obj")
    p.add_argument("--pose", default=None, help="roll,pitch,yaw,tx,ty,tz; --pose-format 控制角度单位")
    p.add_argument("--pose-format", default="rad", choices=["rad", "deg"])
    p.add_argument("--out", default=f"{REPO}/model/generated/ray_compare")
    p.add_argument("--threshold", type=float, default=DELTA_THRESH)
    a = p.parse_args()

    if a.pose:
        vals = [float(x) for x in a.pose.split(",")]
        if a.pose_format == "deg":
            vals[:3] = [np.deg2rad(x) for x in vals[:3]]
        rot, trans = vals[:3], vals[3:6]
    else:
        rot, trans = ROT_DEFAULT, TRANS_DEFAULT
    print(f"pose: rotation={rot} translation={trans}", flush=True)

    import trimesh
    m_old = trimesh.load(a.mesh_old, process=False)
    m_new = trimesh.load(a.mesh_new, process=False)
    tris_old = np.asarray(m_old.triangles).reshape(-1, 3, 3)
    tris_new = np.asarray(m_new.triangles).reshape(-1, 3, 3)
    print(f"faces: old={len(tris_old)} new={len(tris_new)}", flush=True)

    R = R_from_rpy(*rot)
    origin = np.array(trans, dtype=float)

    rows = []
    stats = {"old": [0, 0], "new": [0, 0]}  # [hit, total]
    with open(a.detections_csv) as f:
        header = f.readline().strip().split(",")
        for line in f:
            parts = line.strip().split(",")
            rec = dict(zip(header, parts))
            frame = rec["frame"]
            cls_raw = rec["final_class"]
            cls_id = int(cls_raw) if cls_raw.isdigit() else -1
            u, v = float(rec["u"]), float(rec["v"])
            conf = float(rec["l1_conf"])
            d = pixel_to_ray(u, v, R)
            ho = mt_intersect(origin, d, tris_old)
            hn = mt_intersect(origin, d, tris_new)
            stats["old"][0] += ho is not None; stats["old"][1] += 1
            stats["new"][0] += hn is not None; stats["new"][1] += 1
            delta = np.hypot(ho[0]-hn[0], ho[1]-hn[1]) if (ho is not None and hn is not None) else np.nan
            name = CLASS_IDS[cls_id] if 0 <= cls_id < 12 else str(cls_id)
            rows.append((frame, name, conf, u, v,
                         ho[0] if ho is not None else "", ho[1] if ho is not None else "",
                         1 if ho is not None else 0,
                         hn[0] if hn is not None else "", hn[1] if hn is not None else "",
                         1 if hn is not None else 0, delta))

    os.makedirs(a.out, exist_ok=True)
    csv_path = os.path.join(a.out, "ray_compare.csv")
    with open(csv_path, "w") as f:
        f.write("frame,class_name,conf,u,v,map_x_old,map_y_old,hit_ok_old,map_x_new,map_y_new,hit_ok_new,delta_m\n")
        for r in rows:
            f.write(",".join(str(x) for x in r) + "\n")

    deltas = [r[11] for r in rows if not np.isnan(r[11])]
    print(f"\n=== summary ===")
    for k in ("old", "new"):
        h, t = stats[k]
        print(f"mesh {k}: hit {h}/{t} = {h/t*100:.1f}%")
    if deltas:
        print(f"delta (old vs new, hit both): n={len(deltas)} mean={np.mean(deltas):.2f}m "
              f"p95={np.percentile(deltas,95):.2f}m max={np.max(deltas):.2f}m")
        print(f"delta > {a.threshold}m: {sum(d > a.threshold for d in deltas)}/{len(deltas)}")
    print(f"CSV -> {csv_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
