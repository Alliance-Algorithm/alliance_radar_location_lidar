# Camera 射线地图对比实现计划(field_zup.obj vs walls_v2 mesh)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 从 `jinan_field_map_reg_walls_v2.pcd` 生成带墙 mesh,与现有 `field_zup.obj` 在 100 张抽帧图的 3 层推理结果上做射线投影对比,评估哪个效果更好。

**Architecture:** 三个独立组件:①open3d Ball Pivoting 生成 OBJ;②改造现有 `validate_3layer.cpp` 输出检测中心像素坐标并重跑 100 帧;③Python 脚本(投影公式与 projector.cpp 一致)对两个 mesh 分别求交,输出对比 CSV + 每帧可视化。

**Tech Stack:** Python 3.11(venv: `/home/yukikaze/Documents/workspace/alliance_radar_location_lidar/.venv`,已有 numpy/open3d/trimesh/opencv)、C++23 + TensorRT(宿主机 `/home/yukikaze/radar_trt_stage`)、OpenCV。

## Global Constraints

- 相机内参(部署版标定,5472×3648):`fx=6753.698616, fy=6737.450110, cx=2620.748274, cy=1924.062270`,畸变系数全 0(来自 `ros_ws/src/radar_bringup/config/camera/radar_camera.yaml`)
- 相机外参(蓝方初始位姿,修正 pitch):rotation `[0.0, 1.8159, 3.14159]`,translation `[13.9965, 0.0800, 3.9803]`
- 旋转构造:`R = Rz(yaw) * Ry(pitch) * Rx(roll)`(projector.cpp:41-44)
- 投影方向:`dir_cam = normalize((u-cx)/fx, (v-cy)/fy, 1)`,`dir_world = R * dir_cam`,`origin = translation`
- 求交:Möller–Trumbore 取最近命中 t > 1e-8,与 projector.cpp:132-165 一致
- 100 帧图:`/home/yukikaze/Downloads/抽帧_100张_2/frame_00X.jpg`(5472×3648,仅 Task 2 推理需要)
- 现有 mesh:`/home/yukikaze/Documents/workspace/alliance_radar_location_lidar/model/generated/field_zup.obj`(205700 面)
- 源 PCD:`/home/yukikaze/Documents/workspace/alliance_radar_location_lidar/model/generated/jinan_field_map_reg_walls_v2.pcd`(678504 点,±14.1m × ±7.6m,z -0.2..3.48)
- 新 mesh 输出:`model/generated/jinan_field_map_reg_walls_v2.obj`,目标面数 ≤ 300k
- 所有脚本用 `REPO_ROOT=/home/yukikaze/Documents/workspace/alliance_radar_location_lidar` 下的相对路径
- 对比仅输出坐标 CSV 与 stdout 汇总,不做图像可视化

---

### Task 1: `pcd_to_obj.py` — PCD 转 OBJ 网格生成

**Files:**
- Create: `tools/pcd_to_mesh/pcd_to_obj.py`
- Test: `tools/pcd_to_mesh/test_pcd_to_obj.py`

**Interfaces:**
- Produces: CLI `pcd_to_obj.py <input.pcd> <output.obj> [--voxel 0.1] [--radii 0.25,0.5,0.75] [--max-faces 300000]`;stdout 打印点/面数、包围盒、耗时;非零退出码表示失败

- [ ] **Step 1: 写失败测试**

```python
# tools/pcd_to_mesh/test_pcd_to_obj.py
import subprocess, sys, os
REPO = "/home/yukikaze/Documents/workspace/alliance_radar_location_lidar"
def run(args):
    return subprocess.run([sys.executable, os.path.join(REPO, "tools/pcd_to_mesh/pcd_to_obj.py")] + args,
                          capture_output=True, text=True)

def test_smoke_reg_pcd():
    r = run([os.path.join(REPO, "model/generated/jinan_field_map_reg.pcd"),
             "/tmp/opencode/test_reg.obj"])
    assert r.returncode == 0, r.stderr
    assert "faces=" in r.stdout
    assert "bounds" in r.stdout

def test_missing_input_fails():
    r = run(["/tmp/opencode/does_not_exist.pcd", "/tmp/opencode/x.obj"])
    assert r.returncode != 0
```

- [ ] **Step 2: 运行确认失败**

Run: `.venv/bin/python tools/pcd_to_mesh/test_pcd_to_obj.py -v`
Expected: FAIL(FileNotFoundError,模块不存在)

- [ ] **Step 3: 实现脚本**

```python
#!/usr/bin/env python3
"""PCD -> OBJ via open3d voxel downsample + normals + Ball Pivoting."""
import argparse, sys, time
import numpy as np
import open3d as o3d

def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("input_pcd")
    p.add_argument("output_obj")
    p.add_argument("--voxel", type=float, default=0.1)
    p.add_argument("--radii", default="0.25,0.5,0.75")
    p.add_argument("--max-faces", type=int, default=300000)
    a = p.parse_args()
    t0 = time.time()
    cloud = o3d.io.read_point_cloud(a.input_pcd)
    if len(cloud.points) == 0:
        print(f"ERROR: no points in {a.input_pcd}", file=sys.stderr)
        return 1
    cloud = cloud.voxel_down_sample(a.voxel)
    cloud.estimate_normals(o3d.geometry.KDTreeSearchParamKNN(30))
    radii = [float(x) for x in a.radii.split(",")]
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        cloud, o3d.utility.DoubleVector(radii))
    if len(mesh.triangles) == 0:
        print(f"ERROR: ball pivoting produced no triangles (try smaller voxel/larger radii)",
              file=sys.stderr)
        return 1
    if len(mesh.triangles) > a.max_faces:
        print(f"WARNING: {len(mesh.triangles)} faces > {a.max_faces}; applying decimation")
        mesh = mesh.simplify_quadric_decimation(a.max_faces)
    pts = np.asarray(mesh.vertices)
    o3d.io.write_triangle_mesh(a.output_obj, mesh)
    print(f"points={len(cloud.points)} verts={len(mesh.vertices)} "
          f"faces={len(mesh.triangles)} bounds=[{pts.min(0).round(2)}]..[{pts.max(0).round(2)}] "
          f"t={time.time()-t0:.1f}s")
    return 0

if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 4: 运行确认通过**

Run: `.venv/bin/python tools/pcd_to_mesh/test_pcd_to_obj.py -v`
Expected: PASS(2 passed;生成 /tmp/opencode/test_reg.obj 且面数 > 0)

- [ ] **Step 5: 生成正式 mesh**

Run:
```bash
.venv/bin/python tools/pcd_to_mesh/pcd_to_obj.py \
  model/generated/jinan_field_map_reg_walls_v2.pcd \
  model/generated/jinan_field_map_reg_walls_v2.obj
```
Expected: 输出 verts/faces 统计,faces ≤ 300k,包围盒与 PCD 一致(±14.1 × ±7.6)

- [ ] **Step 6: 提交**

```bash
git add tools/pcd_to_mesh/
git commit -m "feat(tools): pcd_to_obj mesh generation via ball pivoting"
```

---

### Task 2: `validate_3layer.cpp` 输出检测中心坐标并重跑 100 帧

**Files:**
- Modify: `tools/armor_verify/validate_3layer.cpp`(CSV 列 + 中心坐标)
- Modify: `tools/armor_verify/CMakeLists.txt`(路径可配置)

**Interfaces:**
- Produces: `results.csv` 列 `frame,det_idx,l1_class,final_class,l1_conf,u,v`(u,v 为原图分辨率检测框中心);`validate_3layer <frames_dir> <out_dir> <model_dir>` 用法不变

- [ ] **Step 1: 改 CSV 输出**

在 validate_3layer.cpp:142-143 与 218-219 处修改:

```cpp
    std::ofstream csv(std::string(out_dir) + "/results.csv");
    csv << "frame,det_idx,l1_class,final_class,l1_conf,u,v\n";
```

```cpp
            float cx = (d.x1 + d.x2) / 2.0f;
            float cy = (d.y1 + d.y2) / 2.0f;
            csv << files[fi].filename().string() << "," << di << "," << l1_name << "," << final_name
                << "," << d.conf << "," << cx << "," << cy << "\n";
```

- [ ] **Step 2: CMakeLists 路径可配置**

把 CMakeLists.txt:4-11 改为:

```cmake
if(NOT DEFINED RADAR_SRC)
    set(RADAR_SRC /workspace/ros_ws/src/radar_camera)
endif()
if(NOT DEFINED TRT_ROOT)
    set(TRT_ROOT /opt/radar_camera_trt)
endif()
set(CUDA_ROOT ${TRT_ROOT})
```

- [ ] **Step 3: 构建**

Run:
```bash
cd tools/armor_verify/build && cmake .. \
  -DRADAR_SRC=/home/yukikaze/Documents/workspace/alliance_radar_location_lidar/ros_ws/src/radar_camera \
  -DTRT_ROOT=/home/yukikaze/radar_trt_stage && make -j$(nproc) validate_3layer
```
Expected: 编译成功,生成 `tools/armor_verify/build/validate_3layer`

- [ ] **Step 4: 重跑 100 帧推理**

Run:
```bash
mkdir -p /tmp/opencode/l123_out
tools/armor_verify/build/validate_3layer \
  "/home/yukikaze/Downloads/抽帧_100张_2" /tmp/opencode/l123_out \
  /home/yukikaze/Documents/workspace/alliance_radar_location_lidar/ros_ws/src/radar_camera/model
```
Expected: 处理 100 帧;`/tmp/opencode/l123_out/results.csv` 有约 100-300 行(每行含 u,v 列),u∈[0,5472], v∈[0,3648]

- [ ] **Step 5: 提交**

```bash
git add tools/armor_verify/validate_3layer.cpp tools/armor_verify/CMakeLists.txt
git commit -m "feat(tools): validate_3layer outputs detection center u,v"
```

---

### Task 3: `eval_ray_projection.py` — 双 mesh 射线投影对比

**Files:**
- Create: `tools/pcd_to_mesh/eval_ray_projection.py`
- Create: `tools/pcd_to_mesh/test_eval_ray_projection.py`

**Interfaces:**
- Consumes: `results.csv`(Task 2 产物,列 `frame,det_idx,l1_class,final_class,l1_conf,u,v`)、两个 mesh 路径(CLI 参数)、`--pose`(可选,默认蓝方修正初始位姿)
- Produces: `ray_compare.csv`(列 `frame,class_name,conf,u,v,map_x_old,map_y_old,hit_ok_old,map_x_new,map_y_new,hit_ok_new,delta_m`)、stdout 汇总(命中率/差异统计)。不生成可视化图。

- [ ] **Step 1: 写失败测试**

```python
# tools/pcd_to_mesh/test_eval_ray_projection.py
import numpy as np
from eval_ray_projection import R_from_rpy, pixel_to_ray, mt_intersect
import trimesh

REPO = "/home/yukikaze/Documents/workspace/alliance_radar_location_lidar"
FX, FY, CX, CY = 6753.698616, 6737.450110, 2620.748274, 1924.062270
ROT = [0.0, 1.8159, 3.14159]
TRANS = [13.9965, 0.0800, 3.9803]

def test_rpy_construction():
    R = R_from_rpy(*ROT)
    # 光轴 (+Z) 应朝下(负 z)并朝场地内(负 x, 蓝方在 +x 端)
    dz = R @ np.array([0.0, 0.0, 1.0])
    assert dz[2] < 0 and dz[0] < 0

def test_known_projection_hits_ground():
    mesh = trimesh.load(f"{REPO}/model/generated/field_zup.obj", process=False)
    tris = np.asarray(mesh.triangles).reshape(-1, 3, 3)
    origin, R = TRANS, R_from_rpy(*ROT)
    hit = mt_intersect(np.array(origin), pixel_to_ray(3836.81, 3578.17, R, FX, FY, CX, CY), tris)
    assert hit is not None
    assert abs(hit[2]) < 0.5          # 命中地面
    assert -14 <= hit[0] <= 14 and -7.5 <= hit[1] <= 7.5
```

- [ ] **Step 2: 运行确认失败**

Run: `.venv/bin/python tools/pcd_to_mesh/test_eval_ray_projection.py -v`
Expected: FAIL(ModuleNotFoundError: eval_ray_projection)

- [ ] **Step 3: 实现脚本**

```python
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
```

- [ ] **Step 4: 运行确认通过**

Run: `.venv/bin/python tools/pcd_to_mesh/test_eval_ray_projection.py -v`
Expected: PASS(2 passed;已知点命中地面 z≈0,坐标在场地范围内)

- [ ] **Step 5: 运行对比**

Run:
```bash
.venv/bin/python tools/pcd_to_mesh/eval_ray_projection.py \
  /tmp/opencode/l123_out/results.csv \
  --out model/generated/ray_compare
```
Expected: stdout 打印两个 mesh 的命中率与 delta 统计;`model/generated/ray_compare/ray_compare.csv` 生成成功(无可视化产物)

- [ ] **Step 6: 提交**

```bash
git add tools/pcd_to_mesh/
git commit -m "feat(tools): camera ray projection comparison across two meshes"
```

---

### Task 4: 结果汇总与结论

**Files:**
- Create: `docs/superpowers/sdd/2026-08-04-camera-ray-mesh-comparison-report.md`

**Interfaces:**
- Consumes: Task 3 的 `ray_compare.csv`、`ray_vis/` 可视化、stdout 汇总

- [ ] **Step 1: 统计对比**

Run:
```bash
.venv/bin/python - <<'EOF'
import csv
rows = list(csv.DictReader(open("model/generated/ray_compare/ray_compare.csv")))
old_hit = sum(r["hit_ok_old"] == "1" for r in rows); new_hit = sum(r["hit_ok_new"] == "1" for r in rows)
n = len(rows)
both = [abs(float(r["delta_m"])) for r in rows if r["hit_ok_old"] == "1" and r["hit_ok_new"] == "1"]
old_only = [r for r in rows if r["hit_ok_old"] == "1" and r["hit_ok_new"] == "0"]
new_only = [r for r in rows if r["hit_ok_old"] == "0" and r["hit_ok_new"] == "1"]
print(f"total={n} old_hit={old_hit}({old_hit/n*100:.1f}%) new_hit={new_hit}({new_hit/n*100:.1f}%)")
print(f"old_only={len(old_only)} new_only={len(new_only)}")
print(f"delta mean={sum(both)/len(both):.2f}m max={max(both):.2f}m" if both else "no both-hit")
print("old_only frames:", [(r["frame"], r["class_name"]) for r in old_only][:20])
print("new_only frames:", [(r["frame"], r["class_name"]) for r in new_only][:20])
EOF
```

- [ ] **Step 2: 坐标合理性分析**

在 `ray_compare.csv` 中抽查 delta 大、old_only、new_only 的检测:对比 map 坐标是否在场地范围(±14 × ±7.5)内、是否落在墙体位置(带墙 mesh 的命中点若在墙附近,检查是否合理);输出新旧 mesh 坐标不一致的具体案例列表,写进报告

- [ ] **Step 3: 写报告**

在 `docs/superpowers/sdd/2026-08-04-camera-ray-mesh-comparison-report.md` 记录:两个 mesh 的命中率、delta 统计、old_only/new_only 具体检测、目视结论、是否切换 `mesh_path` 的建议

- [ ] **Step 4: 提交**

```bash
git add docs/superpowers/sdd/2026-08-04-camera-ray-mesh-comparison-report.md
git commit -m "docs: camera ray mesh comparison report"
```
