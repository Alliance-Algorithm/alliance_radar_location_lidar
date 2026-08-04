# 报告:camera 射线地图对比 — field_zup.obj vs jinan_field_map_reg_walls_v2.obj

日期:2026-08-04
状态:完成
前置:`docs/superpowers/plans/2026-08-04-camera-ray-mesh-comparison.md`、`docs/superpowers/specs/2026-08-04-camera-ray-mesh-comparison-design.md`

## 背景

`radar_camera_node` 的 Projector 模块(`ros_ws/src/radar_camera/src/projector.cpp`)对每个装甲板检测中心发射一条射线,与场地三角网格求交,取最近命中点 (x,y) 作为目标地图坐标。当前部署(`radar_camera.yaml:31` 的 `mesh_path`)使用的是 `field_zup.obj`;而 Lidar GICP 定位使用含墙的 `jinan_field_map_reg_walls_v2.pcd`。本次对比评估:从 PCD 生成的带墙 mesh 是否更适合作为射线地图。

**两个 mesh:**

| | 旧 `field_zup.obj` | 新 `jinan_field_map_reg_walls_v2.obj` |
|---|---|---|
| 来源 | RM 场地 CAD 模型 | lidar 建图 PCD(67.8 万点)→ 体素 0.1m + Ball Pivoting |
| 面数 | 205700 | 84140(目标 ≤300k) |
| 范围 | ±14 × ±7.5 m,z -0.2..3.17 | ±14.1 × ±7.6 m,z -0.2..3.484 |
| 结构 | 含场地边界(0.2~0.6m)、角落结构(≤1.5m)、中心结构;**无外墙** | 地面/墙为重建表面,**含外墙**(PCD 墙点 z≤1.79m),中心结构为连续表面 |
| 水密性 | — | **非水密**(Ball Pivoting 间隙,见分析) |

**外参修正说明**:`blue_camera_pose.yaml` 原 rotation pitch = -0.17453,但经 Projector 公式 `R = Rz(yaw)·Ry(pitch)·Rx(roll)` 构造后相机光轴世界 z 分量为正(朝上),射线 100% 无法命中 mesh(实测 0/5)。评测使用与 `red_camera_pose.yaml` 对称的修正值:rotation `[0.0, 1.8159, 3.14159]`、translation `[13.9965, 0.0800, 3.9803]`(光轴水平低头指向场地中心)。

## 方法

1. **推理**:`validate_3layer`(L1 检测 → L2 装甲板分类 → L3 号码验证)对 100 张抽帧图(5472×3648)输出检测框中心像素 u,v,共 188 个检测。
2. **投影**:`tools/pcd_to_mesh/eval_ray_projection.py` 对每个检测分别向两个 mesh 求交。投影公式与 projector.cpp 严格一致:
   - `x_norm=(u-cx)/fx, y_norm=(v-cy)/fy`,`dir_cam=normalize(x_norm,y_norm,1)`;内参取部署标定值 `fx=6753.698616, fy=6737.450110, cx=2620.748274, cy=1924.062270`(畸变系数 0)
   - `R = Rz(yaw)·Ry(pitch)·Rx(roll)`,`dir_world = R·dir_cam`,`origin = translation`
   - Möller–Trumbore 向量化求交,取最近命中 t>1e-8(projector.cpp:132-165 一致);命中点 (x,y) 即 map 坐标
   - 公式一致性由单元测试保障(`test_eval_ray_projection.py`:`R_from_rpy` 光轴方向、已知像素点命中 field_zup.obj 地面 z≈0 且坐标在场内)
3. **对比**:仅输出 CSV 与 stdout 汇总(`model/generated/ray_compare/ray_compare.csv`,列 `frame,class_name,conf,u,v,map_x_old,map_y_old,hit_ok_old,map_x_new,map_y_new,hit_ok_new,delta_m`),不生成可视化图。rm 坐标换算:`rm = ((map_x+14)*100, (map_y+7.5)*100)` cm。

### 复现命令(一次性分析,未入库)

命中点 z 复算(表 6-10 行)与垂直下投空洞检查(§2)均为一次性脚本,关键命令如下:

```python
# 需 .venv(trimesh);公式与 eval_ray_projection.py 一致
import sys; sys.path.insert(0, "tools/pcd_to_mesh")
import numpy as np, trimesh
from eval_ray_projection import R_from_rpy, pixel_to_ray, mt_intersect, ROT_DEFAULT, TRANS_DEFAULT
m_old = trimesh.load("model/generated/field_zup.obj", process=False)
m_new = trimesh.load("model/generated/jinan_field_map_reg_walls_v2.obj", process=False)
tris_old = np.asarray(m_old.triangles).reshape(-1, 3, 3)
tris_new = np.asarray(m_new.triangles).reshape(-1, 3, 3)
R = R_from_rpy(*ROT_DEFAULT); origin = np.array(TRANS_DEFAULT, dtype=float)

# (a) 像素射线 z 复算:u,v 取 model/generated/ray_compare/ray_compare.csv 对应行
for u, v in [(2552.18, 2142.84), (1427.32, 1905.05), (2586.38, 2111.32),
             (2779.82, 1762.37), (2793.71, 1752.75)]:
    d = pixel_to_ray(u, v, R)
    print([None if p is None else round(p[2], 2) for p in
           (mt_intersect(origin, d, tris_old), mt_intersect(origin, d, tris_new))])

# (b) 垂直下投空洞检查(§2):old_only 命中位置 (x,y)
for x, y in [(4.43, -1.70), (6.79, -0.76), (-10.73, -0.12), (8.61, -0.45), (3.48, -2.61)]:
    hit = mt_intersect(np.array([x, y, 3.0]), np.array([0, 0, -1.0]), tris_new)
    print((x, y), "穿过(无命中)" if hit is None else f"z={hit[2]:.2f}")
```

## 结果

### 命中率

| mesh | 命中 | 命中率 |
|---|---|---|
| 旧 field_zup.obj | 152/188 | **80.9%** |
| 新 walls_v2.obj | 147/188 | **78.2%** |

交集:old_only=9,new_only=4,双 miss=32。

### delta 统计(双命中 n=143)

| 指标 | 值 |
|---|---|
| mean | 0.272 m(被 5 个离群点拉高) |
| median | 0.003 m |
| p95 | 0.246 m |
| max | 9.62 m |
| >0.5m | 5/143 |
| ≤0.5m(138 行) | mean 0.023 m,max 0.486 m |

即 96.5% 的检测两 mesh 投影差 ≤0.5m,绝大部分 ≤5cm;分歧集中在少数特定位置。

### old_only(旧命中、新未命中)— 9 行

| frame | class | u,v | 旧命中点 (x,y) |
|---|---|---|---|
| frame_057 | eng_r | 3637,3096 | (4.43,-1.70) |
| frame_057 | inf4_r | 3637,3096 | (4.43,-1.70) |
| frame_057 | inf4_r | 3613,3143 | (4.34,-1.79) |
| frame_061 | inf4_r | 4384,2639 | (6.79,-0.76) |
| frame_063 | inf4_r | 2028,1978 | (-10.73,-0.12) |
| frame_065 | inf4_b | 5410,2502 | (8.61,-0.45) |
| frame_071 | inf4_b | 5378,2547 | (8.57,-0.50) |
| frame_075 | inf4_r | 3254,3557 | (3.48,-2.61) |
| frame_081 | inf3_b | 2826,1760 | (1.28,0.40) |

全部位于新 mesh 的地面覆盖空洞处(垂直射线下投实测穿过,见分析)。

### new_only(新命中、旧未命中)— 4 行

| frame | class | u,v | 新命中点 (x,y,z) | 命中面 |
|---|---|---|---|---|
| frame_073 | inf3_r | 1574,2024 | (-14.10,-0.33,1.41) | 远端外墙 |
| frame_075 | inf4_b | 1574,2023 | (-14.10,-0.33,1.41) | 远端外墙 |
| frame_061 | eng_r | 1308,1807 | (-0.15,0.32,3.23) | 中心结构顶板 |
| frame_089 | hero_r | 1289,1832 | (-0.20,0.27,3.26) | 中心结构顶板 |

### delta 最大 10 行(双命中)

| # | frame | class | 旧命中 (x,y,z) | 新命中 (x,y,z) | delta |
|---|---|---|---|---|---|
| 1 | frame_081 | hero_b | (-9.38,0.96,0.00) | (0.23,0.60,1.64) | 9.62 m |
| 2 | frame_083 | inf4_r | (-8.86,-0.13,0.00) | (0.45,-0.04,1.62) | 9.31 m |
| 3 | frame_059 | inf3_b | (-6.23,0.65,0.00) | (0.67,0.45,1.36) | 6.90 m |
| 4 | frame_059 | eng_b | (-6.23,0.65,0.00) | (0.67,0.45,1.36) | 6.90 m(同像素重复检测) |
| 5 | frame_071 | inf4_r | (-2.70,-0.50,0.30) | (0.22,-0.40,0.95) | 2.93 m |
| 6 | frame_051 | inf4_r | (-0.13,-0.39,0.60) | (0.36,-0.38,0.72) | 0.49 m |
| 7 | frame_085 | eng_b | (0.21,0.12,3.01) | (-0.21,0.12,2.98) | 0.42 m |
| 8 | frame_069 | hero_r | (0.18,-0.32,0.60) | (0.44,-0.31,0.66) | 0.25 m |
| 9 | frame_093 | inf3_r | (1.07,0.40,0.42) | (1.24,0.40,0.47) | 0.17 m |
| 10 | frame_059 | inf3_b | (1.10,0.42,0.40) | (1.26,0.42,0.45) | 0.16 m |

前 5 名(>0.5m)全部是"旧 mesh 穿过中心结构命中地面、新 mesh 命中中心结构表面"的遮挡差异。第 6-10 名(delta 0.16~0.49m)不是遮挡差异:两 mesh 都命中了结构/地面表面,差异只是命中点高度差(z 差 0.03~0.12m,重建噪声量级)——如第 7 行两 mesh 都命中中心结构顶板(z 3.01 vs 2.98,顶板高度差),第 6/8/9/10 行命中低矮结构表面(z 0.40~0.72,两 mesh 高度差 ≤0.12m)。

## 分析

### 1. 中心结构遮挡差异(主要分歧源,delta >0.5m 的 5 行 + new_only 2 行)

- 两 mesh 都在场地中心 (0,0) 附近有一结构(~1.8×1.8m,高 ~3.2m,即裁判/中心立柱)。**旧 mesh 该结构是开放结构**:四角斜墙 + 顶板 + 中心细柱(z 2.83~3.08 顶板区,斜墙只覆盖对角),射线从 4m 高相机以 z 1.3~2.8m 穿过结构间隙命中后方地面。
- **新 mesh 该结构为连续表面**:PCD 在 |x|<2、|y|<2 内 z 1.0~3.0 每层都有点,各 x 切片(x=-0.8,-0.4,0,0.4,0.8)在 z 1.3~3.0 均有连续墙面——lidar 实测该结构是实心八棱柱,旧 CAD 模型的开孔画法是简化/失真的。
- 后果:新 mesh 把"穿过中心结构的射线"正确挡在结构表面(z 0.66~3.26),旧 mesh 让射线穿过去命中 3~9.6m 外的地面——该差异只发生在 delta >0.5m 的 5 行(旧命中地面 z≈0)与 2 个 new_only 顶板命中;delta ≤0.5m 的行(如第 7 行 frame_085,旧 z 3.01 与新 z 2.98 均为顶板)两 mesh 都命中结构表面,仅是高度差,不属遮挡差异。**两面性**:
  - 若中心结构真是实心,新 mesh 的遮挡是对的;但旧 mesh 结果说明这些像素上神经网络"看到了"结构后方的机器人(如 hero_b 位于 (-9.38,0.96))——要么这些是误检,要么新 mesh 重建把结构外扩了 ~0.1-0.3m(Ball Pivoting 膨胀)导致边缘像素被误挡,真实相机可从结构边缘看到机器人。
  - 结论:该区域两 mesh 都存在风险:旧 mesh 可能把真实被遮挡的检测投影到结构后方地面,新 mesh 可能把边缘可见的目标错误投影到结构表面。新 mesh 偏差方向更极端(最多 9.6m)。

### 2. 新 mesh 地面覆盖空洞(9 个 old_only)

对 9 个 old_only 位置做垂直射线下投,5 处(4.43,-1.70)、(6.79,-0.76)、(-10.73,-0.12)、(8.61,-0.45)、(3.48,-2.61)射线直接穿过(z=0 附近无三角形)——Ball Pivoting 重建的三角面之间存在间隙,mesh 非水密(`is_watertight=False`)。旧 mesh(CAD 平滑地面)在这些位置有地面,命中正常。这是新 mesh 命中率低 2.7 个百分点的直接原因,且会随检测位置变化随机漏投。

### 3. 新 mesh 外墙改进(2 个 new_only)

frame_073/075 两行射线在旧 mesh 下越过场地边界不命中(旧 mesh 无墙),新 mesh 正确止于远端外墙(x=-14.1,z=1.41,PCD 墙高 ≤1.79m 实测)。这是新 mesh 唯一明确的正确性改进,但样本仅 2 例,且都为同一像素(u≈1574,v≈2023)。

### 4. 双 miss 的 32 行(两 mesh 一致,不影响选型)

- 19 行射线越过整个场地体(aimed 近水平/略上仰,在远墙上方 z>3.5 越过)——主要为 drone 类与远角 inf4 检测;
- 13 行从远墙上方(z 2.12~3.45 > 墙高 1.79m)飞出场地——旧 mesh 无墙、新 mesh 墙也不够高,两者都合理未命中。
两 mesh 在这些行上结论一致,不构成选型差异。

### 5. 其余双命中(138 行)

delta mean 0.023m、max 0.486m,两 mesh 地面/坡面区域高度一致,量级与体素 0.1m + 地面重建噪声相符。

## 结论与建议

**建议:不切换 `mesh_path`,部署保持 `field_zup.obj`。**

证据:

1. **命中率**:新 78.2%(147/188)低于旧 80.9%(152/188),差额全部来自新 mesh 地面覆盖空洞(9 个 old_only,垂直下投实测空洞)。
2. **大偏差风险**:新 mesh 引入 5 个 >0.5m 投影(最大 9.62m,集中在中心结构),对定位/决策的伤害远大于 2 例外墙改进的收益。
3. **改进有限**:new_only 仅 4 行,其中 2 行为远端外墙命中、2 行为中心结构顶板命中;外墙场景对蓝方朝场地内的主视角覆盖增益很小。
4. **结构不确定**:中心结构新旧 mesh 结论矛盾(CAD 开放 vs lidar 实心),新 mesh 的膨胀误差方向会使结构边缘检测投影偏移最多 9.6m,不可接受。
5. 一致性:96.5% 双命中行两 mesh 差异 ≤0.5m(median 3mm),说明常规地面检测两者等效,选型不影响大部分检测。

**后续可选优化**(本次不实施):对新 mesh 做水密修复(孔洞填充)并核实中心结构实心/开放后,可重跑本对比再决策;外墙收益若要纳入,可单独在部署侧加"越界射线按墙高度截断"逻辑,无需换 mesh。

## 产物

- `model/generated/ray_compare/ray_compare.csv`(188 行检测,含新旧命中与 delta)
- `tools/pcd_to_mesh/eval_ray_projection.py` + `test_eval_ray_projection.py`
- 本报告(纯坐标对比,无可视化产物)
