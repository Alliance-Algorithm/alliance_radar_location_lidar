# 离线配准改造设计文档 — 稳定收敛全局最优 + 坐标系规范化

日期: 2026-07-01
状态: 待评审
作者:  @yukikaze223344

---

## 1. 背景与问题

`radar_lidar` 包内有离线标定/配准工具,用于把单帧 LiDAR 扫描配准到场地地图,
反解出雷达→地图的高精度外参 `T_map_lidar`,为后续相机-雷达联合标定与裁判系统
位置上报提供基础。

当前调用方式(`offline-test` 脚本 → `offline_test_node`):

```bash
offline-registration model/pcd_rmuc2026_jinan.pcd --map model/generated/map.pcd \
  -p map_y_up:=true -p scan_voxel:=0.5 -p voxel_leaf:=0.15 \
  -p max_iterations:=200 -p max_corr_distance:=30.0 -p coarse_to_fine:=false
```

**症状**: 配准不准,陷入局部最优解。

### 1.1 根因(已通过代码走查 + 数据实测确认)

1. **初值自由度丢失(核心)**。`offline_test_node.cpp:215` 构造初值时把
   `z` 写死为 `0.0`,旋转只构造 `yaw`、无 `pitch`。而雷达实际架在短边中心、
   约 4m 高、低头约 14° 指向场地中心。实测:雷达位于工作系 `(-14, 0, 4)`、
   注视场地中心 `(0, 0, 0.5)` 时,`pitch ≈ 14.04°` 低头。当前代码在高度(4m)
   与俯仰(14°)上完全偏离真值,GICP 作为局部优化器从这个歪起点出发必进错误盆地。
   **这不是迭代次数问题**——`max_iterations:=200` 只会让它更深地陷入错误盆地。

2. **命令未传初值 + 跳过粗配准兜底**。示例命令没有传 `initial_x/y/yaw_deg`,
   `init_pose` 停在 `Identity`;`coarse_to_fine:=false` 又跳过粗配准兜底,
   等于让 GICP 从原点裸奔。

3. **评分指标不可比**。`localization.cpp:125` 用 `small_gicp` 的 `result.error`
   (加权残差和,与对应点数量耦合)作为 fitness。它非归一化,跨不同起点比较大小
   会选错。要做多起点搜索,必须换成基于地图 KdTree 的归一化评分(内点率 + RMSE)。

4. **地图坐标系靠运行时 hack**。`offline_test_node.cpp:90-105` 在 `map_y_up:=true`
   时每次运行都把地图绕 X 轴 +90° 旋转、写入 `/tmp/map_z_up.pcd`。慢、依赖 `/tmp`、
   忘加参数就用歪地图配准。

### 1.2 参考实现对比(T-DT_Radar)

参考 [T-DT_Radar](https://github.com/T-DT-Algorithm-2025/T-DT_Radar) 用 PCL GICP、
**不给初值**(identity),靠预处理(累积 20 帧 → 0.1° 方位/俯仰网格取每格最远点
→ 场地裁剪 → 0.1m 体素)使扫描接近地图,单次 GICP,`fitnessScore < 0.2` 判收敛。

**为什么不能照抄**: T-DT 的 Livox 安装方向本身就大致对齐场地系,`T_map_lidar`
接近单位阵,所以 identity 起手可行。而我方 Odin 带 180°/14°/4m 的大刚体偏移,
identity 起手必失败。**正确解法相反:把已知物理先验完整喂进去**,而非丢弃初值。

---

## 2. 坐标系规范(地基,必须一步到位)

### 2.1 官方上报坐标系(2026 通信协议 V1.3.0,权威)

2026 赛季协议重命名为《RoboMaster 2026 机甲大师高校系列赛通信协议》,
版本从 **V1.1.0** 起,最新 **V1.3.0 (20260327)**。**不存在 V2.0.0**
(发版前需再核对官方最新版本号)。

雷达位置上报 `0x0305 map_robot_data_t`:
- **原点**: 小地图**左下角**
- **+X**: 水平向右(沿 28m 长边)
- **+Y**: 竖直向上(沿 15m 短边)
- **单位**: **厘米 (cm)**,`uint16_t`,X∈[0,2800]、Y∈[0,1500]
- **纯 2D**: 无 Z(高度)字段
- **(0,0)**: 特殊值 = "不发送该机器人位置",不可用作有效坐标
- **V1.3.0 结构体 48 字节**: 含敌方 6 个(hero/engineer/infantry3/4/aerial/sentry)
  + 全部 7 个友方位置

### 2.2 本阶段工作坐标系(配准 + 标定用)

**决策(用户确认)**: 本阶段用**中心原点工作系**,发裁判系统时再单独转官方系。
理由: 地图当前就是中心原点,避免本阶段折腾平移/朝向;中心↔官方仅差已知平移,随时可转。

工作系定义:
- **原点**: 场地正中心地面投影
- **x**: 长边 ∈ [-14, 14]
- **y**: 短边 ∈ [-7.5, 7.5]
- **z**: 高度,+z 向上,z=0 = 比赛地面(地面在板面,不强行归零,保留物理高度)
- **单位**: 米

### 2.3 场地朝向(红蓝,用户确认)

- **红方在左**(工作系 x = -14 短边),**蓝方在右**(x = +14)。
- 官方原点(左下角、红方近角)= 工作系左端 x=-14。
- 官方 +X(向右)= 工作系 +x 方向(红→蓝),**同向,无需转 180°**。
- 短边 +Y 指向哪一侧的最后 180° 自由度,留待 Foxglove 比对官方小地图确认一次。

### 2.4 工作系 → 官方上报系转换(本阶段不实现,公式备查)

```
X_official_cm = (x_work + 14.0) * 100      # 红(左,-14)→0cm, 蓝(右,+14)→2800cm
Y_official_cm = (y_short_work + 7.5) * 100  # 短边 [-7.5,7.5] → [0,1500]cm
```
抽样验证(已通过): 红角 (-14,-7.5)m→(0,0)cm;蓝角 (+14,+7.5)m→(2800,1500)cm;
中心 (0,0)m→(1400,750)cm。发送时 (0,0) 需避开(特殊值),Z 不上报。

---

## 3. 地图规范化(一次性生成规范 Z-up 地图)

### 3.1 地图诊断(实测 `model/generated/map.pcd`,FBX 手动转)

- 2449886 点,字段含 normal + rgb。**Y-up**(高度在 y ∈ [-0.2, 3.175])。
- x ∈ [-14,14](长边 28m),z ∈ [-7.5,7.5](短边 15m),中心精确居中。
- 地面平面拟合: 残余倾斜 **0.0001°**(无残余 roll/pitch),平整度 RMSE 1.5cm。
  → **几何非常干净,无需姿态矫正**。
- 地面在 y ≈ -0.198m: 是**场地板物理厚度**,非误差,**不做归零**。
- 全场统一材质色(蓝 82% 为地垫通用色),**红蓝完全对称,点云自身无法定朝向**
  → 朝向必须由外部约定(见 2.3)提供。

### 3.2 规范化变换(纯刚体,已验证右手系不镜像)

仅需 **Y-up → Z-up 轴变换**(高度从 y 挪到 z),不平移、不转 180°、不缩放:

```
x_work = x_map          # 长边不变
y_work = -z_map         # 短边(绕 X 轴 +90°)
z_work =  y_map         # 高度: Y-up → Z-up
```

验证(已通过): 变换矩阵行列式 = 1.000000(右手系✓)、`R·Rᵀ=I`(正交✓);
变换后 x∈[-14,14]、y∈[-7.5,7.5]、z∈[-0.2,3.175]。

### 3.3 产物

- 一次性离线脚本/工具读取原始 `map.pcd`,应用 3.2 变换,输出规范
  **Z-up 地图**(建议 `model/generated/map_zup.pcd`),保留原始高度(不归零)。
- 配准工具默认加载规范地图,**移除运行时 `map_y_up` 旋转 + `/tmp/map_z_up.pcd` hack**。
- 保留 `map_y_up` 参数为兼容开关(默认 false),但文档标注推荐用规范地图。

> 待实现时用 open3d 或等价工具做转换(注: 容器内 `.venv` 的 python 软链指向宿主机路径,
> open3d pybind 不可用;规范化工具应用 C++/PCL 实现,或用系统 python + 手写 PCD 二进制读写)。

---

## 4. 配准工具改造(方案 A)

目标: 从固定粗略物理先验出发,用满约 2 分钟预算,稳定收敛到全局最优,
输出高精度 `T_map_lidar` + 可信评分。

改造点集中在 `offline_test_node.cpp`,`LocalizationStage::process` 复用不改。

### 4.1 Look-at 完整 6DOF 初值(核心修复)

输入雷达位置 `(init_x, init_y, init_z)` + 注视点(默认场地中心,可配
`look_at_x/y/z`),用 look-at 几何直接算出完整 6DOF 初值:

- 方向向量 `d = normalize(target - eye)`
- `yaw = atan2(d.y, d.x)`(绕 +Z)
- `pitch = atan2(-d.z, hypot(d.x, d.y))`(绕 +Y,右手系;雷达在高处俯视 → `d.z<0` → `pitch>0`。
  实测: eye=(-14,0,4)、target=(0,0,0.5) → yaw=0°、pitch=+14.04°,即 pitch>0 表示低头)
- `roll = 0`(默认)
- 平移 = `(init_x, init_y, init_z)`,**修复 z 写死 0 的 bug**,补全 pitch。

组装 `init_pose = Translation(eye) * Rz(yaw) * Ry(pitch) * Rx(roll)`
(内旋顺序 Z→Y→X 固定,实现中注释;等价 `AngleAxis(yaw,Z) * AngleAxis(pitch,Y) * AngleAxis(roll,X)`)。

### 4.2 局部 yaw 多起点搜索

角度只有大概,故围绕 look-at 算出的 yaw 做**局部网格搜索**(非 360° 盲搜):
- 参数: `yaw_search_range_deg`(默认 ±30)、`yaw_search_step_deg`(默认 10)。
- 每个 yaw 候选 → 从对应初值跑一次**快速粗配准**(大 voxel、大 max_corr、少迭代)。
- 多线程并行(复用 `num_threads`),用满约 2 分钟预算。
- 可选把 pitch 也纳入搜索(留参数 `pitch_search_range_deg`,默认 0=不搜)。

### 4.3 归一化评分(替换不可比的 `result.error`)

用地图已有的 `map->pcl_tree()`(`pcl::KdTreeFLANN`)对每个候选打分:
- 把候选 `T` 变换后的 source 点查最近邻;
- **内点率** = 距离 < `inlier_threshold`(默认 ~0.3m,可配)的点占比;
- **内点 RMSE** = 内点距离的均方根;
- 排序: 内点率优先,RMSE 次之。取 top-1(或 top-K 再精配)。

### 4.4 精配准

对最优候选,从其位姿出发跑**精配准**(小 `max_corr`、小 voxel、迭代到收敛),
输出最终 `T_map_lidar`。保留现有由粗到精思路,但起点来自 4.2/4.3 的最优候选而非盲选。

### 4.5 保留现有能力

- 扇形 ROI 裁剪(Odin 扇形视场)与 source 降采样保持不变。
- `/offline/*` 可视化话题(map/scan_raw/scan_roi/scan_aligned/overlay/pose/diagnostics)照旧发布,
  供 Foxglove 比对(也用于 2.3 的朝向确认)。

---

## 5. 输出

- `pose.json` 增加: `T_map_lidar`(4x4 或 平移+四元数)、`inlier_ratio`、`rmse`、
  候选分数表(每个 yaw 候选的 inlier_ratio/rmse,便于调试)、`converged`。
- 明确标注坐标系: 输出外参在**工作系(中心原点、米、Z-up)**下,
  方向为 `T_map_lidar`(雷达点 → 地图),并附 2.4 的官方转换公式备注。
- `/offline/pose` 话题照旧,covariance 保留。

---

## 6. 改动范围

| 文件 | 改动 |
|---|---|
| 新增: 地图规范化工具 (C++/PCL 或脚本) | 一次性 Y-up→Z-up 生成 `map_zup.pcd` |
| `tools/offline_test_node.cpp` | look-at 6DOF 初值(修 z/pitch bug)+ yaw 搜索循环 + KdTree 评分 + 精配准;移除 `/tmp` 旋转 hack,默认加载规范地图 |
| `include/radar_lidar/config.hpp` | 新增参数(见下) |
| `config/offline_registration.yaml` | 新增参数默认值 |
| `src/localization.cpp` / `localization.hpp` | 不改(复用 `process` / `set_initial_pose`) |
| `.script/offline-test` / launch | 按需更新默认地图路径 |

新增参数(工作系,米/度):
`initial_z`、`look_at_x/y/z`、`yaw_search_range_deg`、`yaw_search_step_deg`、
`pitch_search_range_deg`(默认 0)、`inlier_threshold`;`initial_pitch_deg` 作为可选覆盖。

---

## 7. 验收标准

1. 从默认物理先验(雷达短边中心/4m/指向中心)出发,**不再陷入错误盆地**,
   `inlier_ratio` 高、`rmse` 低(具体阈值实现时用真实 scan 标定,参考 T-DT fitness<0.2 量级)。
2. `T_map_lidar` 在 Foxglove overlay 中扫描与地图目视贴合;
   官方小地图比对确认红蓝朝向正确。
3. 规范地图为固定文件,工具默认加载,无运行时 `/tmp` 依赖。
4. 搜索在约 2 分钟预算内完成(多线程)。
5. `lsp_diagnostics` 干净,构建通过。

## 8. 非目标(本阶段不做)

- 相机-雷达联合标定(仅产出雷达侧外参基础)。
- 裁判系统 0x0305 发送端实现(仅在 spec 记录转换公式与结构)。
- 地图原点迁移到官方角点(推迟到发送阶段)。
- 全局特征配准(FPFH/RANSAC)、手动拖拽对齐(已评估,不采用)。
