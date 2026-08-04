# 设计:对比 camera 射线地图 — field_zup.obj vs walls_v2 生成 mesh

日期:2026-08-04
状态:已确认

## 背景

`radar_camera_node` 的 Projector 模块(`ros_ws/src/radar_camera/src/projector.cpp`)对每个装甲板检测中心发射一条射线,与场地三角网格做 ray-triangle 求交,取最近命中点的 (x,y) 作为目标地图坐标。

当前部署使用的射线地图 mesh(即 `mesh_path`):

- `ros_ws/src/radar_bringup/config/camera/radar_camera.yaml:31` → `/workspace/model/generated/field_zup.obj`
- RM 场地网格,z-up,±14m × ±7.5m,617k verts / 205k faces,**不含墙体**

Lidar GICP 定位使用的地图是 `model/generated/jinan_field_map_reg_walls_v2.pcd`(67.8 万点,含墙体),与相机射线 mesh 不是同一个东西。

目标:从 `jinan_field_map_reg_walls_v2.pcd` 生成一个新的带墙 mesh,离线对比新旧两个 mesh 在 100 张抽帧图上的射线投影效果,判断哪个更好。

## 组件

### 1. `tools/pcd_to_mesh/pcd_to_obj.py`(新增)— PCD → OBJ

- 输入:`model/generated/jinan_field_map_reg_walls_v2.pcd`
- 流程(open3d):
  1. 读取点云(xyz)
  2. 体素下采样,体素边长 0.1m(参数化 `--voxel`,默认 0.1)
  3. 估计法线(knn=30)
  4. Ball Pivoting 重建,半径序列 `[0.25, 0.5, 0.75]`(参数化 `--radii`)
  5. 过滤孤立三角形(面积异常小/极大)
  6. 导出 OBJ:输出 `model/generated/jinan_field_map_reg_walls_v2.obj`
- 面数目标 ≤ 300k(Projector 求交是 O(N) 暴力遍历,当前 205k 面,需控制规模)
- 输出统计信息:点/面数、包围盒、处理耗时

### 2. `tools/armor_verify/validate_3layer.cpp`(小改动)

- 现有工具已实现 L1(1280x1280)+ L2(L2 armor)+ L3(number)完整流程,输出 `results.csv`
- 改动:CSV 增加 `u,v` 两列 —— 检测 bbox 中心在**原始分辨率**(5472×3648)下的像素坐标,`u=(x1+x2)/2, v=(y1+y2)/2`
- 重新编译后对 100 张帧(`/home/yukikaze/Downloads/抽帧_100张_2/`)运行
- 输出:`<out_dir>/results.csv`,列:`frame,det_idx,l1_class,final_class,l1_conf,u,v`

### 3. `tools/pcd_to_mesh/eval_ray_projection.py`(新增)— 投影对比

- 输入:
  - 检测结果 CSV(步骤 2 产物)
  - 两个 mesh:`field_zup.obj`(现有)与 `jinan_field_map_reg_walls_v2.obj`(步骤 1 产物)
  - 相机位姿:`ros_ws/src/radar_bringup/config/camera/blue_camera_pose.yaml`(蓝方初始位姿,rotation `[0.0, -0.17453, 3.14159]`,translation `[13.9965, 0.0800, 3.9803]`),`--pose` 参数可覆盖
- 投影公式与 `Projector::proj_pixel_to_ray`(projector.cpp:106-130)严格一致:
  1. `cv::undistortPoints(pixel, K=I, dist=0, noArray, noArray)` → 归一化坐标(单位阵下即 `(u, v)`)
  2. `dir_cam = normalize(x_norm, y_norm, 1.0)`
  3. `dir_world = R_map_camera * dir_cam`,`origin = t_map_camera`(注意:此步与 C++ 一致,不额外取负)
  4. 与 mesh 求交(trimesh `ray.intersects_location`,取最近命中),`map_x, map_y = hit.x, hit.y`
- 求交失败的检测记为 `hit_ok=0`
- 输出:
  - `ray_compare.csv`:每个检测一行 —— `frame,class_name,conf,u,v,map_x_old,map_y_old,map_x_new,map_y_new,hit_ok_old,hit_ok_new,delta_m`
  - 每帧可视化图到 `ray_vis/`:两个 mesh 的投影点分别以蓝/绿圆点画在帧上(原点 = 旧 mesh,绿点 = 新 mesh),差值 > 阈值(默认 0.5m)时标红并连线
- 汇总指标打印:hit_ok 命中率(新旧)、投影点差异分布(均值/p95/最大)

## 评估方式

1. **命中率**:hit_ok 比例,新旧 mesh 对比(带墙 mesh 射线可能提前命中墙,命中率可能更高或打错面)
2. **目视检查**:可视化图上投影点是否落在正确的机器人位置上(蓝方初始位姿下,场地坐标 ±14m × ±7.5m,rm 坐标 = (map_x+14)*100, (map_y+7.5)*100 cm)
3. **稳定性**:同目标跨帧投影抖动
4. 结论:旧 mesh(field_zup.obj)投影点合理且命中率高 → 保持不变;新 mesh 更准 → 更新部署配置 `mesh_path`

## 边界与错误处理

- PCD 读取失败 / mesh 生成失败 → 脚本报错退出,给出可读信息
- 检测 CSV 无有效行 → 提示无检测,不生成对比结果
- 某帧图不存在 → 跳过该帧并提示
- 相机位姿文件缺失 → 使用默认值(蓝方初始位姿)并警告
- Ball Pivoting 产物面数为 0(重建失败)→ 提示降低体素/调整半径
- 100 帧推理耗时较长(TensorRT GPU,预计几分钟),脚本打印进度

## 测试

- `pcd_to_obj.py`:先用 `jinan_field_map_reg.pcd`(无墙)快速冒烟,确认输出 OBJ 面数 > 0、包围盒合理;再跑 walls_v2
- `validate_3layer.cpp`:复用现有 tools/armor_verify 的构建与测试方式;改动仅 CSV 列,不影响推理路径
- `eval_ray_projection.py`:用旧 mesh 跑一遍,与既有 `model/generated/camera_ray_per_frame_blue.csv`(同一 mesh 的历史投影结果)抽查对比数值量级一致(u,v 分辨率、map 坐标范围),确认投影公式正确后再跑新 mesh

## 不做的事(YAGNI)

- 不修改线上 `radar_camera_node` 任何代码
- 不做实时推理对比,不做耗时基准
- 不生成除了 OBJ 之外的其他 mesh 格式
- 不自动切换部署配置,结论由人工判断后手动改 `mesh_path`
