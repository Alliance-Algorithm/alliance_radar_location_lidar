# radar-lidar Todo

更新时间：2026-06-30

## 当前重点：Fusion 模块 T3

目标：让 `radar_fusion` 支持纯雷达模式与雷达 + 相机模式，并输出统一的融合结果。

- [ ] T3.1 定义相机检测消息接口
  - 创建 `radar_interfaces/msg/CameraDetection.msg`
  - 字段：`header`, `target_id`, `position` (`Point`), `size` (`Vector3`), `type`（`robot` / `dart` / `aerial`）
- [ ] T3.2 添加相机订阅到 `FusionNode`
  - 订阅 `/camera/detection` 话题
  - 接收相机检测结果
- [ ] T3.3 实现相机→地图坐标投影
  - 使用相机内参 + 外参将 2D 检测投影到 3D 地图坐标
  - 如果上游已经输出 3D 检测，则直接订阅 3D 结果
- [ ] T3.4 扩展数据关联
  - 支持雷达轨迹 ↔ 相机测量关联
  - 门限匹配 + 距离度量
- [ ] T3.5 实现融合逻辑
  - 雷达 + 相机测量加权融合
  - 更新卡尔曼状态
- [ ] T3.6 添加融合输出接口
  - 发布 `/fusion/fused_tracks`
  - 保留 `/fusion/tracks` 兼容纯雷达模式
- [ ] T3.7 添加测试
  - 单元测试：纯雷达模式
  - 单元测试：雷达 + 相机融合模式

## 完成判定

- `radar_interfaces/msg/CameraDetection.msg` 已生成并可在 `FusionNode` 中引用
- `/fusion/tracks` 保持纯雷达兼容
- `/fusion/fused_tracks` 在相机数据存在时输出融合结果
- 对应单元测试通过

---

# 离线配准重构（2026-07-01 完成）

设计文档：`docs/superpowers/specs/2026-07-01-offline-registration-redesign-design.md`

## 根因（最终定位）

初值丢了 z（4m 高度）和 pitch（俯仰）：旧 `init_pose` 写死 `z=0` 且只有 yaw，
GICP 从贴地平视的歪起点出发，陷局部最优。不是迭代次数/阈值问题。

## 已完成

- [x] 地图规范化：改用 `tools/model_to_map --y-up` 从 FBX 直接生成 Z-up 地图，
      去掉每次运行转 `/tmp` 的 hack（`map_y_up` 降级为带弃用警告的兼容回退）。
      一次性补丁工具 `normalize_map` 已删除（与 `model_to_map --y-up` 功能重叠）
- [x] look-at 6DOF 初值：由雷达位置指向注视点自动算 yaw+pitch，补回 z/pitch
- [x] yaw 局部多起点搜索（围绕 look-at 的 yaw，非 360 盲搜）
- [x] KdTree 归一化评分（内点率 + 内点 RMSE，可跨候选比较）
- [x] 精配准 + pose.json 输出（含坐标系标注、内点率、RMSE）
- [x] 合成真值验证：`make_synth_scan.py` 按 Odin1 真实规格（FOV 120°×90°、
      量程 70m、z-buffer 遮挡、2cm 噪声）合成 scan → `model/generated/synth_scan_odin.pcd`。
      配准从偏离初值恢复真值：平移误差毫米级、inlier=1.000、rmse=0.045m

## 待办

- [ ] **jinan 场外点融合（方案 A）**：把 `pcd_rmuc2026_jinan.pcd` 中场地框外
      （|x|>14 或 |y|>7.5）的真实点变换到工作系后叠进合成 scan，模拟真实场外
      墙/看台干扰，并验证扇形 ROI 能否滤掉这些杂点。需先粗对齐 jinan 在其自身
      坐标系下的场地位姿。
- [ ] 真实部署 scan 到手后，用它验证 look-at 默认初值（合成验证只证明算法机制，
      真实传感器遮挡/噪声/密度需实测）
- [ ] 红蓝方朝向：在 Foxglove 比对官方小地图，确认 180° 朝向自由度

---

# 大任务（独立排期）

## 1. LiDAR 算法重构优化（现代 C++ + 设计模式）

以现代 C++（C++23）和设计模式角度重构 `radar_lidar` 算法链，目标最优化 + 简洁。

- 范围：`localization` / `spherical_grid` / `frame_accumulator` / `dynamic_cloud` / `cluster` / `pipeline`
- 关注点：消除重复、明确职责边界、RAII/所有权清晰、零成本抽象、可测试性
- 约束：不破坏现有配准精度（用 `make_synth_scan` 合成真值回归验证）

## 2. 包间通讯节点梳理（写进 docs）

梳理各 ROS 包之间的通讯契约（话题 / 服务 / 消息类型 / QoS / 坐标系 / frame_id），
写进 `docs/`，方便后续开发对接。

- 范围：radar_lidar / radar_camera / radar_fusion / radar_bridge / radar_calibration / radar_bringup
- 产出：每个包的输入/输出话题表 + 数据流图 + 坐标系约定
