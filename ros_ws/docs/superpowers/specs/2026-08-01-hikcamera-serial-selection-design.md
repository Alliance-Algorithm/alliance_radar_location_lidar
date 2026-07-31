# HIK 相机按序列号选择 — 设计文档

日期：2026-08-01

## 背景与问题

系统现在同时插了两台 HIK USB3 相机：

| 枚举索引 | 型号 | 序列号 | 用途 |
|---|---|---|---|
| 0 | MV-CS200-10UC | DB0067863 | 目标相机（`hikcamera.yaml` 按 CS200 调参） |
| 1 | MV-CS050-10UC | DB0864607 | 干扰相机（另一台） |

当前驱动 `hikcamera_sdk/src/utility.hpp:139` 的 `search_device()` 在未指定筛选时，
**只有恰好枚举到 1 台设备才连接**；现在有 2 台，`camera_connect()` 直接失败
（"2 devices were found"）。`utility.hpp` 里按 `chUserDefinedName` 匹配的 `compare()`
存在但从未被接线，且两台相机的 UserDefinedName 目前都是空。

## 目标

- 通过配置指定序列号，驱动只连接匹配的相机（目标：`DB0067863` 的 MV-CS200-10UC）。
- 不改变未配置序列号时的既有行为（恰好 1 台才连）。
- 枚举失败/无匹配时报错信息列出所有已枚举设备（型号 + SN），便于用户直接抄序列号。

## 设计

### 1. SDK：Config 增加序列号字段

`hikcamera_sdk/include/hikcamera/capturer.hpp` — `Config` 增加：

```cpp
std::string serial_number;  // 空 = 不过滤
```

### 2. SDK：按序列号匹配

`hikcamera_sdk/src/utility.hpp`：

- 新增 `compare_serial(sdk::DeviceInfo const&, std::string_view)`：USB 读
  `SpecialInfo.stUsb3VInfo.chSerialNumber`，GigE 读 `stGigEInfo.chSerialNumber`，
  与目标字符串比较。
- `search_device()` 签名改为接受可选的 `serial_number`：
  - 空 → 维持现状（1 台直连，多台报错并附设备清单）。
  - 非空 → 枚举后返回第一个 SN 匹配的设备；无匹配报错并附设备清单。

### 3. SDK：connect 接线

`hikcamera_sdk/src/capturer.impl.hpp:113` — `connect()` 调用
`util::search_device(config->serial_number)` 传入筛选。

### 4. 驱动：ROS 参数

`hikcamera_ros_driver/src/camera_bridge.cpp` — `load_config()` 声明并读取
`serial_number` 参数（默认空字符串）写入 `hik_config_.serial_number`。

### 5. 配置

`ros_ws/src/radar_bringup/config/camera/hikcamera.yaml` — 增加：

```yaml
# 按序列号选择相机（空 = 仅单台时自动连接）
serial_number: "DB0067863"
```

## 行为约定

| 场景 | 行为 |
|---|---|
| `serial_number` 非空且匹配 | 连接该设备 |
| `serial_number` 非空但无匹配 | 报错并列出所有已枚举设备（型号 + SN） |
| `serial_number` 为空且恰 1 台 | 连接（现状） |
| `serial_number` 为空且多台 | 报错并列出所有已枚举设备（改善现有错误信息） |

## 错误信息格式（stderr / RCLCPP_ERROR）

```
N devices were found:
[0] MV-CS200-10UC SN=DB0067863
[1] MV-CS050-10UC SN=DB0864607
```
无匹配时：`No device matches serial number 'xxx' among N devices`。

## 测试计划

1. 编译整个 workspace（`.script/build-radar` 或 colcon build）。
2. 硬件验证：`ros2 launch radar_bringup hikcamera.launch.py`，
   日志 `DIAG: device opened` 应打印 CS200 的 SN；相机线程 TIMING 日志正常输出。
3. 配置清空 `serial_number` 复跑，确认报错并列出两台设备清单。

## 范围外（不做）

- 按用户自定义名匹配（UserDefinedName 均为空，暂无用武之地）。
- 多相机并行采集。
