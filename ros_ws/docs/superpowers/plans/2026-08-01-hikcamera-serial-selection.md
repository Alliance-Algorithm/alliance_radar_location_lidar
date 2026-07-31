# HIK Camera Serial Selection Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让 hikcamera 驱动通过配置的序列号在两台 HIK 相机中选择目标相机（MV-CS200-10UC, SN=`DB0067863`）。

**Architecture:** 在 `hikcamera` SDK 的 `Config` 中新增 `serial_number` 字段；`util::search_device()` 支持按序列号筛选并改进错误信息（列出所有已枚举设备）；`hikcamera_ros_driver` 新增 ROS 参数 `serial_number` 透传到 SDK；`radar_bringup` 的 `hikcamera.yaml` 配置目标序列号。

**Tech Stack:** C++23, ROS 2 Jazzy, colcon, HIK MVS SDK (`MvCameraControl`), ament_cmake。

**Spec:** `ros_ws/docs/superpowers/specs/2026-08-01-hikcamera-serial-selection-design.md`

## Global Constraints

- 工作区根目录：`/home/yukikaze/Documents/workspace/alliance_radar_location_lidar`（下文中 `RADAR_WS`）。
- 构建环境：`source /opt/ros/jazzy/setup.bash`；所有 colcon 命令在 `RADAR_WS/ros_ws` 下执行。
- 目标相机序列号：`DB0067863`（MV-CS200-10UC）；另一台为 `DB0864607`（MV-CS050-10UC）。
- `serial_number` 为空时必须保持现状行为（恰好 1 台设备才连接）。
- 不得添加新依赖、不得改动与本任务无关的文件、不得引入注释以外的格式变更（跟随既有代码风格）。
- C++ 标准：C++23（SDK 与驱动 CMakeLists 已设置）。

---

### Task 1: SDK 按序列号匹配设备

**Files:**
- Modify: `ros_ws/third-party/hikcamera_sdk/include/hikcamera/capturer.hpp`（`Config` 增加字段）
- Modify: `ros_ws/third-party/hikcamera_sdk/src/utility.hpp`（`compare_serial`、`device_summary`、`device_list_message`、`search_device`）
- Modify: `ros_ws/third-party/hikcamera_sdk/src/capturer.impl.hpp`（`connect()` 接线）

**Interfaces:**
- Produces（Task 2 依赖）:
  - `struct Config { std::string serial_number; }`（空 = 不过滤）
  - `util::search_device(std::string_view serial_number = {}) noexcept -> std::expected<sdk::DeviceInfo*, std::string>`
  - `util::compare_serial(sdk::DeviceInfo const&, std::string_view) noexcept -> bool`
  - `util::device_summary(sdk::DeviceInfo const&) noexcept -> std::string`（格式 `"MV-CS200-10UC SN=DB0067863"`）
  - `util::device_list_message(sdk::DeviceInfoList const&) noexcept -> std::string`（格式 `"N devices were found:\n[0] <summary>\n[1] <summary>"`）

- [ ] **Step 1: `capturer.hpp` — `Config` 增加 `serial_number`**

在 `hikcamera_sdk/include/hikcamera/capturer.hpp` 顶部添加 `#include <string>`，并在 `Config` 结构体（当前 9-28 行，`bool fixed_framerate = true;` 之后）添加：

```cpp
    /// 按序列号选择设备；空字符串 = 不筛选（仅单台设备时自动连接）
    std::string serial_number;
```

- [ ] **Step 2: `utility.hpp` — 新增序列号匹配与设备清单辅助函数**

在 `utility.hpp` 的 `compare()` 函数（114-137 行）之后、`search_device()` 之前插入：

```cpp
constexpr auto compare_serial(sdk::DeviceInfo const& info, std::string_view serial) noexcept -> bool {
    unsigned char const* raw_serial;
    switch (info.nTLayerType) {
    case MV_GIGE_DEVICE:
        raw_serial = info.SpecialInfo.stGigEInfo.chSerialNumber;
        break;
    case MV_USB_DEVICE:
        raw_serial = info.SpecialInfo.stUsb3VInfo.chSerialNumber;
        break;
    default:
        return false;
    }
    return serial == reinterpret_cast<char const*>(raw_serial);
}

inline auto device_summary(sdk::DeviceInfo const& info) noexcept -> std::string {
    std::string_view model  = "Unknown";
    std::string_view serial;
    switch (info.nTLayerType) {
    case MV_GIGE_DEVICE: {
        const auto& gige = info.SpecialInfo.stGigEInfo;
        model  = reinterpret_cast<char const*>(gige.chModelName);
        serial = reinterpret_cast<char const*>(gige.chSerialNumber);
        break;
    }
    case MV_USB_DEVICE: {
        const auto& usb = info.SpecialInfo.stUsb3VInfo;
        model  = reinterpret_cast<char const*>(usb.chModelName);
        serial = reinterpret_cast<char const*>(usb.chSerialNumber);
        break;
    }
    default:
        return "Unknown device type";
    }
    return std::format("{} SN={}", model, serial);
}

inline auto device_list_message(sdk::DeviceInfoList const& devices) noexcept -> std::string {
    auto msg = std::format("{} devices were found:", devices.nDeviceNum);
    for (auto index = 0u; index < devices.nDeviceNum; index++) {
        if (devices.pDeviceInfo[index] == nullptr) continue;
        msg += std::format("\n[{}] {}", index, device_summary(*devices.pDeviceInfo[index]));
    }
    return msg;
}
```

- [ ] **Step 3: `utility.hpp` — `search_device()` 支持序列号筛选并改进错误信息**

将 `search_device()`（当前 139-173 行）整体替换为：

```cpp
inline auto search_device(std::string_view serial_number = { }) noexcept
    -> std::expected<sdk::DeviceInfo*, std::string> {

    auto devices = sdk::DeviceInfoList { };
    std::memset(&devices, 0, sizeof(devices));

    const auto result = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &devices);
    if (result != MV_OK) {
        const auto msg = translate_error(result);
        return std::unexpected { std::format("Failed to enum device: {}", msg) };
    }

    const auto [device_num, device_infos] = devices;
    if (device_num == 0) {
        return std::unexpected { "No device was found" };
    }

    if (!serial_number.empty()) {
        for (auto index = 0; index < device_num; index++) {
            auto* info_ptr = device_infos[index];
            if (info_ptr == nullptr) continue;
            if (compare_serial(*info_ptr, serial_number)) {
                return info_ptr;
            }
        }
        return std::unexpected { std::format(
            "No device matches serial number '{}' among {} devices:\n{}",
            serial_number, device_num, device_list_message(devices)) };
    }

    if (device_num == 1 && device_infos[0] != nullptr) {
        return device_infos[0];
    }
    return std::unexpected {
        std::format("{}\n{}", device_list_message(devices), "set serial_number to select one") };
}
```

- [ ] **Step 4: `capturer.impl.hpp` — `connect()` 传入序列号**

在 `capturer.impl.hpp` 的 `connect()` 中（当前 113 行 `auto result = util::search_device();`），改为：

```cpp
        auto result = util::search_device(config->serial_number);
```

- [ ] **Step 5: 构建 SDK 包验证编译**

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src third-party --packages-select hikcamera --symlink-install
```

Expected: 构建成功，无警告/错误。

- [ ] **Step 6: 编译硬件探测程序（只读枚举，不推流）**

创建 `/tmp/opencode/probe_serial.cpp`：

```cpp
#include <hikcamera/capturer.hpp>
#include "utility.hpp"

#include <cstdio>
#include <string_view>

int main(int argc, char** argv) {
    std::string_view serial = argc > 1 ? argv[1] : std::string_view{};
    auto result = hikcamera::util::search_device(serial);
    if (!result) {
        fprintf(stderr, "PROBE ERROR: %s\n", result.error().c_str());
        return 1;
    }
    auto const& info = **result;
    fprintf(stderr, "PROBE MATCH: %s\n", hikcamera::util::device_summary(info).c_str());
    return 0;
}
```

编译（在 `RADAR_WS` 下）：

```bash
g++ -std=c++23 -O0 -g /tmp/opencode/probe_serial.cpp \
  -I ros_ws/third-party/hikcamera_sdk/include \
  -I ros_ws/third-party/hikcamera_sdk/src \
  -L ros_ws/install/hikcamera/lib \
  -Wl,-rpath,$(pwd)/ros_ws/install/hikcamera/lib \
  -lhikcamera -lMvCameraControl \
  -o /tmp/opencode/probe_serial
```

- [ ] **Step 7: 硬件验证三个用例（相机已插着）**

```bash
/tmp/opencode/probe_serial DB0067863     # 用例 1：目标序列号 → 命中 CS200
/tmp/opencode/probe_serial DB0864607     # 用例 2：另一台序列号 → 命中 CS050
/tmp/opencode/probe_serial wrong-serial  # 用例 3：无匹配 → 报错 + 列出两台
/tmp/opencode/probe_serial               # 用例 4：空 → 报错 + 列出两台
```

Expected:
- 用例 1 stdout/stderr 含 `PROBE MATCH: MV-CS200-10UC SN=DB0067863`，exit 0
- 用例 2 含 `PROBE MATCH: MV-CS050-10UC SN=DB0864607`，exit 0
- 用例 3 stderr 含 `No device matches serial number 'wrong-serial' among 2 devices:` 且列出 `[0]`、`[1]`，exit 1
- 用例 4 stderr 含 `2 devices were found:` 且列出两台，exit 1

- [ ] **Step 8: 提交**

```bash
git add ros_ws/third-party/hikcamera_sdk/include/hikcamera/capturer.hpp \
        ros_ws/third-party/hikcamera_sdk/src/utility.hpp \
        ros_ws/third-party/hikcamera_sdk/src/capturer.impl.hpp
git commit -m "feat(hikcamera): select device by serial number"
```

---

### Task 2: 驱动 ROS 参数与配置

**Files:**
- Modify: `ros_ws/third-party/hikcamera_ros_driver/src/camera_bridge.cpp`（`load_config()`）
- Modify: `ros_ws/src/radar_bringup/config/camera/hikcamera.yaml`
- Test: 硬件运行 `hikcamera.launch.py`

**Interfaces:**
- Consumes: `Config::serial_number`（Task 1 产出）、`util::search_device(std::string_view)`（Task 1 产出）
- Produces: ROS 参数 `serial_number`（string，默认空）

- [ ] **Step 1: `camera_bridge.cpp` — 声明并读取 `serial_number` 参数**

在 `load_config()` 中，紧跟现有 `node.declare_parameter("shm_name", "/hikcamera_shm");`（28 行）之后添加：

```cpp
    node.declare_parameter("serial_number", hik_config_.serial_number);
```

在 `shm_name_ = node.get_parameter("shm_name").as_string();`（44 行）之后添加：

```cpp
    node.get_parameter("serial_number", hik_config_.serial_number);
```

- [ ] **Step 2: `hikcamera.yaml` — 配置目标序列号**

在 `ros_ws/src/radar_bringup/config/camera/hikcamera.yaml` 的 `ros__parameters:` 块内（`shm_name` 之后）添加：

```yaml
    # 按序列号选择相机（空 = 仅单台时自动连接）
    serial_number: "DB0067863"
```

- [ ] **Step 3: 构建驱动包**

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src third-party --packages-select hikcamera_ros_driver --symlink-install
```

Expected: 构建成功。

- [ ] **Step 4: 硬件正向验证（目标相机连接 + 出图）**

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
timeout 15 ros2 launch radar_bringup hikcamera.launch.py
```

Expected（日志中）:
- `DIAG: device opened:` 后跟随的 `User Defined Name`/`Serial Number` 行为 `DB0067863`
- `camera_connect succeeded`、`shm_init succeeded`、`TIMING: frame=...` 持续输出

- [ ] **Step 5: 硬件负向验证（清空序列号 → 报错并列出设备）**

临时把 `hikcamera.yaml` 中 `serial_number: "DB0067863"` 改为 `serial_number: ""`，然后：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
timeout 15 ros2 launch radar_bringup hikcamera.launch.py 2>&1 | grep -E "devices were found|MV-CS|set serial_number"
```

Expected: 输出含 `2 devices were found:`、`[0] MV-CS200-10UC SN=DB0067863`、`[1] MV-CS050-10UC SN=DB0864607`。

**改回 `serial_number: "DB0067863"` 并再次运行 Step 4 命令确认恢复。**

- [ ] **Step 6: 提交**

```bash
git add ros_ws/third-party/hikcamera_ros_driver/src/camera_bridge.cpp \
        ros_ws/src/radar_bringup/config/camera/hikcamera.yaml
git commit -m "feat(bringup): select hikcamera by serial number"
```
