# radar_bridge 包重构 TODO

## 结构目标
```
radar_bridge/
├── CMakeLists.txt
├── package.xml
├── include/radar_bridge/
│   ├── zmq_data_format.hpp    ← 协议消息体 (常量 kPascalCase)
│   ├── bridge_node.hpp        ← BridgeNode : rclcpp::Node
│   ├── zmq_bridge.hpp         ← ZmqBridge (PUB+SUB+序列化)
│   └── udp_streamer.hpp       ← UdpStreamer (SHMRead+JPEG+UDP)
├── src/
│   ├── runtime.cpp
│   ├── bridge_node.cpp
│   ├── zmq_bridge.cpp
│   └── udp_streamer.cpp
```

## 执行顺序

- [ ] 1. 清理空目录 `zmq/` `shm/` `config/`
- [ ] 2. 重写 `zmq_data_format.hpp` — 常量改 `kPascalCase`
- [ ] 3. 新建 `bridge_node.hpp` + `bridge_node.cpp` — ROS 节点 + 回调 + 编排 ZMQ/UDP
- [ ] 4. 新建 `zmq_bridge.hpp` + `zmq_bridge.cpp` — ZMQ PUB/SUB + JSON 编解码
- [ ] 5. 新建 `udp_streamer.hpp` + `udp_streamer.cpp` — SHMRead + JPEG + UDP 推流
- [ ] 6. 重写 `runtime.cpp` — `main()` 入口
- [ ] 7. 更新 `package.xml` — 完整依赖
- [ ] 8. 更新 `CMakeLists.txt` — 链接所有依赖
- [ ] 9. 编译验证

## 涉及的 ROS 话题

### 订阅 (从雷达仓库)
| 话题 | 类型 | 来源 |
|------|------|------|
| `/lidar/pose` | `PoseWithCovarianceStamped` | radar_lidar |
| `/lidar/cluster` | `PointCloud2` | radar_lidar |

### 订阅后的去向
- `on_lidar_pose` → `ReceiveLidarLocation` 坐标填充 → mutex+latest → ZMQ PUB
- `on_cluster` → 留空（后续可提取分类目标坐标）

## 线程模型
```
ROS callbacks (10Hz, 回调驱动)
  → mutex { latest_lidar_ = ... }

timer_ (10Hz)
  → mutex { ... → zmq_->publish_lidar(latest_) }

ZmqBridge::sub_thread_ (阻塞 zmq_recv)
  → JSON 解包 → 按 cmd_id 派发回调

UdpStreamer::thread_ (阻塞 sem_timedwait)
  → SHMRead → JPEG → udp_send
```

## 依赖矩阵
| 步骤 | 依赖 | 被阻拦 |
|------|------|--------|
| 1 | — | — |
| 2 | — | 3,4,5 |
| 3 | 2 | 6 |
| 4 | 2 | 6 |
| 5 | 2 | 6 |
| 6 | 3,4,5 | 8 |
| 7 | — | 8 |
| 8 | 6,7 | 9 |
| 9 | 8 | — |
