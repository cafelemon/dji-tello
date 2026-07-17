# 00 项目总览

## 项目目标

Tello Edge 在地面侧嵌入式 Linux 计算平台上完成 Tello 视频接收、人体检测、多目标跟踪和安全飞行控制。系统不修改 Tello 内部固件；所有飞行命令通过公开的 Tello SDK UDP 接口发送。

当前软件包版本保持 `0.1.0`，`0.2.0` Jetson 运行层正在实现；版本号将在该版本全部验收项关闭后统一更新。

## 核心原则

1. 视觉算法只能提出控制建议，飞行管理器拥有最终控制权。
2. 默认不自动起飞；起飞、降落和急停均通过显式服务触发。
3. 链路、视频、遥测或控制建议超时后输出零速度，并按状态机降级。
4. 所有路径、阈值和速率通过参数配置，不依赖用户目录。
5. 实机与 Mock 使用相同 ROS2 公共接口。

## 包结构

| 包 | 职责 |
| --- | --- |
| `tello_interfaces` | 强类型消息与服务 |
| `tello_transport_cpp` | UDP、ACK、遥测、视频、诊断和 RC 输出 |
| `tello_vision` | YOLOv5、ByteTrack、目标锁定和控制建议 |
| `tello_flight_manager` | 状态机、PID、安全策略和最终控制 |
| `tello_mock` | Tello SDK 行为模拟与故障注入 |
| `tello_bringup` | Launch 和统一 YAML 配置 |
| `tello_system_monitor` | CPU、GPU、RAM、温度、功耗与进程健康诊断 |
