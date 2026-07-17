# 07 决策记录

## D-001 目标平台

固定 Ubuntu 22.04、ROS2 Humble；使用容器提供一致构建环境。

## D-002 通信层语言

关键 Tello UDP Transport 使用 C++，视觉和飞行策略使用 Python；通过强类型 ROS2 接口解耦。

## D-003 控制权限

只有 FlightManager 可以发布最终 `/tello/cmd_vel`。Transport 只执行已批准控制，Vision 只发布建议。

## D-004 起飞策略

默认 `auto_takeoff=false`，所有起飞均通过显式服务且必须通过电量与链路检查。

## D-005 垂直控制优先级

默认使用遥测定高；视觉垂直跟随为互斥的可选模式。

## D-006 第三方依赖

YOLOv5 与 ByteTrack 通过 `dependencies.repos` 固定 Commit，不再使用缺少映射的 GitLink；模型由下载步骤放入忽略目录。

## D-007 文档证据边界

硬件测试方法、结果表和执行状态只进入 `08_hardware_test.md`；其他文档不重复硬件测试状态，也不使用推测数据冒充结果。

## D-008 Jetson 软件基线

固定 JetPack 6.2.1、Jetson Linux 36.4.4、Ubuntu 22.04、CUDA 12.6、TensorRT 10.3 和 ROS2 Humble。

## D-009 Jetson 部署方式

ROS2 与 TensorRT 原生部署，由 systemd 管理不可变版本目录和 current/previous 软链接；容器继续承担 CPU 构建与软件验证。

## D-010 TensorRT 产物边界

ONNX 使用静态 batch 1、640×640、opset 13；FP16 Engine 在目标 Jetson 生成且不进入 Git，平台或模型升级后重新构建。

## D-011 本机稳定性验收时长

本机软件验收采用最多 2 核 CPU、4GB 内存的 120 秒离线回放，按真实帧数、错误数和 RSS 窗口变化判定；更长时间运行进入目标平台专项测试，不用短时数据替代。
