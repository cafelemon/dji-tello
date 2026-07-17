# 06 进度记录

## 0.1.0

| 工作项 | 状态 |
| --- | --- |
| 文档与仓库基线 | 完成 |
| 强类型 ROS2 接口 | 完成 |
| C++ Tello Transport | 完成 |
| Vision 最新帧与参数化 | 完成 |
| FlightManager 与安全状态机 | 完成 |
| Mock、离线回放和故障注入 | 完成 |
| ROS2 Humble 构建与自动化测试 | 完成 |
| 稳定性验证 | 完成 |

更新规则：只有获得当前提交的可复现证据后，工作项才可改为“完成”。

### 当前验证结论

- 容器构建、21 项 ROS2 测试、16 项本地纯逻辑/静态检查和 Mock 核心故障注入均通过。
- A-07～A-10 已按 `docs/04_acceptance_checklist.md` 获得可复现证据并关闭。

## 0.2.0-prep

| 工作项 | 状态 |
| --- | --- |
| JetPack/TensorRT 固定基线 | 完成 |
| PyTorch/TensorRT 后端参数 | 完成 |
| ONNX 导出与校验 | 完成 |
| TensorRT 目标机构建脚本 | 完成 |
| 统一推理基准工具 | 完成 |
| systemd 安装、激活和回滚骨架 | 完成 |
| Jetson 部署与性能规范 | 完成 |

`0.2.0-prep` 仅代表预研产物完成，正式 `0.2.0` 仍按路线图推进。

## 0.2.0

| 软件工作项 | 状态 |
| --- | --- |
| `jetson.yaml` 与 Jetson Launch | 完成 |
| Transport `software/nvv4l2` 解码器选择 | 完成 |
| TensorRT FP16 输入路径修正 | 完成 |
| `/system/diagnostics` 系统监控节点 | 完成 |
| Jetson 前置检查与 release 布局校验 | 完成 |
| systemd current/previous 激活和回滚 | 完成 |

当前包版本仍保持 `0.1.0`。正式版本号只在 `0.2.0` 全部验收项关闭后统一更新。

### 当前软件验证结论

- ROS2 Humble 限制 2 核 CPU、4GB 内存完成 7 个包构建，28 项 ROS2 测试全部通过。
- 本地纯逻辑和仓库静态门禁 20 项全部通过。
- Jetson 前置检查、TensorRT Engine 命令完成 dry-run，systemd 服务文件完成目标目录布局下的静态校验。
