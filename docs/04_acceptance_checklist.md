# 04 验收测试清单

| ID | 验收项 | 证据 | 状态 |
| --- | --- | --- | --- |
| A-01 | 清洁工作区可完成 ROS2 Humble 构建 | `docker build -f docker/Dockerfile.humble -t tello-edge:humble .` | 通过 |
| A-02 | 单元与集成测试通过 | `docker run --rm tello-edge:humble bash -lc "source /opt/ros/humble/setup.bash && source install/setup.bash && colcon test && colcon test-result --verbose"` | 通过 |
| A-03 | 不存在用户绝对路径或失效入口 | `tools/verify_repo.sh` | 通过 |
| A-04 | 一条 Launch 启动 Mock 核心节点 | `tools/run_mock_smoke.sh /tmp/mock-smoke` | 通过 |
| A-05 | 低电量拒绝起飞，重复起飞不重复执行 | `colcon test`、Mock smoke | 通过 |
| A-06 | ACK 超时、错误 ACK 和丢包触发重试/失败 | `tools/run_mock_smoke.sh /tmp/mock-smoke` | 通过 |
| A-07 | 遥测、视频、控制建议超时输出零速度 | `tools/run_flight_fault_smoke.sh` | 通过 |
| A-08 | 目标恢复受门限约束，超时进入降落 | `test_reacquire.py`、状态机测试 | 通过 |
| A-09 | 离线视频可产生图像、状态和控制建议 | `tools/run_offline_smoke.sh` | 通过 |
| A-10 | 资源受限短时回放无崩溃和持续内存增长 | `artifacts/offline-short-120-20260717/summary.json` | 通过 |

状态只能使用：通过、失败、受阻、待执行。所有“通过”必须附当前版本的可复现命令或产物。

## 2026-07-15 验证记录

- ROS2 Humble 容器内完成 6 个包的构建。
- `colcon test-result --verbose` 汇总 15 个测试，0 错误、0 失败、0 跳过。
- 本地纯逻辑和仓库静态检查汇总 10 个测试通过。
- Mock smoke 覆盖 ACK 超时、错误 ACK、恢复重连、重复起飞拒绝、遥测中断和自动降落。
- 稳定性脚本完成 60 秒试跑。

## 2026-07-17 收尾验证记录

- A-07 四个独立故障场景通过：控制建议过期、视频中断、遥测中断和急停；安全降落前连续发布零速度。
- A-08 受限重识别覆盖时间、中心距离、面积比例、多候选和不匹配目标拒绝。
- A-09 离线回放产生标注图像、可见目标状态、非零控制建议，并观察到最新帧策略丢弃中间帧。
- A-10 在最多 2 核 CPU、4GB 内存条件下运行 120 秒：311 帧图像、340 次可见状态、0 错误；RSS 基准中位数 839804KB，末段中位数 839880KB，末段正增长步比例 0.333，通过短时稳定性门限。
