# 03 技术架构

## 数据流

```text
UDP 8890 ──> TelloTelemetry ─────────────────────┐
UDP 11111 ─> Image ─> Vision ─> TrackingStatus ─┼─> FlightManager
UDP 8889 <─ Transport <──── approved Twist ──────┘       │
     ACK/Retry/Rate limit                     FlightStatus
Jetson sysfs/procfs ─> SystemMonitor ─> /system/diagnostics
```

Transport 是唯一访问 Tello UDP 的组件，FlightManager 是唯一发布最终 `/tello/cmd_vel` 的组件。Vision 只发布带时间戳的 `/tracking/cmd_vel`。

## 状态机

```text
DISCONNECTED -> CONNECTED -> READY -> TAKING_OFF -> TRACKING
                                  \                  |
                                   \-> ERROR <- LOST_TARGET
                                         |
                                      LANDING -> LANDED
```

READY 要求控制、遥测和视频链路健康。TRACKING 中的状态、视频或控制建议过期立即产生零速度；目标短时恢复后返回 TRACKING，超过阈值进入 LANDING。

## 垂直控制

默认 `altitude_hold`：FlightManager 根据遥测高度生成 `linear.z`，忽略视觉建议中的垂直分量。`visual_follow` 模式才使用 Vision 的 `linear.z`。两个控制器不会同时生效。

## 故障与恢复

- SDK 命令串行发送，ACK 超时后按配置重试。
- RC 命令无 ACK，按固定频率发送；过期命令替换为零速度。
- 控制链路断开后 Transport 周期性重新进入 SDK command 模式。
- ERROR、LANDING 和 LANDED 状态禁止运动控制。
- Transport 析构时先发送多次零 RC；记录为空中状态时再尝试降落。

## Jetson 运行层

- 默认配置使用 `video_decoder=software`；Jetson 覆盖配置使用 `video_decoder=nvv4l2`。
- 两种解码管线都保持 appsink 最新帧策略。硬件解码初始化失败会进入视频链路错误，不自动回退，从而避免配置与实际运行状态不一致。
- `tello_system_monitor` 从 `/proc`、Jetson sysfs 和 `nvpmodel` 发布 `/system/diagnostics`。不可读取的设备指标明确标记为 `unavailable` 或 `unknown`。
- `jetson.launch.py` 仅在默认配置之上叠加 Jetson 参数，不改变控制所有权、状态机或兼容话题语义。
