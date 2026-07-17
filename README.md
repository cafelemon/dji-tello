# Tello Edge：ROS2 无人机边缘视觉跟踪与安全控制

Tello Edge 是面向 Ubuntu 22.04 与 ROS2 Humble 的 Tello 地面侧控制系统。系统使用自研 C++ UDP Transport 管理 Tello SDK 三通道，使用 YOLOv5 与 ByteTrack 完成人体跟踪，并由独立飞行状态机统一批准控制指令、处理链路异常和执行安全降落。

## 架构

```text
Tello / Mock Tello
  ├─ UDP 8889 命令与 ACK ─┐
  ├─ UDP 8890 遥测 ───────┼─ tello_transport_cpp ── /tello/*
  └─ UDP 11111 H264 ──────┘
                                  │
 /tello/image_raw ── tello_vision ── /tracking/cmd_vel
                                  │
                    tello_flight_manager ── /tello/cmd_vel
```

视觉节点只发布控制建议，只有飞行管理节点能够向 Transport 发布最终 `/tello/cmd_vel`。

## 获取依赖

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-vcstool python3-rosdep \
  ros-humble-cv-bridge ros-humble-diagnostic-msgs libopencv-dev \
  gstreamer1.0-libav gstreamer1.0-plugins-{base,good,bad}

vcs import . < dependencies.repos
python3 -m pip install -r requirements-vision.txt
mkdir -p models
wget -O models/yolov5s.pt \
  https://github.com/ultralytics/yolov5/releases/download/v6.1/yolov5s.pt
rosdep install --from-paths src --ignore-src -r -y
```

依赖仓库会进入已忽略的 `vendor/` 目录。启动前设置项目根目录，避免任何用户绝对路径：

```bash
export TELLO_EDGE_ROOT="$PWD"
```

## 构建与测试

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

也可以使用固定环境构建：

```bash
docker build -f docker/Dockerfile.humble -t tello-edge:humble .
docker run --rm -it --network host tello-edge:humble
```

视觉回放与资源受限短时稳定性验收：

```bash
docker build -f docker/Dockerfile.vision-test -t tello-edge:vision-test .
docker run --rm --cpus 2 --memory 4g tello-edge:vision-test bash -lc \
  "tools/run_offline_smoke.sh /tmp/offline-smoke && \
   tools/run_offline_soak.sh 120 /tmp/offline-short-120"
```

## Mock 快速启动

```bash
source install/setup.bash
ros2 launch tello_bringup mock.launch.py
```

另一个终端中：

```bash
ros2 service call /flight/connect std_srvs/srv/Trigger '{}'
ros2 service call /flight/takeoff std_srvs/srv/Trigger '{}'
ros2 topic echo /flight/status
ros2 service call /flight/land std_srvs/srv/Trigger '{}'
```

离线视频回放：

```bash
ros2 run tello_vision offline_video_publisher --ros-args \
  -p video_path:=/path/to/video.mp4 -p loop:=true
```

## 实机启动

连接 Tello Wi-Fi 后执行：

```bash
source install/setup.bash
ros2 launch tello_bringup real.launch.py
```

系统默认不会自动起飞。设置跟踪目标：

```bash
ros2 service call /tracking/set_target tello_interfaces/srv/SetTarget '{track_id: 1}'
ros2 service call /tracking/clear_target std_srvs/srv/Trigger '{}'
```

## 配置与诊断

统一参数位于 `src/tello_bringup/config/default.yaml`。常用接口：

- `/tello/telemetry`：Tello 遥测；
- `/tello/link_status`：控制、遥测、视频链路诊断；
- `/tracking/status`、`/tracking/cmd_vel`：跟踪结果和控制建议；
- `/tracking/diagnostics`：推理后端、FPS、耗时、帧龄、丢帧和错误计数；
- `/flight/status`：飞行状态、原因和状态时间；
- `/yolo/image_out`：标注图像。

故障排查先检查：

```bash
ros2 topic echo /tello/link_status
ros2 topic hz /tello/telemetry
ros2 topic hz /tello/image_raw
ros2 topic echo /flight/status
```

完整设计和验收资料见 [docs/00_overview.md](docs/00_overview.md)，采购盘点使用 [docs/09_hardware_checklist.md](docs/09_hardware_checklist.md)，Jetson 预研见 [docs/10_jetson_deployment.md](docs/10_jetson_deployment.md)。
