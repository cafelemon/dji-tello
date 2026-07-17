# 10 Jetson 部署预研

## 固定基线

- NVIDIA Jetson Orin Nano Super Developer Kit 8GB；
- JetPack 6.2.1、Jetson Linux 36.4.4、Ubuntu 22.04；
- CUDA 12.6、TensorRT 10.3、ROS2 Humble；
- 原生运行 ROS2 与 TensorRT，systemd 管理服务；容器用于 CPU 构建和自动测试。

## 模型路径

1. 使用 `tools/export_onnx.sh` 将 `models/yolov5s.pt` 导出为静态 batch 1、640×640、opset 13 ONNX。
2. 在目标 Jetson 上运行 `tools/build_tensorrt_engine.sh` 构建 FP16 Engine。
3. Engine 只在生成它的目标平台使用，不进入 Git；升级 JetPack、TensorRT 或模型后必须重新生成。
4. 通过 `inference_backend=tensorrt`、`device=cuda:0`、`fp16=true` 切换运行配置。

## 原生部署与回滚

```bash
tools/check_jetson_prereqs.sh
sudo deploy/install-release.sh "$PWD" 0.2.0-prep-YYYYMMDD
sudo deploy/activate-release.sh 0.2.0-prep-YYYYMMDD
systemctl status tello-edge.service
```

固定目录：

- `/opt/tello-edge/releases/<release-id>`：不可变版本目录；
- `/opt/tello-edge/current`：当前版本软链接；
- `/opt/tello-edge/previous`：上一个版本软链接；
- `/etc/tello-edge/tello-edge.env`：设备本地环境配置。

回滚执行 `sudo deploy/rollback-release.sh`。服务使用专用 `tello` 用户、`Restart=on-failure`、SIGINT 和 20 秒退出窗口，使节点有机会发送零速度和安全降落命令。
