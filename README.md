
# 🚁 DJI Tello ROS2 人体跟踪系统 | ROS2 Human Tracking System for DJI Tello

## 📄 项目简介 | Project Overview

该系统基于 **ROS2 + YOLOv5 + ByteTrack**，结合 Tello 无人机，实现了对人体目标的自动检测、ID 锁定、精准追踪控制，并支持高度闭环控制、手动锁定、自动恢复、以及安全避障。

This project combines **ROS2**, **YOLOv5**, and **ByteTrack** to enable automatic human detection, manual ID locking, smooth tracking, and robust PID-based drone control on the DJI Tello. It supports altitude holding, dynamic ID recovery, and safety fallback.

## 💡 功能特点 | Features

✅ YOLOv5 实时人体检测  
✅ ByteTrack 多目标 ID 分配与持续追踪  
✅ 手动输入 ID 锁定 & 动态取消  
✅ 目标丢失后自动 ID 恢复  
✅ PID 控制器平滑跟踪（前后、上下、旋转）  
✅ Tello 高度自动保持（目标 1.8 m）  
✅ ROS2 topic 架构，模块化设计

## 🗺️ 系统架构 | System Architecture

```
[ Tello Drone ]
    │
    │──> /tello/image_raw (视频流)
    │
[ tracker_node.py ]
    │──> YOLOv5 检测
    │──> ByteTrack 跟踪与 ID 分配
    │──> PID 控制输出
    │──> /tello/cmd_vel (速度指令)
    │──> /yolo/image_out (标注后图像)
    │
[ tello_cmd_vel_bridge.py ]
    │──> 接收 /tello/cmd_vel
    │──> 调用 send_rc_control 控制无人机
    │──> 高度 PID 控制
```

## ⚙️ 文件结构 | File Structure

```
tello_tracking_ws/
├── src/
│   ├── tracker_node.py
│   ├── tello_cmd_vel_bridge.py
│   └── yolov5/ (模型代码)
├── yolov5s.pt (模型权重)
├── README.md
```

## 🚀 运行步骤 | Running

### 1️⃣ 安装依赖

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-pip
pip install torch torchvision opencv-python djitellopy numpy
```

### 2️⃣ 下载 YOLOv5 权重

```bash
wget https://github.com/ultralytics/yolov5/releases/download/v6.1/yolov5s.pt
```

### 3️⃣ 启动无人机

```bash
ros2 run your_package_name tello_cmd_vel_bridge.py
```

### 4️⃣ 启动跟踪节点

```bash
ros2 run your_package_name tracker_node.py
```

### 5️⃣ 查看图像话题（可选）

```bash
rqt_image_view /yolo/image_out
```

## 🛠️ PID 调参建议 | PID Tuning Tips

- **yaw (旋转):** 当前限制 ±120，可根据需要调大或调小
- **linear.x (前后):** 用检测框面积闭环，建议小幅微调 kp
- **linear.z (上下):** 对应垂直中心误差，可微调以减小抖动

## 🔥 高级功能（已集成）| Advanced Features

- 🔄 自动中心新 ID 恢复追踪
- 🛑 丢失超时自动解锁
- 🟥 红框标记当前锁定目标

## ⚠️ 注意事项 | Notes

- 默认 CPU 推理，可配置 `torch.device('cuda')` 使用 GPU
- 建议在稳定光照、较空旷环境测试

## 📝 作者 | Author

Fei Jia (贾飞)  
Master in Embedded AI, ESIGELEC, Rouen  
🇨🇳 | 🇫🇷 | ✉️ fei.jia@groupe-esigelec.org

---

✅ 若需要自动生成 `.gitignore` 或 launch 文件，可联系我进一步完善！
