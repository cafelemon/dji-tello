# 🛸 DJI Tello ROS2 自动跟踪系统 | ROS2-based DJI Tello Tracking System

本项目基于 **ROS2 + YOLOv5 + ByteTrack**，实现对 DJI Tello 无人机的视频识别与目标自动跟踪控制，可通过 PID 控制维持目标在画面中心，并支持飞行高度自动控制与图像回传。

This project provides an automatic object tracking system for **DJI Tello** using **ROS2, YOLOv5, and ByteTrack**. The drone tracks human targets in real-time, maintaining position using PID control, with altitude stabilization and video publishing features.

---

## ✨ 功能 | Features

- 🎯 实时人像检测（YOLOv5）
- 🧠 多目标关联跟踪（ByteTrack）
- 🚁 Tello 实时控制（djitellopy + ROS2 cmd_vel）
- 🖼 图像话题发布 `/tello/image_raw`
- 🛠 自定义 PID 控制器维持目标中心 & 高度稳定
- ⚡ 遇障识别 + 目标丢失处理（可扩展）

---

## 📦 系统要求 | Requirements

- ROS2 Foxy/Humble（建议使用 Ubuntu 20.04/22.04）
- Python 3.8+
- `djitellopy`, `torch`, `opencv-python`, `cv_bridge`, `numpy`
- 已添加 SSH 密钥的 GitHub 账户（用于拉取 YOLO 模型等）

---

## 📁 文件结构 | Project Structure
tello_tracking_ws/
├── src/
│ ├── tello_cmd_vel_bridge.py # Tello 控制器节点（含自动起飞+高度 PID）
│ ├── tracker_node.py # 图像识别+目标跟踪+控制逻辑
│ ├── tello_image_publisher.py # 备用图像发布节点
│ └── yolov5_bytetrack_ros/ # 模型和追踪器代码（YOLOv5 + ByteTrack）
├── yolov5s.pt # YOLOv5 权重文件

---

## 🚀 快速运行 | Quick Start

### ✅ 安装依赖

```bash
sudo apt update
sudo apt install python3-pip python3-colcon-common-extensions
pip install torch opencv-python djitellopy numpy

✅ 启动项目
打开无人机电源，确保与主机 Wi-Fi 连接

启动 ROS2 节点：

bash
cd tello_tracking_ws
python3 src/tello_cmd_vel_bridge.py       # 启动无人机控制与图像发布
python3 src/tracker_node.py               # 启动检测跟踪控制逻辑
可选：使用 rqt_image_view 查看图像话题 /tello/image_raw

📌 TODO & 可扩展功能
 遮挡避障逻辑（自动规避 & 重新识别）

 支持多目标切换

 引入深度相机进行 3D 避障

 UI 控制界面（Web 控制或语音指令）

📜 License
本项目源代码遵循 MIT 开源协议。
YOLOv5 模型遵循 Ultralytics 的许可证约定。

🤝 作者 | Author
Fei Jia (贾飞)
Master in Embedded AI, ESIGELEC, Rouen
🇨🇳 | 🇫🇷 | ✉️ fei.jia@groupe-esigelec.org
GitHub: cafelemon

---

### ✅ 操作建议：

保存为 `tello_tracking_ws/README.md`，然后执行：

```bash
git add README.md
git commit -m "Add bilingual README for Tello tracking system"
git push
