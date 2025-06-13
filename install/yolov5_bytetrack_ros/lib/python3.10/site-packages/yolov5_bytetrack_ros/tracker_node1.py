import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import torch
import sys
import numpy as np
from types import SimpleNamespace

# 临时修复 numpy 警告
np.float = float

# 加载 YOLOv5 和 ByteTrack 路径
sys.path.append('/home/jf/tello_tracking_ws/src/yolov5')
sys.path.append('/home/jf/tello_tracking_ws/src/ByteTrack')

from models.common import DetectMultiBackend
from utils.general import non_max_suppression
from utils.augmentations import letterbox
from yolox.tracker.byte_tracker import BYTETracker


def scale_coords(img1_shape, coords, img0_shape, ratio_pad=None):
    coords = coords.clone()
    if ratio_pad is None:
        gain = min(img1_shape[0] / img0_shape[0], img1_shape[1] / img0_shape[1])
        pad = ((img1_shape[1] - img0_shape[1] * gain) / 2,
               (img1_shape[0] - img0_shape[0] * gain) / 2)
    else:
        gain = ratio_pad[0][0]
        pad = ratio_pad[1]

    coords[:, [0, 2]] -= pad[0]
    coords[:, [1, 3]] -= pad[1]
    coords[:, :4] /= gain
    coords[:, :4] = coords[:, :4].clamp(0)
    return coords


class PIDController:
    def __init__(self, kp, ki, kd, setpoint=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0

    def update(self, measurement):
        error = self.setpoint - measurement
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative


class TrackerNode(Node):
    def __init__(self):
        super().__init__('tracker_node')
        self.bridge = CvBridge()

        weights = 'yolov5s.pt'
        self.img_size = 640
        self.device = torch.device('cpu')

        self.get_logger().info("🔧 加载 YOLOv5 模型中...")
        self.model = DetectMultiBackend(weights, device=self.device)
        self.model.eval()
        self.stride = int(self.model.stride)
        self.get_logger().info(f"✅ 模型加载完成，stride = {self.stride}")

        self.get_logger().info("🔧 初始化 ByteTrack 中...")
        self.tracker = BYTETracker(
            SimpleNamespace(
                track_thresh=0.5,
                track_buffer=30,
                match_thresh=0.8,
                min_box_area=10,
                mot20=False
            ),
            frame_rate=30
        )
        self.get_logger().info("✅ ByteTrack 初始化完成")

        self.pid_x = PIDController(kp=0.005, ki=0.0, kd=0.001)
        self.pid_y = PIDController(kp=0.005, ki=0.0, kd=0.001)

        self.subscription = self.create_subscription(Image, '/tello/image_raw', self.image_callback, 10)
        self.result_pub = self.create_publisher(Image, '/yolo/image_out', 10)
        self.cmd_pub = self.create_publisher(Twist, '/tello/cmd_vel', 10)

        self.get_logger().info("✅ YOLOv5 + ByteTrack + PID 控制器初始化完成")
        self.get_logger().info("📸 正在订阅 /tello/image_raw")

    def image_callback(self, msg):
        try:
            self.get_logger().info("📥 收到图像帧")
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.get_logger().info(f"📐 图像尺寸: {frame.shape}")
            img_h, img_w = frame.shape[:2]

            img = letterbox(frame, self.img_size, stride=self.stride, auto=True)[0]
            img = img[:, :, ::-1].transpose(2, 0, 1)
            img = np.ascontiguousarray(img)
            self.get_logger().info("🧮 图像预处理完成")

            img_tensor = torch.from_numpy(img).to(self.device).float() / 255.0
            if img_tensor.ndimension() == 3:
                img_tensor = img_tensor.unsqueeze(0)
            self.get_logger().info("🔄 图像转换为Tensor完成")

            with torch.no_grad():
                pred = self.model(img_tensor, augment=False, visualize=False)
            self.get_logger().info("🧠 模型推理完成")

            pred = non_max_suppression(pred, 0.25, 0.45, classes=None, agnostic=False)[0]
            self.get_logger().info(f"📦 NMS检测到 {len(pred) if pred is not None else 0} 个目标")

            dets = []
            if pred is not None and isinstance(pred, torch.Tensor) and len(pred) > 0:
                pred[:, :4] = scale_coords(img_tensor.shape[2:], pred[:, :4], frame.shape).round()
                for *xyxy, conf, cls in pred:
                    x1, y1, x2, y2 = map(int, xyxy)
                    dets.append([x1, y1, x2, y2, float(conf), int(cls)])
            self.get_logger().info(f"📊 有效检测框数量: {len(dets)}")

            if len(dets) > 0:
                dets_np = np.array(dets)[:, :5].astype(np.float32) 
                dets_tensor = torch.from_numpy(dets_np).to(self.device)  # ✅ 转为 tensor
                img_h, img_w = frame.shape[:2]
                self.get_logger().info(f"📡 调用ByteTrack更新: dets.shape={dets_np.shape}")
                online_targets = self.tracker.update(
                    output_results=dets_tensor,
                    img_info=(img_h, img_w),
                    img_size=(img_h, img_w)
                )

                self.get_logger().info(f"🧍 当前跟踪对象数量: {len(online_targets)}")

                if len(online_targets) > 0:
                    for track in online_targets:
                        tlwh = track.tlwh
                        track_id = track.track_id
                        x1, y1, w, h = map(int, tlwh)
                        x2, y2 = x1 + w, y1 + h
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                        cv2.putText(frame, f'ID: {track_id}', (x1, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                    track = online_targets[0]
                    x1, y1, w, h = map(int, track.tlwh)
                    cx, cy = x1 + w // 2, y1 + h // 2
                    center_x, center_y = img_w // 2, img_h // 2

                    error_x = cx - center_x
                    error_y = cy - center_y
                    vel_msg = Twist()
                    vel_msg.linear.y = float(self.pid_x.update(error_x))  # 左右移动
                    vel_msg.linear.z = float(self.pid_y.update(error_y))  # 上下移动
                    self.cmd_pub.publish(vel_msg)
                    self.get_logger().info(f"🎮 发布控制指令: vy={vel_msg.linear.y:.2f}, vz={vel_msg.linear.z:.2f}")
                else:
                    self.get_logger().info("🚫 当前帧未检测到任何可跟踪目标")

            result_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.result_pub.publish(result_msg)
            self.get_logger().info("📤 发布标注图像到 /yolo/image_out")

        except Exception as e:
            self.get_logger().error(f"❌ 处理图像出错: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

