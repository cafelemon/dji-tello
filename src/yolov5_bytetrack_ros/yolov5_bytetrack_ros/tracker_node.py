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
import threading

np.float = float

sys.path.append('/home/jf/tello_tracking_ws/src/yolov5')
sys.path.append('/home/jf/tello_tracking_ws/src/ByteTrack')

from models.common import DetectMultiBackend
from utils.general import non_max_suppression
from utils.augmentations import letterbox
from yolox.tracker.byte_tracker import BYTETracker

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

class TrackerNode(Node):
    def __init__(self):
        super().__init__('tracker_node')
        self.bridge = CvBridge()

        weights = 'yolov5s.pt'
        self.img_size = 640
        self.device = torch.device('cpu')
        self.model = DetectMultiBackend(weights, device=self.device)
        self.model.eval()
        self.stride = int(self.model.stride)

        self.tracker = BYTETracker(
            SimpleNamespace(
                track_thresh=0.3,
                track_buffer=30,
                match_thresh=0.6,
                min_box_area=10,
                mot20=False
            ),
            frame_rate=30
        )

        self.pid_x = PIDController(kp=0.2, ki=0.0, kd=0.005)
        self.pid_y = PIDController(kp=0.2, ki=0.0, kd=0.005)
        self.pid_dist = PIDController(kp=0.001, ki=0.0, kd=0.0001)
        self.desired_area = None

        self.target_id = None
        self.locked = False
        self.lost_counter = 0

        self.cmd_pub = self.create_publisher(Twist, '/tello/cmd_vel', 10)
        self.sub = self.create_subscription(Image, '/tello/image_raw', self.image_callback, 10)
        self.result_pub = self.create_publisher(Image, '/yolo/image_out', 10)

        threading.Thread(target=self.user_input_loop, daemon=True).start()

    def user_input_loop(self):
        while True:
            user_input = input("🔧 输入目标 ID 进行锁定（输入 'c' 取消锁定）：")
            if user_input.strip().lower() == 'c':
                self.locked = False
                self.target_id = None
                self.get_logger().info("🔓 已取消锁定")
            else:
                try:
                    self.target_id = int(user_input.strip())
                    self.locked = True
                    self.get_logger().info(f"🔐 手动锁定目标 ID = {self.target_id}")
                except:
                    self.get_logger().warn("⚠️ 无效输入，请输入数字或 'c'")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            img_h, img_w = frame.shape[:2]

            img = letterbox(frame, self.img_size, stride=self.stride, auto=True)[0]
            img = img[:, :, ::-1].transpose(2, 0, 1)
            img = np.ascontiguousarray(img)
            img_tensor = torch.from_numpy(img).to(self.device).float() / 255.0
            if img_tensor.ndimension() == 3:
                img_tensor = img_tensor.unsqueeze(0)

            with torch.no_grad():
                pred_raw = self.model(img_tensor, augment=False, visualize=False)
            pred = non_max_suppression(pred_raw, 0.1, 0.4, classes=[0])[0]

            dets = []
            if pred is not None and len(pred) > 0:
                pred[:, :4] = scale_coords(img_tensor.shape[2:], pred[:, :4], frame.shape).round()
                for *xyxy, conf, cls in pred:
                    x1, y1, x2, y2 = map(int, xyxy)
                    dets.append([x1, y1, x2, y2, float(conf), int(cls)])

            dets_np = np.array(dets)[:, :5].astype(np.float32) if dets else np.empty((0, 5), dtype=np.float32)
            online_targets = self.tracker.update(torch.from_numpy(dets_np), (img_h, img_w), (img_h, img_w))

            twist_msg = Twist()
            target_track = None

            for track in online_targets:
                x1, y1, w, h = map(int, track.tlwh)
                color = (0, 255, 0)
                if self.locked and track.track_id == self.target_id:
                    target_track = track
                    color = (0, 0, 255)
                cv2.rectangle(frame, (x1, y1), (x1 + w, y1 + h), color, 2)
                cv2.putText(frame, f'ID:{track.track_id}', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            if target_track:
                self.lost_counter = 0
                x1, y1, w, h = map(int, target_track.tlwh)
                cx, cy = x1 + w // 2, y1 + h // 2
                center_x, center_y = img_w // 2, img_h // 2
                error_x = cx - center_x
                error_y = cy - center_y

                yaw = float(self.pid_x.update(-error_x))
                yaw = max(min(yaw, 120), -120)
                vz = float(self.pid_y.update(-error_y))
                vz = max(min(vz, 60), -60)
                twist_msg.angular.z = yaw / 100
                twist_msg.linear.z = vz / 100

                area = w * h
                if self.desired_area is None:
                    self.desired_area = area
                    self.pid_dist.setpoint = self.desired_area
                fd = float(self.pid_dist.update(area))
                fd = max(min(fd, 100), -100)
                twist_msg.linear.x = fd / 100

            elif self.locked:
                self.lost_counter += 1

                # 🔁 自动恢复逻辑：检测中心附近新 ID
                for track in online_targets:
                    x1, y1, w, h = map(int, track.tlwh)
                    cx = x1 + w // 2
                    cy = y1 + h // 2
                    dx = abs(cx - (img_w // 2))
                    dy = abs(cy - (img_h // 2))
                    if dx < 60 and dy < 60:
                        self.get_logger().info(f"🎯 目标恢复：锁定新 ID = {track.track_id}")
                        self.target_id = track.track_id
                        self.lost_counter = 0
                        break

                if self.lost_counter > 60:
                    self.get_logger().warn("⚠️ 目标丢失超时，解除锁定")
                    self.locked = False
                    self.target_id = None
                    self.lost_counter = 0

            self.cmd_pub.publish(twist_msg)
            result_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.result_pub.publish(result_msg)

        except Exception as e:
            self.get_logger().error(f"❌ 图像处理出错: {e}")

    def destroy_node(self):
        self.cmd_pub.publish(Twist())
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
