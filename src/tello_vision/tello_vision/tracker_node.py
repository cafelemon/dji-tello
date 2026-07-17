"""YOLOv5 + ByteTrack node that processes only the latest camera frame."""

from __future__ import annotations

import os
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace

from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TwistStamped
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from tello_flight_manager.pid import Limits, PIDController
from tello_interfaces.msg import TrackingStatus
from tello_interfaces.srv import SetTarget

from .latest_slot import LatestSlot
from .reacquire import select_reacquire_candidate


def resolve_project_path(value: str) -> Path:
    """Resolve a configured path without embedding a user-specific directory."""
    path = Path(value).expanduser()
    if path.is_absolute():
        return path
    root = os.environ.get('TELLO_EDGE_ROOT')
    if root:
        return Path(root) / path
    return Path.cwd() / path


def scale_coords(img_shape, coords, original_shape):
    coords = coords.clone()
    gain = min(img_shape[0] / original_shape[0], img_shape[1] / original_shape[1])
    pad_x = (img_shape[1] - original_shape[1] * gain) / 2
    pad_y = (img_shape[0] - original_shape[0] * gain) / 2
    coords[:, [0, 2]] -= pad_x
    coords[:, [1, 3]] -= pad_y
    coords[:, :4] /= gain
    coords[:, 0].clamp_(0, original_shape[1])
    coords[:, 1].clamp_(0, original_shape[0])
    coords[:, 2].clamp_(0, original_shape[1])
    coords[:, 3].clamp_(0, original_shape[0])
    return coords


class TrackerNode(Node):
    def __init__(self) -> None:
        super().__init__('tello_tracker')
        self.bridge = CvBridge()
        self.image_size = int(self.declare_parameter('image_size', 640).value)
        self.device_name = str(self.declare_parameter('device', 'cpu').value)
        self.inference_backend = str(
            self.declare_parameter('inference_backend', 'pytorch').value
        ).lower()
        if self.inference_backend not in {'pytorch', 'tensorrt'}:
            raise ValueError('inference_backend must be pytorch or tensorrt')
        self.fp16 = bool(self.declare_parameter('fp16', False).value)
        self.weights_path = resolve_project_path(
            str(self.declare_parameter('weights_path', 'models/yolov5s.pt').value)
        )
        self.yolov5_repo = resolve_project_path(
            str(self.declare_parameter('yolov5_repo', 'vendor/yolov5').value)
        )
        self.bytetrack_repo = resolve_project_path(
            str(self.declare_parameter('bytetrack_repo', 'vendor/ByteTrack').value)
        )
        self.engine_path = resolve_project_path(
            str(self.declare_parameter('engine_path', 'models/yolov5s_fp16.engine').value)
        )
        self.auto_select_target = bool(
            self.declare_parameter('auto_select_target', False).value
        )
        self.diagnostics_period_s = float(
            self.declare_parameter('diagnostics_period_s', 1.0).value
        )
        if self.diagnostics_period_s <= 0.0:
            raise ValueError('diagnostics_period_s must be positive')
        self.confidence_threshold = float(
            self.declare_parameter('confidence_threshold', 0.25).value
        )
        self.iou_threshold = float(self.declare_parameter('iou_threshold', 0.45).value)
        self.reacquire_timeout_s = float(
            self.declare_parameter('reacquire_timeout_s', 2.0).value
        )
        self.reacquire_center_ratio = float(
            self.declare_parameter('reacquire_center_ratio', 0.15).value
        )
        self.reacquire_area_ratio_min = float(
            self.declare_parameter('reacquire_area_ratio_min', 0.5).value
        )
        self.reacquire_area_ratio_max = float(
            self.declare_parameter('reacquire_area_ratio_max', 2.0).value
        )
        self.desired_area_parameter = float(self.declare_parameter('desired_area', 0.0).value)

        self.yaw_pid = self._new_pid('yaw_pid', 0.002, 0.0, 0.0001, -1.0, 1.0)
        self.vertical_pid = self._new_pid('vertical_pid', 0.002, 0.0, 0.0001, -0.6, 0.6)
        self.distance_pid = self._new_pid('distance_pid', 0.00001, 0.0, 0.000001, -1.0, 1.0)

        self.target_id: int | None = None
        self.desired_area: float | None = None
        self.last_target_box: tuple[float, float, float, float] | None = None
        self.target_lost_since: float | None = None
        self.slot: LatestSlot[Image] = LatestSlot()
        self.running = True
        self.received_frames = 0
        self.processed_frames = 0
        self.dropped_frames = 0
        self.error_count = 0
        self.last_inference_ms = 0.0
        self.last_frame_age_ms = 0.0
        self.diagnostics_started = time.monotonic()

        self._load_backend()
        self.image_sub = self.create_subscription(
            Image, '/tello/image_raw', self._on_image, qos_profile_sensor_data
        )
        self.result_pub = self.create_publisher(Image, '/yolo/image_out', qos_profile_sensor_data)
        self.command_pub = self.create_publisher(TwistStamped, '/tracking/cmd_vel', 1)
        self.status_pub = self.create_publisher(TrackingStatus, '/tracking/status', 10)
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, '/tracking/diagnostics', 10
        )
        self.set_target_service = self.create_service(
            SetTarget, '/tracking/set_target', self._set_target
        )
        self.clear_target_service = self.create_service(
            Trigger, '/tracking/clear_target', self._clear_target
        )
        self.worker = threading.Thread(target=self._worker_loop, name='vision-latest-frame', daemon=True)
        self.worker.start()
        self.diagnostics_timer = self.create_timer(
            self.diagnostics_period_s, self._publish_diagnostics
        )
        self.get_logger().info('Tracker ready; waiting for target selection')

    def _new_pid(self, prefix, kp, ki, kd, minimum, maximum):
        return PIDController(
            float(self.declare_parameter(f'{prefix}.kp', kp).value),
            float(self.declare_parameter(f'{prefix}.ki', ki).value),
            float(self.declare_parameter(f'{prefix}.kd', kd).value),
            integral_limits=Limits(-100.0, 100.0),
            output_limits=Limits(minimum, maximum),
            deadband=float(self.declare_parameter(f'{prefix}.deadband', 5.0).value),
            filter_alpha=float(self.declare_parameter(f'{prefix}.filter_alpha', 0.3).value),
        )

    def _load_backend(self) -> None:
        # The pinned ByteTrack commit still uses aliases removed in NumPy 1.24.
        for alias, builtin in (('float', float), ('int', int), ('bool', bool)):
            if alias not in np.__dict__:
                setattr(np, alias, builtin)
        model_path = self.engine_path if self.inference_backend == 'tensorrt' else self.weights_path
        missing = [path for path in (model_path, self.yolov5_repo, self.bytetrack_repo) if not path.exists()]
        if missing:
            raise FileNotFoundError('missing vision dependency: ' + ', '.join(str(path) for path in missing))
        sys.path.insert(0, str(self.yolov5_repo))
        sys.path.insert(0, str(self.bytetrack_repo))
        import torch
        from models.common import DetectMultiBackend
        from utils.augmentations import letterbox
        from utils.general import non_max_suppression
        from yolox.tracker.byte_tracker import BYTETracker

        self.torch = torch
        self.letterbox = letterbox
        self.non_max_suppression = non_max_suppression
        self.device = torch.device(self.device_name)
        self.model = DetectMultiBackend(
            str(model_path), device=self.device, fp16=self.fp16
        )
        self.model.eval()
        self.stride = int(self.model.stride)
        self.tracker = BYTETracker(
            SimpleNamespace(
                track_thresh=float(self.declare_parameter('track_threshold', 0.3).value),
                track_buffer=int(self.declare_parameter('track_buffer', 30).value),
                match_thresh=float(self.declare_parameter('match_threshold', 0.6).value),
                min_box_area=float(self.declare_parameter('min_box_area', 10.0).value),
                mot20=False,
            ),
            frame_rate=float(self.declare_parameter('nominal_frame_rate', 30.0).value),
        )

    def _on_image(self, message: Image) -> None:
        self.received_frames += 1
        self.slot.put(message)

    def _set_target(self, request: SetTarget.Request, response: SetTarget.Response):
        if request.track_id < 0:
            response.success = False
            response.message = 'track_id must be non-negative'
            return response
        self.target_id = int(request.track_id)
        self.desired_area = self.desired_area_parameter or None
        self.last_target_box = None
        self.target_lost_since = None
        self._reset_pids()
        response.success = True
        response.message = f'target {self.target_id} locked'
        return response

    def _clear_target(self, _request: Trigger.Request, response: Trigger.Response):
        self.target_id = None
        self.desired_area = None
        self.last_target_box = None
        self.target_lost_since = None
        self._reset_pids()
        response.success = True
        response.message = 'target cleared'
        return response

    def _reset_pids(self) -> None:
        self.yaw_pid.reset()
        self.vertical_pid.reset()
        self.distance_pid.reset()

    def _worker_loop(self) -> None:
        sequence = 0
        while self.running:
            previous_sequence = sequence
            sequence, message = self.slot.wait_next(sequence)
            if message is None:
                continue
            self.dropped_frames += max(0, sequence - previous_sequence - 1)
            try:
                self._process_image(message)
            except Exception as exc:
                self.error_count += 1
                self.get_logger().error(f'image processing failed: {exc}')
                self._publish_status(False, reason=str(exc))
                self.command_pub.publish(TwistStamped())

    def _process_image(self, message: Image) -> None:
        processing_started = time.monotonic()
        frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        image_height, image_width = frame.shape[:2]
        prepared = self.letterbox(frame, self.image_size, stride=self.stride, auto=True)[0]
        prepared = prepared[:, :, ::-1].transpose(2, 0, 1)
        tensor = self.torch.from_numpy(np.ascontiguousarray(prepared)).to(self.device)
        tensor = tensor.half() if self.model.fp16 else tensor.float()
        tensor /= 255.0
        if tensor.ndimension() == 3:
            tensor = tensor.unsqueeze(0)
        with self.torch.no_grad():
            prediction = self.model(tensor, augment=False, visualize=False)
        self.last_inference_ms = (time.monotonic() - processing_started) * 1000.0
        prediction = self.non_max_suppression(
            prediction, self.confidence_threshold, self.iou_threshold, classes=[0]
        )[0]

        detections = []
        if prediction is not None and len(prediction):
            prediction[:, :4] = scale_coords(tensor.shape[2:], prediction[:, :4], frame.shape).round()
            for *xyxy, confidence, _class_id in prediction:
                x1, y1, x2, y2 = map(float, xyxy)
                detections.append([x1, y1, x2, y2, float(confidence)])
        detection_array = np.asarray(detections, dtype=np.float32)
        if detection_array.size == 0:
            detection_array = np.empty((0, 5), dtype=np.float32)
        targets = self.tracker.update(
            self.torch.from_numpy(detection_array),
            (image_height, image_width),
            (image_height, image_width),
        )

        if self.target_id is None and self.auto_select_target and targets:
            selected_for_lock = max(targets, key=lambda item: float(item.tlwh[2] * item.tlwh[3]))
            self.target_id = int(selected_for_lock.track_id)
            self.desired_area = float(selected_for_lock.tlwh[2] * selected_for_lock.tlwh[3])
            self._reset_pids()
            self.get_logger().info(f'auto-selected offline target {self.target_id}')

        selected = None
        for target in targets:
            x, y, width, height = map(float, target.tlwh)
            color = (0, 0, 255) if target.track_id == self.target_id else (0, 255, 0)
            cv2.rectangle(frame, (int(x), int(y)), (int(x + width), int(y + height)), color, 2)
            cv2.putText(
                frame, f'ID:{target.track_id}', (int(x), max(15, int(y) - 5)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2,
            )
            if target.track_id == self.target_id:
                selected = target

        now = time.monotonic()
        if selected is None and self.target_id is not None:
            selected = self._restricted_reacquire(targets, image_width, image_height, now)

        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = 'tello_camera'
        if selected is not None:
            self.target_lost_since = None
            x, y, width, height = map(float, selected.tlwh)
            self.last_target_box = (x, y, width, height)
            center_x = x + width / 2.0
            center_y = y + height / 2.0
            area = width * height
            if self.desired_area is None:
                self.desired_area = area
            self.distance_pid.setpoint = self.desired_area
            command.twist.angular.z = self.yaw_pid.update(center_x - image_width / 2.0, now=now)
            command.twist.linear.z = self.vertical_pid.update(center_y - image_height / 2.0, now=now)
            command.twist.linear.x = self.distance_pid.update(area, now=now)
            self._publish_status(
                True,
                center_x=center_x,
                center_y=center_y,
                width=width,
                height=height,
                reason='target visible',
            )
        else:
            if self.target_id is not None and self.target_lost_since is None:
                self.target_lost_since = now
            self._reset_pids()
            self._publish_status(False, reason='target not visible')
        self.command_pub.publish(command)
        output = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        output.header = message.header
        self.result_pub.publish(output)
        self.processed_frames += 1
        stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        if stamp_ns > 0:
            self.last_frame_age_ms = max(
                0.0, (self.get_clock().now().nanoseconds - stamp_ns) / 1_000_000.0
            )

    def _restricted_reacquire(self, targets, image_width: int, image_height: int, now: float):
        if self.last_target_box is None or self.target_lost_since is None:
            return None
        if now - self.target_lost_since > self.reacquire_timeout_s:
            return None
        candidate_boxes = [tuple(map(float, target.tlwh)) for target in targets]
        index = select_reacquire_candidate(
            self.last_target_box,
            candidate_boxes,
            elapsed_s=now - self.target_lost_since,
            timeout_s=self.reacquire_timeout_s,
            image_width=image_width,
            image_height=image_height,
            center_ratio=self.reacquire_center_ratio,
            area_ratio_min=self.reacquire_area_ratio_min,
            area_ratio_max=self.reacquire_area_ratio_max,
        )
        if index is None:
            return None
        target = targets[index]
        self.target_id = int(target.track_id)
        self.get_logger().info(f'reacquired target with constrained new ID {self.target_id}')
        return target

    def _publish_status(
        self,
        visible: bool,
        *,
        center_x: float = 0.0,
        center_y: float = 0.0,
        width: float = 0.0,
        height: float = 0.0,
        reason: str,
    ) -> None:
        status = TrackingStatus()
        status.stamp = self.get_clock().now().to_msg()
        status.locked = self.target_id is not None
        status.visible = visible
        status.target_id = self.target_id if self.target_id is not None else -1
        status.confidence = 0.0
        status.center_x = float(center_x)
        status.center_y = float(center_y)
        status.width = float(width)
        status.height = float(height)
        status.reason = reason
        self.status_pub.publish(status)

    def _publish_diagnostics(self) -> None:
        elapsed = max(1e-6, time.monotonic() - self.diagnostics_started)
        status = DiagnosticStatus()
        status.name = 'tello_vision/tracking'
        status.hardware_id = self.device_name
        status.level = DiagnosticStatus.OK if self.error_count == 0 else DiagnosticStatus.WARN
        status.message = 'tracking healthy' if self.error_count == 0 else 'tracking errors recorded'
        values = {
            'backend': self.inference_backend,
            'device': self.device_name,
            'fps': f'{self.processed_frames / elapsed:.3f}',
            'inference_ms': f'{self.last_inference_ms:.3f}',
            'frame_age_ms': f'{self.last_frame_age_ms:.3f}',
            'received_frames': str(self.received_frames),
            'processed_frames': str(self.processed_frames),
            'dropped_frames': str(self.dropped_frames),
            'error_count': str(self.error_count),
        }
        status.values = [KeyValue(key=key, value=value) for key, value in values.items()]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self.diagnostics_pub.publish(message)

    def destroy_node(self) -> bool:
        self.running = False
        self.slot.close()
        if hasattr(self, 'worker') and self.worker.is_alive():
            self.worker.join(timeout=2.0)
        self.command_pub.publish(TwistStamped())
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
