"""Publish a video file as a ROS2 camera stream for repeatable offline runs."""

from __future__ import annotations

from cv_bridge import CvBridge
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class OfflineVideoPublisher(Node):
    def __init__(self) -> None:
        super().__init__('offline_video_publisher')
        self.video_path = str(self.declare_parameter('video_path', '').value)
        self.loop = bool(self.declare_parameter('loop', False).value)
        requested_rate = float(self.declare_parameter('publish_rate_hz', 0.0).value)
        if not self.video_path:
            raise ValueError('video_path parameter is required')
        self.capture = cv2.VideoCapture(self.video_path)
        if not self.capture.isOpened():
            raise RuntimeError(f'cannot open video: {self.video_path}')
        source_rate = self.capture.get(cv2.CAP_PROP_FPS)
        self.publish_rate = requested_rate if requested_rate > 0.0 else max(1.0, source_rate)
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, '/tello/image_raw', qos_profile_sensor_data)
        self.timer = self.create_timer(1.0 / self.publish_rate, self._publish_next)

    def _publish_next(self) -> None:
        ok, frame = self.capture.read()
        if not ok and self.loop:
            self.capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self.capture.read()
        if not ok:
            self.get_logger().info('offline video complete')
            self.timer.cancel()
            return
        message = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'offline_camera'
        self.publisher.publish(message)

    def destroy_node(self) -> bool:
        self.capture.release()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OfflineVideoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
