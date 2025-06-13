import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
from djitellopy import Tello
import time


class TelloImagePublisher(Node):
    def __init__(self):
        super().__init__('tello_image_publisher')

        self.publisher_ = self.create_publisher(Image, '/tello/image_raw', 10)
        self.br = CvBridge()

        self.get_logger().info('Connecting to Tello...')
        self.tello = Tello()
        self.tello.connect()
        self.tello.streamon()
        self.get_logger().info('Tello connected and stream on.')

        self.get_logger().info('Waiting for Tello video stream to stabilize...')
        time.sleep(2)  # 等待视频流稳定（很关键）

        self.frame_count = 0
        self.timer = self.create_timer(0.03, self.timer_callback)  # ~30 FPS

    def timer_callback(self):
        frame = self.tello.get_frame_read().frame
        if frame is None:
            self.get_logger().warn('Failed to get frame from Tello.')
            return
        img_msg = self.br.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher_.publish(img_msg)

        self.frame_count += 1
        if self.frame_count % 30 == 0:
            self.get_logger().info(f'Published {self.frame_count} frames')

    def destroy_node(self):
        try:
            self.tello.streamoff()  # 尝试关闭视频流
        except Exception as e:
            self.get_logger().error(f"Error while turning off stream: {e}")

    # 其他关闭操作
        super().destroy_node()  # 确保关闭 ROS2 节点



def main(args=None):
    rclpy.init(args=args)
    node = TelloImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

