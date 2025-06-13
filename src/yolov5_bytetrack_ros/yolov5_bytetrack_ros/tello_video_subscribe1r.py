import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from djitellopy import Tello
import time

class TelloNode(Node):
    def __init__(self):
        super().__init__('tello_node')

        # 初始化 Tello
        self.tello = Tello()
        self.tello.connect()
        self.get_logger().info(f"Tello 电量: {self.tello.get_battery()}%")

        # 开启视频流
        self.tello.streamon()
        time.sleep(2)  # 等待视频流稳定

        # 发布图像话题
        self.image_pub = self.create_publisher(Image, '/tello/image_raw', 10)
        self.cv_bridge = CvBridge()

        # 订阅速度指令话题
        self.sub = self.create_subscription(Twist, '/tello/cmd_vel', self.cmd_vel_callback, 10)

        # 定时器发布图像（约33Hz）
        self.timer_image = self.create_timer(0.03, self.publish_frame)

        # 定时器持续发送rc控制命令（10Hz）
        self.timer_rc = self.create_timer(0.1, self.send_rc_control_periodic)

        self.frame_count = 0

        # 缓存当前速度指令，初始为停止
        self.current_lr = 0
        self.current_fb = 0
        self.current_ud = 0
        self.current_yw = 0
                # 延迟起飞，防止初始化时 Tello 状态未准备好
        self.create_timer(3.0, self.delayed_takeoff, callback_group=None)  # 3秒后调用一次

    def delayed_takeoff(self):
        battery = self.tello.get_battery()
        if battery < 20:
            self.get_logger().warn(f"电量过低（{battery}%），无法起飞")
            return
        try:
            self.get_logger().info("延迟起飞中...")
            self.tello.takeoff()
            self.get_logger().info("起飞成功")
        except Exception as e:
            self.get_logger().error(f"起飞失败：{e}")




    def publish_frame(self):
        frame = self.tello.get_frame_read().frame
        if frame is not None:
            img_msg = self.cv_bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.image_pub.publish(img_msg)
            self.frame_count += 1
            if self.frame_count % 30 == 0:
                self.get_logger().info(f'Published {self.frame_count} frames')
        else:
            self.get_logger().warn('No frame received from Tello')

    def cmd_vel_callback(self, msg):
        # 将接收到的速度命令转换为[-100, 100]整数范围，并缓存
        self.current_lr = int(max(min(msg.linear.y * 100, 30), -30))#30-  -30 x<、xianfu 
        self.current_fb = int(max(min(msg.linear.x * 100, 30), -30))
        self.current_ud = int(max(min(msg.linear.z * 100, 30), -30))
        self.current_yw = int(max(min(msg.angular.z * 100, 30), -30))
        self.get_logger().info(f"Received cmd_vel: lr={self.current_lr}, fb={self.current_fb}, ud={self.current_ud}, yw={self.current_yw}")

    def send_rc_control_periodic(self):
        # 持续发送当前缓存的速度指令，保证无人机持续响应
        #self.tello.send_rc_control(self.current_lr, self.current_fb, self.current_ud, self.current_yw)
        self.tello.send_rc_control(self.current_lr, self.current_fb, 0, 0)#修改评比 屏蔽
       # self.tello.send_rc_control(0, 30, 0, 0)
        self.get_logger().debug(f"Sent rc control: lr={self.current_lr}, fb={self.current_fb}, ud={self.current_ud}, yw={self.current_yw}")

    def destroy_node(self):
        # 停止运动，安全降落并断开连接
        self.tello.send_rc_control(0, 0, 0, 0)
        self.tello.land()
        self.tello.end()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TelloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
