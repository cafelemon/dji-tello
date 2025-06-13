import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TelloControlNode(Node):
    def __init__(self):
        super().__init__('tello_control_node')
        self.cmd_pub = self.create_publisher(Twist, '/tello/cmd_vel', 10)
        self.get_logger().info('Tello Control Node 已启动')

        # 延时起飞示例
        self.timer = self.create_timer(1.0, self.control_loop)
        self.step = 0

    def control_loop(self):
        twist = Twist()

        if self.step == 0:
            self.get_logger().info('起飞并悬停...')
            # 一般Tello通过sdk命令起飞，这里示例只发送速度命令让它悬停
            # 真实起飞需要先用SDK命令起飞（此处仅为速度控制示例）
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)

        elif self.step == 1:
            self.get_logger().info('向右移动...')
            twist.linear.y = 0.2  # 向右移动速度
            self.cmd_pub.publish(twist)

        elif self.step == 2:
            self.get_logger().info('停止移动，悬停...')
            twist.linear.y = 0.0
            self.cmd_pub.publish(twist)

        elif self.step == 3:
            self.get_logger().info('向上移动...')
            twist.linear.z = 0.2
            self.cmd_pub.publish(twist)

        elif self.step == 4:
            self.get_logger().info('停止移动，悬停...')
            twist.linear.z = 0.0
            self.cmd_pub.publish(twist)

        self.step += 1
        if self.step > 4:
            self.get_logger().info('动作演示结束，关闭节点')
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = TelloControlNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

