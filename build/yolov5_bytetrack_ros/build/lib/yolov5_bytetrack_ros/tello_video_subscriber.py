#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from djitellopy import Tello
import time

class TelloDirectControl(Node):
    def __init__(self):
        super().__init__('tello_direct_control')
        self.tello = Tello()
        self.tello.connect()
        self.get_logger().info(f"连接成功! 电量: {self.tello.get_battery()}%")
        
        # 执行飞行任务
        self.execute_mission()

    def execute_mission(self):
        try:
            self.tello.takeoff()
            self.get_logger().info("起飞指令已发送")
            time.sleep(2)

            self.tello.move_left(50)
            self.get_logger().info("向左移动中...")
            time.sleep(3)

            self.tello.land()
            self.get_logger().info("降落指令已发送")

        except Exception as e:
            self.get_logger().error(f"控制失败: {e}")
        finally:
            self.tello.end()
            self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TelloDirectControl()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
