import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64

class ModelSwitcher(Node):
    def __init__(self):
        super().__init__('model_switcher')
        self.publisher_ = self.create_publisher(Int64, 'robot_mode', 10)
        timer_period = 0.1  # seconds (10Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.msg = Int64()
        self.sent = False  # 添加状态标记

    def timer_callback(self):
        if not self.sent:  # 检查状态标记
            self.msg.data = 2
            self.publisher_.publish(self.msg)
            self.get_logger().info('Publishing: "%d"' % self.msg.data)
            self.sent = True  # 设置状态标记

def main():
    rclpy.init()

    modelSwitcher = ModelSwitcher()  # 创建节点

    rclpy.spin(modelSwitcher)

    modelSwitcher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()