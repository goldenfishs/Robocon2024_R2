import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
import subprocess

mode = 0
class MySubscriber(Node):

    def __init__(self):
        super().__init__('my_subscriber')
        self.subscription = self.create_subscription(Int64,'robot_mode',self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        if msg.data == 2 | msg.data == 6 | msg.data == 1 | mode == 0:
            mode = 1
            self.get_logger().info('I heard: "%d"' % msg.data)
            subprocess.call(["pkill", "-f", "/home/ubuntu/r2_algorithm/src/rc_vision/rc_vision/pub_zhaokuang_vino.py"])
            
            subprocess.Popen(["python3", "/home/ubuntu/r2_algorithm/src/rc_vision/rc_vision/pub_quqiu.py"])
            
        elif msg.data == 7 :
            mode = 0
            self.get_logger().info('I heard: "%d"' % msg.data)
            subprocess.call(["pkill", "-f", "/home/ubuntu/r2_algorithm/src/rc_vision/rc_vision/pub_quqiu.py"])
            subprocess.Popen(["python3", "/home/ubuntu/r2_algorithm/src/rc_vision/rc_vision/pub_zhaokuang_vino.py"])
            
def main(args=None):
    rclpy.init(args=args)

    my_subscriber = MySubscriber()

    rclpy.spin(my_subscriber)

    my_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()