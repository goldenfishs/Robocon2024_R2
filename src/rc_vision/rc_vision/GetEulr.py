import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformStamped
from scipy.spatial.transform import Rotation as R
import numpy as np

class QuaternionToEulerNode(Node):
    def __init__(self):
        super().__init__('quaternion_to_euler_node')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/filtered_pose',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        quaternion = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        )
        r = R.from_quat(quaternion)
        euler = r.as_euler('xyz', degrees=True)
        self.get_logger().info('Roll: %f, Pitch: %f, Yaw: %f' % tuple(euler))

def main(args=None):
    rclpy.init(args=args)
    quaternion_to_euler_node = QuaternionToEulerNode()
    rclpy.spin(quaternion_to_euler_node)
    quaternion_to_euler_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()