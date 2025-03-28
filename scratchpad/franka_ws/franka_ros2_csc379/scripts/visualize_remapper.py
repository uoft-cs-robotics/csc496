import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
import numpy as np

# Solution for A5

class FrankaStateSubscriber():
    def __init__(self, node_handle):
        self.subscription = node_handle.create_subscription(
            JointState,
            '/franka/measured_js',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        pass       

def main(args=None):
    rclpy.init(args=args)
    node_handle = Node('remapper')
    remapper = FrankaStateSubscriber(node_handle)
    rclpy.spin(node_handle)
    remapper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
