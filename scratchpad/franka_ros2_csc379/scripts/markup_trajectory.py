import threading
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray

# Solution for A6

class PoseArraySubscriber():
    def __init__(self, node_handle):        
        self.subscription = node_handle.create_subscription(
            PoseArray,
            '/markup_points',
            self.listener_callback,
            10)
        self.poses = None

    def listener_callback(self, msg):
        # Get your poses
        pass

def main(args=None):
    rclpy.init(args=args)

    node_handle = Node('markup_trajectory')
    pose_array_subscriber = PoseArraySubscriber(node_handle)
    # Task: Create your franka state subscriber here

    spin_func = lambda _ : rclpy.spin(node_handle)
    spin_thread = threading.Thread(target=spin_func, args=(0,))
    spin_thread.start()
    time.sleep(1) # sleep to allow spin thread to get some messages
    
    # Task: Get the markup points, add orientations
    # do IK, create joint trajectory, and execute joint trajectory.

    spin_thread.join()
    pose_array_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
