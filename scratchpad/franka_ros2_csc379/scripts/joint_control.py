import rclpy
import threading
import time
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import JointState

# Taken from ros docs minimal publisher

class FrankaStateInterface():
    def __init__(self, node_handle):
        self.publisher_ = node_handle.create_publisher(
            JointState, # Task: Use the correct topic type
            '/franka/servo_jp',  # Task: Use the correct topic name
            10)
        
        self.subscription = node_handle.create_subscription(
            JointState, # Task: Change to the correct topic type
            '/franka/measured_js', # Task: Change to the correct topic name
            self.listener_callback,
            10)
        self.joint_positions = None

    def publish_joints(self, joint_command):
        pass

    def listener_callback(self, msg):
        pass

def main(args=None):
    rclpy.init(args=args)

    node_handle = Node('joint_control')
    fsi = FrankaStateInterface(node_handle)
    spin_func = lambda _ : rclpy.spin(node_handle)
    spin_thread = threading.Thread(target=spin_func, args=(0,))
    spin_thread.start()
    time.sleep(1) # sleep to allow spin thread to get some messages
    
    # Task: Get the current joint state as in problem set 1

    # Task: Create a joint trajectory with a desired goal joint state
    # within the franka q limits
    # and the start joint state as the robots current state
    
    # Task: Send your trajectory and move the Franka robot
    while rclpy.ok():
        # Task: Print your trajectory first before uncommenting
        # the publisher

        fsi.publish_joints(command_joint_positions)
        # Task: Important to set your sleep! 
        # This is how fast you are sending the points,
        # which sets the velocity
        time.sleep(1) # The default 1s will be too slow

    spin_thread.join()
    fsi.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
