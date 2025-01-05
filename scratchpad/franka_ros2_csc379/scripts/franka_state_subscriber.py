import rclpy
from rclpy.node import Node

from std_msgs.msg import String

# Taken from ros docs minimal subscriber

class FrankaStateSubscriber():
    def __init__(self, node_handle):
        self.subscription = node_handle.create_subscription(
            String, # Task: Change to the correct topic type
            'topic', # Task: Change to the correct topic name
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # Task: Read the joint states and print

        # Task: Create an FK DH model and print the Franka flange position and rotation
        pass

def main(args=None):
    rclpy.init(args=args)
    node_handle = Node('joint_state_subscriber')
    joint_state_subscriber = FrankaStateSubscriber(node_handle)
    rclpy.spin(node_handle)
    joint_state_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
