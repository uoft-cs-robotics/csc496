import rclpy
from rclpy.node import Node

from std_msgs.msg import String

# Taken from ros docs minimal subscriber

class JointStateSubcriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.subscription = self.create_subscription(
            String, # Task: Change to the correct topic type
            'topic', # Task: Change to the correct topic name
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # Task: Read the joint states and print

        # Task: Create an FK DH model and print the tip position
        pass

def main(args=None):
    rclpy.init(args=args)
    joint_state_subscriber = JointStateSubcriber()
    rclpy.spin(joint_state_subscriber)
    joint_state_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
