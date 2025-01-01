import rclpy
import threading
import time
from rclpy.node import Node

from std_msgs.msg import String

# Taken from ros docs minimal publisher

class JointStatePublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(
            String, # Task: Use the correct topic type
            'topic',  # Task: Use the correct topic name
            10)

    def publish_joints(self, joint_command):
        # self.publisher_.publish(msg)
        pass

def main(args=None):
    rclpy.init(args=args)

    joint_state_publisher = JointStatePublisher()

    spin_func = lambda _ : rclpy.spin(joint_state_publisher)
    thread = threading.Thread(target=spin_func, args=(0,))
    thread.start()

    # Task: Get the current joint state as in problem set 1

    # Task: Create a joint trajectory with a desired goal joint state
    # within the franka q limits
    # and the start joint state as the robots current state
    
    # Task: Validate the trajectory with joint speed not exceeding
    # the qmax of the Franka joint

    # Task: Send your trajectory and move the Franka robot
    while rclpy.ok():
        # Task: Print your trajectory first before uncommenting
        # the publisher

        joint_state_publisher.publish_joints([0, 0, 0, 0, 0, 0, 0])
        # Task: Important to set your sleep! 
        # This is how fast you are sending the points,
        # which sets the velocity
        time.sleep(1) # The default 1s will be too slow

    joint_state_publisher.destroy_node()
    rclpy.shutdown()
    thread.join()

if __name__ == '__main__':
    main()
