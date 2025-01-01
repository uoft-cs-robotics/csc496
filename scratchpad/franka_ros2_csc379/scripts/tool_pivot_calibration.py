import threading
import time
import rclpy
from rclpy.node import Node

import roboticstoolbox as rtb

def main(args=None):
    rclpy.init(args=args)
    node_handle = Node('tool_pivot_calibration_node')

    spin_func = lambda _ : rclpy.spin(node_handle)
    spin_thread = threading.Thread(target=spin_func, args=(0,))
    spin_thread.start()
    
    # Task: Create the read joint states interface from problem set 1
    
    while rclpy.is_ok():
        # Task: From problem set 1, Get the current joint states and calculate FK 
        # to get Franka flange position and rotation transform
        current_transform_se3 = panda.fkine(current_joint_positions) # OR use this, but make sure its right
        
        # Task: Collect a dataset of current Franka flange posiitons while moving robot around
        # where the tip position is stationary (pivot calibration)
        
        time.sleep(0.5)

    # Task: Solve for least squares to get the Transform from Franka flange to tool tip

    rclpy.shutdown()
    spin_thread.join()

if __name__ == '__main__':
    main()
