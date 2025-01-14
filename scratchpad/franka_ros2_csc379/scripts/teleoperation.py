import threading
import pygame
import time
from spatialmath  import SE3
from spatialmath.base import *
import roboticstoolbox as rtb

import rclpy
import time
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args)
    node_handle = Node('teleoperation_node')

    spin_func = lambda _ : rclpy.spin(node_handle)
    spin_thread = threading.Thread(target=spin_func, args=(0,))
    spin_thread.start()
    time.sleep(1) # sleep to allow spin thread to get some messages

    pygame.init()
    screen = pygame.display.set_mode((640, 200))
    pygame.display.set_caption("Click and Press Keys to Teleop")
    pygame.mouse.set_visible(1)

    panda = rtb.models.DH.Panda()

    # Task: Use the read joint states and joint control interfaces from
    # problem set 1 and 2
    
    # Task: From problem set 1, Get the current joint states and calculate FK 
    # to get the Franka flange position and rotation transform
    desired_transform_se3 = panda.fkine(initial_joint_pos) # OR use this, but make sure its right
    current_joint_positions = initial_joint_pos
    
    # Task: Use a python keyboard input to increment the the Franka flange transform
    # by a small amount. Use different keys for change in xyz position and xyz orientations
    
    # Task: Solve the joint positions of this new transform using robotics toolbox panka IK
    
    if sol.success:
        joint_solutions = sol.q

    # Print helpers to check
    # print("desired_transform_se3", desired_transform_se3)
    # Check your joint solutions vs initial solutions
    # print("initial_joint_pos", initial_joint_pos)
    # print("joint_solutions", joint_solutions) 

    # Task: Send joint positions and command robot as in problem set 2

    spin_thread.join()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
