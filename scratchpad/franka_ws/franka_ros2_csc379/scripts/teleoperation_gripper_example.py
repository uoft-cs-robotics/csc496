import pygame
import threading
import time

import rclpy
from rclpy.node import Node

from gripper_interfaces import FrankaGripperActionClient

def main(args=None):
    rclpy.init(args=args)

    node_handle = Node('teleoperation_gripper_node')
    fgac = FrankaGripperActionClient(node_handle)

    spin_func = lambda _ : rclpy.spin(node_handle)
    spin_thread = threading.Thread(target=spin_func, args=(0,))
    spin_thread.start()
    time.sleep(2) # sleep to allow spin thread to get some messages

    pygame.init()
    screen = pygame.display.set_mode((640, 200))
    pygame.display.set_caption("Click and Press Keys to Teleop")
    pygame.mouse.set_visible(1)    

    while (rclpy.ok()):
        events = pygame.event.get()
        keys = pygame.key.get_pressed()
        
        if keys[pygame.K_h]:
            future = fgac.do_homing_async() 
        if keys[pygame.K_o]:
            future = fgac.do_move_async(width=0.06, speed=0.2) 
        if keys[pygame.K_c]:
            future = fgac.do_move_async(width=0.01, speed=0.2)                  

        time.sleep(0.1)
        
    spin_thread.join()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
