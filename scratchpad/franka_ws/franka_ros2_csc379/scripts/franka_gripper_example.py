import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from franka_msgs.action import Grasp, Homing, Move #GripperCommand
from std_srvs.srv import Trigger

class FrankaGripperActionClient(Node):
    def __init__(self):
        print("inited")
        super().__init__('gripper_action_client')
        self._homing_action_client = ActionClient(self, Homing, '/fr3_gripper/homing')
        self._move_action_client = ActionClient(self, Move, '/fr3_gripper/move')
        self._grasp_action_client = ActionClient(self, Grasp, '/fr3_gripper/grasp')
        self._cancel_action_client = self.create_client(Trigger, '/fr3_gripper/stop')
        while not self._cancel_action_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('cancel action service not available, waiting again...')        
        self.cancel_action_req = Trigger.Request()

    def _blocking_helper(self, future):
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result()

    def do_homing_async(self):
        homing_msg = Homing.Goal()
        self._homing_action_client.wait_for_server()
        return self._homing_action_client.send_goal_async(homing_msg)

    def do_homing_blocking(self):
        future = self.do_homing_async()
        return self._blocking_helper(future)

    def do_move_async(self, width, speed):
        move_msg = Move.Goal() 
        move_msg.width = width
        move_msg.speed = speed
        self._move_action_client.wait_for_server() 
        return self._move_action_client.send_goal_async(move_msg)

    def do_move_blocking(self, width, speed):
        future = self.do_move_async(width, speed)
        return self._blocking_helper(future)

    def do_grasp_async(self, width, speed, force = 50.0): 
        grasp_msg = Grasp.Goal()
        grasp_msg.width = width 
        grasp_msg.speed = speed 
        grasp_msg.force = force
        self._grasp_action_client.wait_for_server()
        return self._grasp_action_client.send_goal_async(grasp_msg)

    def do_grasp_blocking(self, width, speed, force = 50.0):
        future = self.do_grasp_async(width, speed, force)
        return self._blocking_helper(future)

    def cancel_action_async(self, ):
        self.future = self._cancel_action_client.call_async(self.cancel_action_req)
        return rclpy.spin_until_future_complete(self, self.future)

def main(args=None):
    rclpy.init(args=args)
    franka_gripper_client = FrankaGripperActionClient()

    # ASYNC calls
    print("calling async functions")
    future = franka_gripper_client.do_homing_async() 
    rclpy.spin_until_future_complete(franka_gripper_client, future)
    print("send homing goal result", future.result())

    future = franka_gripper_client.do_move_async(width=0.01, speed=0.2)
    rclpy.spin_until_future_complete(franka_gripper_client, future)
    print("send move result", future.result())

    future = franka_gripper_client.do_grasp_async(width=0.05, speed=0.2)
    rclpy.spin_until_future_complete(franka_gripper_client, future)
    goal_handle = future.result()
    print("send grasp result", goal_handle)

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(franka_gripper_client, result_future)
    print("Finished grasping")

    # SYNC
    print("calling blocking functions")
    print("homing...")
    franka_gripper_client.do_homing_blocking()
    print("finished homing...")

    print("moving...")
    franka_gripper_client.do_move_blocking(width=0.01, speed=0.2)
    print("finished moving...")

    print("grasping...")
    franka_gripper_client.do_grasp_blocking(width=0.05, speed=0.2)
    print("finished grasping...")

    # franka_gripper_client.cancel_action()

if __name__ == '__main__':
    main()