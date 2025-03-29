import time
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from franka_msgs.action import Grasp, Homing, Move
from std_srvs.srv import Trigger

# ------- Copied from rclpy source code and modified --------
class TimeoutObject:
    """Use timeout object to save timeout."""

    def __init__(self, timeout: float) -> None:
        self._timeout = timeout

    @property
    def timeout(self) -> float:
        return self._timeout

    @timeout.setter
    def timeout(self, timeout: float) -> None:
        self._timeout = timeout

# copied from rclpy.spin_until_future_complete, remove spins, and add a hardcoded 0.001 sleep
def wait_until_future_complete(
    future,
    timeout_sec: Optional[float] = None # None will wait indefinitely
) -> None:

    if timeout_sec is None or timeout_sec < 0:
        while (not future.done() and not future.cancelled()):
            time.sleep(0.001) # 100hz
    else:
        start = time.monotonic()
        end = start + timeout_sec
        timeout_left = TimeoutObject(timeout_sec)
        while (not future.done() and not future.cancelled()):
            time.sleep(0.001) # 100hz
            now = time.monotonic()
            if now >= end:
                return
            timeout_left.timeout = end - now
# -------------------------------------------------------

# This class assumes you have a spin thread running outside of it
class FrankaGripperActionClient():
    def __init__(self, node_handle):
        self._homing_action_client = ActionClient(node_handle, Homing, '/fr3_gripper/homing')
        self._move_action_client = ActionClient(node_handle, Move, '/fr3_gripper/move')
        self._grasp_action_client = ActionClient(node_handle, Grasp, '/fr3_gripper/grasp')
        self._cancel_action_server_client = node_handle.create_client(Trigger, '/fr3_gripper/stop')
        # while not self._cancel_action_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('cancel action service not available, waiting again...')        
        self.cancel_action_req = Trigger.Request()

        self.homing_future = None
        self.moving_future = None
        self.grasping_future = None

    def _action_blocking_helper(self, future):
        wait_until_future_complete(future)
        goal_handle = future.result()
        result_future = goal_handle.get_result_async()
        wait_until_future_complete(result_future)
        return result_future.result()

    def _check_currently_being_executed(goal_future):
        if not goal_future:
            return False
        wait_until_future_complete(goal_future)
        goal_handle = goal_future.result()
        result_future = goal_handle.get_result_async()
        return not result_future.done()

    # Will not send again, if there is one being currently executed
    def do_homing_async(self):
        self._homing_action_client.wait_for_server()
        if self._check_currently_being_executed(self.homing_future):
            return None
        homing_msg = Homing.Goal()
        self.homing_future = self._homing_action_client.send_goal_async(homing_msg)
        return self.homing_future

    def do_homing_blocking(self):
        future = self.do_homing_async()
        if future is None:
            return None
        return self._action_blocking_helper(future)

    def do_move_async(self, width, speed):
        self._move_action_client.wait_for_server() 
        if self._check_currently_being_executed(self.moving_future):
            return None
        move_msg = Move.Goal() 
        move_msg.width = width
        move_msg.speed = speed
        self.moving_future = self._move_action_client.send_goal_async(move_msg)
        return self.moving_future

    def do_move_blocking(self, width, speed):
        future = self.do_move_async(width, speed)
        if future is None:
            return None
        return self._action_blocking_helper(future)

    def do_grasp_async(self, width, speed, force = 50.0): 
        self._grasp_action_client.wait_for_server()
        if self._check_currently_being_executed(self.grasping_future):
            return None
        grasp_msg = Grasp.Goal()
        grasp_msg.width = width 
        grasp_msg.speed = speed 
        grasp_msg.force = force
        self.grasping_future = self._grasp_action_client.send_goal_async(grasp_msg)
        return self.grasping_future

    def do_grasp_blocking(self, width, speed, force = 50.0):
        future = self.do_grasp_async(width, speed, force)
        if future is None:
            return None
        return self._action_blocking_helper(future)

    def cancel_action_async(self): # -> Future
        return self._cancel_action_server_client.call_async(self.cancel_action_req)
    
    def cancel_action_blocking(self):
        future = self.cancel_action_async()
        wait_until_future_complete(future)
        return future.result()
