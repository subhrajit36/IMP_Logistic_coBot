#!/usr/bin/env python3
"""
Moving to multiple pose goals and controlling the gripper magnet using service calls, with different boxes for each position.
"""

from threading import Thread
import rclpy
import time
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5
from linkattacher_msgs.srv import AttachLink, DetachLink


class GripperControl(Node):
    def __init__(self):
        super().__init__('gripper_control')

        # Create service clients for AttachLink (GripperMagnetON) and DetachLink (GripperMagnetOFF)
        self.gripper_on_client = self.create_client(AttachLink, '/GripperMagnetON')
        self.gripper_off_client = self.create_client(DetachLink, '/GripperMagnetOFF')

        # Wait for the services to be available
        while not self.gripper_on_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('GripperMagnetON service not available, waiting...')

        while not self.gripper_off_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('GripperMagnetOFF service not available, waiting...')

    def gripper_magnet_on(self, box_name):
        """Service call to turn the gripper magnet ON (AttachLink)."""
        req = AttachLink.Request()
        # self.get_logger().info(f'box_name')
        req.model1_name = box_name
        req.link1_name = 'link'
        req.model2_name = 'ur5'
        req.link2_name = 'wrist_3_link'

        future = self.gripper_on_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if future.result() is not None:
            self.get_logger().info(f'Gripper magnet ON for {box_name}')
        else:
            self.get_logger().error('Failed to call GripperMagnetON service')
            time.sleep(2)  # wait for 2 seconds before retrying
            retry_future = self.gripper_on_client.call_async(req)
            rclpy.spin_until_future_complete(self, retry_future)
            if retry_future.result() is not None:
                self.get_logger().info(f'Gripper magnet ON for {box_name} after retry')
            else:
                self.get_logger().error('Failed to call GripperMagnetON service after retry')
                # You can exit or handle it in a way that makes sense for your application

    def gripper_magnet_off(self, box_name):
        """Service call to turn the gripper magnet OFF (DetachLink)."""
        req = DetachLink.Request()
        req.model1_name = box_name
        req.link1_name = 'link'
        req.model2_name = 'ur5'
        req.link2_name = 'wrist_3_link'

        future = self.gripper_off_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Gripper magnet OFF for {box_name}')
        else:
            self.get_logger().error('Failed to call GripperMagnetOFF service')


def move_to_pose_sequence(moveit2, node, poses, gripper_control):
    """Move to a sequence of poses and control the gripper at the appropriate points."""
    box_names = ['box49', 'box1', 'box3']
    
    for i, (position, quat_xyzw) in enumerate(poses):
        node.get_logger().info(f"Moving to {{position: {position}, quat_xyzw: {quat_xyzw}}}")

        moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw, cartesian=False)
        moveit2.wait_until_executed()
        node.get_logger().info("MoveIt2 execution finished")


        # Call gripper services at specific points (P1, P2, P3 for ON, and D for OFF)
        if i % 2 == 0:  # Gripper ON for P1, P2, P3
            box_name = box_names[i // 2]
            print(box_name)
            gripper_control.gripper_magnet_on(box_name)
            
            
        else:  # Gripper OFF for D
            box_name = box_names[(i - 1) // 2]
            gripper_control.gripper_magnet_off(box_name)
            

        node.get_logger().info(f"Reached pose {i+1}: {position}")


def main():
    rclpy.init()

    # Create node for the MoveIt2 interface and the gripper control node
    node = Node("gripper_control_with_pose_goal")
    gripper_control = GripperControl()

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=ur5.joint_names(),
        base_link_name=ur5.base_link_name(),
        end_effector_name=ur5.end_effector_name(),
        group_name=ur5.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    # Define the positions and orientations for each pose (P1 → D → P2 → D → P3 → D)
    poses = [
        
        ([0.20, -1.47, 0.65], [0.707, 0.707, 0.0, 0.0]),  # P1
        ([-0.69, 0.10, 0.44], [1.0, 0.0, 0.0, 0.0]),  # D       
        ([0.75, 0.49, -0.05], [1.0, 0.0, 0.0, 0.0]),  # P2
        ([-0.69, 0.10, 0.44], [1.0, 0.0, 0.0, 0.0]),  # D
        ([0.75, -0.23, -0.05],[1.0, 0.0, 0.0, 0.0]), # P3
        ([-0.69, 0.10, 0.44], [1.0, 0.0, 0.0, 0.0]),   # D
   
    ]

    # Move to each pose in sequence and control the gripper
    move_to_pose_sequence(moveit2, node, poses, gripper_control)

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()

