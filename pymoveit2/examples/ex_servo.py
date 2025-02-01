#!/usr/bin/env python3
"""
Example of using MoveIt 2 Servo to achieve sequential poses for payload pickup and drop.
`ros2 run pymoveit2 ex_servo.py`
"""

import rclpy
from control_msgs.msg import JointJog
from sensor_msgs.msg import JointState
from linkattacher_msgs.srv import AttachLink, DetachLink
from payload_service.srv import PassingSRV 
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Int32MultiArray
from math import isclose
import time


class JointJogPublisher(Node):
    def __init__(self):
        super().__init__('joint_jog_publisher')
        self.waiting_for_drop = False  # State variable to track waiting for drop


        qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_ALL,
            )
        self.publisher_ = self.create_publisher(
            JointJog, '/servo_node/delta_joint_cmds', QoSProfile(depth=10)
        )
        # Subscription to marker_coordinates topic
        self.aruco_id_sub = self.create_subscription(
            Int32MultiArray,
            '/marker_ids',
            self.aruco_id_callback,
            10
        )
        self.timer = self.create_timer(0.008, self.timer_callback)  # Publish at ~125 Hz
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        pose_for_1_3_inter = [-1.53, -1.44, 1.35, -1.46, -1.57, 3.09]
        pose_for_1 = [-1.89, -1.30, 1.97, -2.22, -1.57, 2.73]
        pose_for_3 = [-0.95, -0.88, 1.31, -1.98, -1.56, 3.67]
        pose_for_2_inter = [1.25, -1.46, 1.27, -1.39, -1.56, 5.88]
        pose_for_2 = [0.88, -1.10, 1.66, -2.14, -1.55, 5.15]
        home_pose = [0.00, -2.39, 2.40, -3.15, -1.57, 3.15]
        drop_pose = [-0.05, -0.82, 1.89, -2.59, -1.56, 3.10]

        self.configurations = [
            pose_for_1_3_inter,
            pose_for_1,
            home_pose,
            drop_pose,
            home_pose,
            pose_for_2_inter,
            pose_for_2,
            home_pose,
            drop_pose,
            home_pose,
            pose_for_1_3_inter,
            pose_for_3,
            home_pose,
            drop_pose,
            home_pose 
        ]
        
        self.step = 0  # Track which configuration we're on
        self.current_joint_positions = [0.0] * 6  # Placeholder for current joint positions
        
        # Tolerance for reaching the target position (in radians for joint angles)
        self.tolerance = 0.01  # 0.01 rad (example tolerance)

        # Flag to stop publishing
        self.stop_publishing = False

        # Create service client for GripperMagnetON
        self.gripper_control_1 = self.create_client(AttachLink, '/GripperMagnetON')
    
        # Wait for the service to be available
        while not self.gripper_control_1.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Attach service not available, waiting again...')

        self.gripper_control_2 = self.create_client(DetachLink, '/GripperMagnetOFF')
        
        # Wait for the service to be available
        while not self.gripper_control_2.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Detach service not available, waiting again...')

        # Create PassingSRV service server
        self.passing_service = self.create_service(
            PassingSRV, '/passing_srv', self.handle_passing_service
        )
        self.docked = False  # Initial state
        self.get_logger().info("PassingSRV service ready.")
    
    def get_current_joint_positions(self, msg):
        """Callback to update current joint positions from the /joint_states topic."""
        self.current_joint_positions = msg.position

    def aruco_id_callback(self, msg):
        print("SUUIIIIIIIIIIIIIIIIIIIIIIIIIII")
        self.ids = msg.data  # Extract the list of IDs
        self.get_logger().info(f"Received Aruco IDs: {self.ids}")

        # Example: Check if a specific ID (e.g., 12) is in the list
        if 12 in self.ids:
            self.get_logger().info("Marker ID 12 detected! Taking action.")


    def check_target_reached(self, target_config):
        """Check if the arm has reached the target configuration."""
        for i in range(len(target_config)):
            if not isclose(self.current_joint_positions[i], target_config[i], abs_tol=self.tolerance):
                return False
        return True

    def attach_gripper(self, boxname):
        """Attach the gripper magnet after reaching the target pose."""
        req = AttachLink.Request()
        req.model1_name = boxname  # Replace with the name of the box you're attaching
        req.link1_name = 'link'   # Name of the gripper's link
        req.model2_name = 'ur5'   # Robot name (or your robot's URDF name)
        req.link2_name = 'wrist_3_link'  # Name of the gripper's link on the robot

        # Call the service asynchronously
        future = self.gripper_control_1.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        
        if future.result() is None :
            self.get_logger().info("Gripper Magnet ON - Object Attached")
            return True  # Return success for gripper attachment
        else:
            self.get_logger().info(f"Failed to attach object: {future.result().message}")
            return False  # Return failure if attachment fails
        
    def detach_gripper(self, boxname):
        """Call the DetachLink service to turn off the gripper magnet."""
        req = DetachLink.Request()
        req.model1_name = boxname  # Replace with the name of the box you're detaching
        req.link1_name = 'link'   # Name of the gripper's link
        req.model2_name = 'ur5'   # Robot name (or your robot's URDF name)
        req.link2_name = 'wrist_3_link'  # Name of the gripper's link on the robot

        # Call the service asynchronously
        future = self.gripper_control_2.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        
        if future.result() is None:
            self.get_logger().info("Gripper Magnet OFF - Object Detached")
            return True  # Return success for detachment
        else:
            self.get_logger().info(f"Failed to detach object: {future.result().message}")
            return False  # Return failure if detachment fails

    def handle_passing_service(self, req, response):
        """Handle PassingSRV requests to detach the gripper magnet."""
        response = PassingSRV.Response()

        # Debug log to check the value of req.drop
        self.get_logger().info(f"Received drop request: {req.drop}")

        if req.drop:
            self.get_logger().info("Passing service triggered to drop the object.")
            self.waiting_for_drop = True  # Set the state variable
            response.success = True
            response.message = "Waiting to drop the object."
        else:
            response.success = False
            response.message = "Drop request not set to true."

        return response
        

    def move_to_configuration(self, target_config):
        """Move the arm to a specific configuration."""
        while not self.check_target_reached(target_config):
            # Calculate joint displacements (delta)
            delta_config = [target_config[i] - self.current_joint_positions[i] for i in range(len(target_config))]
            
            # Create JointJog message
            jog_msg = JointJog()
            jog_msg.header.stamp = self.get_clock().now().to_msg()  # Set current time
            jog_msg.header.frame_id = ''
            jog_msg.joint_names = self.joint_names
            jog_msg.velocities = [3.5 * delta for delta in delta_config]  # Adjust velocity for faster movement
            jog_msg.duration = 0.01  # Duration for each velocity command
            
            # Publish the message
            self.publisher_.publish(jog_msg)
            self.get_logger().info(f'Publishing JointJog: {jog_msg}')
    
    def timer_callback(self):
        if self.stop_publishing:
            self.get_logger().info('Target reached. Stopping.')
            return

        if self.step >= 14:
            self.get_logger().info('All configurations executed, stopping.')
            self.stop_publishing = True
            return

        target_config = self.configurations[self.step]
        if self.check_target_reached(target_config):
            self.get_logger().info(f"Target configuration {self.step + 1} reached.")

            if self.step in [3, 8, 13]:  # Steps where the arm should drop the box
                if self.waiting_for_drop:
                    # Determine the box name based on step
                    if self.step == 3:
                        boxname = 'box1'
                    elif self.step == 8:
                        boxname = 'box2'
                    elif self.step == 13:
                        boxname = 'box3'

                    # Detach the gripper magnet
                    detached = self.detach_gripper(boxname)
                    if detached:
                        self.get_logger().info(f"Box {boxname} detached successfully.")
                        self.waiting_for_drop = False  # Reset the state variable
                        self.step += 1  # Move to the next configuration
                    else:
                        self.get_logger().info(f"Failed to detach box {boxname}.")
                else:
                    self.get_logger().info("Waiting for drop request...")
                    return  # Wait until drop request is received

            elif self.step in [1, 6, 11]:  # Steps where the arm should pick up the box
                if self.step == 1:
                    boxname = 'box1'
                elif self.step == 6:
                    boxname = 'box2'
                elif self.step == 11:
                    boxname = 'box3'

                # Attach the gripper magnet
                attached = self.attach_gripper(boxname)
                if attached:
                    self.get_logger().info(f"Box {boxname} attached successfully.")
                    self.step += 1  # Move to the next configuration
                else:
                    self.get_logger().info(f"Failed to attach box {boxname}.")
            else:
                self.step += 1  # Move to the next configuration
        else:
            # Calculate joint displacements (delta)
            delta_config = [target_config[i] - self.current_joint_positions[i] for i in range(len(target_config))]

            # Create JointJog message
            jog_msg = JointJog()
            jog_msg.header.stamp = self.get_clock().now().to_msg()  # Set current time
            jog_msg.header.frame_id = ''
            jog_msg.joint_names = self.joint_names
            jog_msg.velocities = [1.5 * delta for delta in delta_config]  # Adjust velocity for faster movement
            jog_msg.duration = 0.01  # Duration for each velocity command

            # Publish the message
            self.publisher_.publish(jog_msg)
            self.get_logger().info(f'Publishing JointJog: {jog_msg}')

    def subscribe_to_joint_states(self):
        """Subscribe to the /joint_states topic to get the current joint positions."""
        self.create_subscription(
            JointState,
            '/joint_states',
            self.get_current_joint_positions,
            10
        )

def main():
    rclpy.init()
    node = JointJogPublisher()
    node.subscribe_to_joint_states()  # Start subscribing to /joint_states

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
