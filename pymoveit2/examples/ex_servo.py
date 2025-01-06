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
from rclpy.qos import QoSProfile
from math import isclose
import time


class JointJogPublisher(Node):
    def __init__(self):
        super().__init__('joint_jog_publisher')
        self.publisher_ = self.create_publisher(
            JointJog, '/servo_node/delta_joint_cmds', QoSProfile(depth=10)
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
        
        self.configurations = [
            [-0.03, -2.15, 1.50, -2.49, -1.54, 3.14],  # 1st
            [-0.02, -1.80, 0.66, -0.42, -1.56, 3.12],  # 2nd
            [1.87, -1.26, -1.48, -1.98, -4.69, -2.76],  # 3rd
            [1.65, -1.87, -2.08, -0.78, -4.69, -2.99],  # 4th (final)
            [0.00, -1.33, 2.33, -2.55, -1.56, 3.14],   # New config 1 (after magnet on)
            [-0.17, -0.52, 1.35, -2.38, -1.56, 2.97]   # New config 2 (after magnet on)
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
        self.get_logger().info("PassingSRV service ready.")
    
    def get_current_joint_positions(self, msg):
        """Callback to update current joint positions from the /joint_states topic."""
        self.current_joint_positions = msg.position

    def check_target_reached(self, target_config):
        """Check if the arm has reached the target configuration."""
        for i in range(len(target_config)):
            if not isclose(self.current_joint_positions[i], target_config[i], abs_tol=self.tolerance):
                return False
        return True

    def attach_gripper(self):
        """Attach the gripper magnet after reaching the target pose."""
        # print('uihui')
        req = AttachLink.Request()
        # print('uihuiiiiiiiiii')
        req.model1_name = 'box2'  # Replace with the name of the box you're attaching
        req.link1_name = 'link'   # Name of the gripper's link
        req.model2_name = 'ur5'   # Robot name (or your robot's URDF name)
        req.link2_name = 'wrist_3_link'  # Name of the gripper's link on the robot

        # Call the service asynchronously
        future = self.gripper_control_1.call_async(req)
        # print('uihui_2')
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        # print('uihui_3')
        
        if future.result() is None :
            self.get_logger().info("Gripper Magnet ON - Object Attached")
            return True  # Return success for gripper attachment
        else:
            self.get_logger().info(f"Failed to attach object: {future.result().message}")
            return False  # Return failure if attachment fails
        
    def detach_gripper(self):
        """Call the DetachLink service to turn off the gripper magnet."""
        req = DetachLink.Request()
        req.model1_name = 'box2'  # Replace with the name of the box you're detaching
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

        
    def handle_passing_service(self):
        """Handle PassingSRV requests to detach the gripper magnet."""
        req = PassingSRV.Request()
        response = PassingSRV.Response()
        req.drop = True
        if req.drop:
            self.get_logger().info("Passing service triggered to drop the object.")

            # Attempt to detach the gripper magnet
            detached = self.detach_gripper()
            if detached:
                response.success = True
                response.message = "Object successfully detached."
            else:
                response.success = False
                response.message = "Failed to detach object."
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
            jog_msg.velocities = [4.5 * delta for delta in delta_config]  # Adjust velocity for faster movement
            jog_msg.duration = 0.01  # Duration for each velocity command
            
            # Publish the message
            self.publisher_.publish(jog_msg)
            self.get_logger().info(f'Publishing JointJog: {jog_msg}')
    
    def timer_callback(self):
        if self.stop_publishing:
            self.get_logger().info('Target reached. Stopping.')
            time.sleep(3)
            self.handle_passing_service()
            return  # Stop the timer once the target is reached
        

        if self.step >= 6:
            self.get_logger().info('All configurations executed, stopping.')
            self.stop_publishing = True
            return  # Stop the timer once all configurations are processed
        
        target_config = self.configurations[self.step]
        
        if self.check_target_reached(target_config):
            self.get_logger().info(f"Target configuration {self.step + 1} reached.")
            print(self.step)
            
            if self.step == 3:  # After reaching the 4th config
                attached = self.attach_gripper()  # Attach the gripper magnet after reaching the 4th pose
                if attached is True:
                    self.step += 1  # Move to the next configuration only after successful attachment
                return  # Allow the next configuration to be processed

            self.move_to_configuration(target_config)  # Move to the next configuration
            self.step += 1  # Increment to the next configuration                
            return  # Stop publishing velocities once target is reached
        
        # Calculate joint displacements (delta)
        delta_config = [target_config[i] - self.current_joint_positions[i] for i in range(len(target_config))]

        # Create JointJog message
        jog_msg = JointJog()
        jog_msg.header.stamp = self.get_clock().now().to_msg()  # Set current time
        jog_msg.header.frame_id = ''
        jog_msg.joint_names = self.joint_names
        jog_msg.velocities = [4.5 * delta for delta in delta_config]  # Adjust velocity for faster movement
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
