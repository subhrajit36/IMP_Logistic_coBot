#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range, Imu
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion
from ebot_docking.srv import DockSw
import math

class MyRobotDockingController(Node):
    def __init__(self):
        super().__init__('my_robot_docking_controller')
        
        # Create callback group
        self.callback_group = ReentrantCallbackGroup()
        
        # Create subscribers
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
        self.ultrasonic_rl_sub = self.create_subscription(Range, '/ultrasonic_rl/scan', self.ultrasonic_rl_callback, 10)
        self.ultrasonic_rr_sub = self.create_subscription(Range, '/ultrasonic_rr/scan', self.ultrasonic_rr_callback, 10)
        
        # Create publisher
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Create service
        self.dock_control_srv = self.create_service(
            DockSw, 
            'dock_control', 
            self.dock_control_callback, 
            callback_group=self.callback_group
        )
        
        # Initialize parameters
        self.robot_pose = [0.0, 0.0, 0.0]  # [x, y, yaw]
        self.usrleft_value = 0.0
        self.usrright_value = 0.0
        self.is_docking = False
        self.dock_aligned = False
        self.target_orientation = 0.0
        self.target_distance = 0.5  # Default target distance in meters
        
        # P-controller gains
        self.Kp_angular = 0.5  # Proportional gain for angular control
        self.Kp_linear = -4.0   # Proportional gain for linear control
        
        # Create control loop timer (10Hz)
        self.controller_timer = self.create_timer(0.1, self.controller_loop)

    def odometry_callback(self, msg):
        # Update robot pose from odometry
        self.robot_pose[0] = msg.pose.pose.position.x
        self.robot_pose[1] = msg.pose.pose.position.y
        
        # Convert quaternion to Euler angles
        orientation = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.robot_pose[2] = yaw

    def ultrasonic_rl_callback(self, msg):
        self.usrleft_value = msg.range

    def ultrasonic_rr_callback(self, msg):
        self.usrright_value = msg.range

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def controller_loop(self):
        if not self.is_docking:
            return

        # Create Twist message for robot control
        cmd_vel = Twist()

        # Calculate orientation error
        orientation_error = self.normalize_angle(self.target_orientation - self.robot_pose[2] - 0.1)  # Adding more degrees for better proficiency
        
        # Calculate distance error using ultrasonic sensors
        current_distance = (self.usrleft_value + self.usrright_value) / 2.0
        distance_error = current_distance - self.target_distance
        
        # Check if orientation is aligned first
        if abs(orientation_error) > 0.05:  # ~3 degrees tolerance
            # Apply P-control for angular correction
            cmd_vel.angular.z = self.Kp_angular * orientation_error
        else:
            # Adjust linear speed for fine-tuned docking
            cmd_vel.linear.x = self.Kp_linear * distance_error  # Move slowly to avoid bumping into the rack

            # Stop once we're close enough to the rack
            if current_distance <= 0.5:  # Less than 10cm from the rack
                self.dock_aligned = True
                self.is_docking = False
                cmd_vel.linear.x = 0.0  # Stop the robot
                cmd_vel.angular.z = 0.0  # Stop rotation
                self.get_logger().info("Docking completed and stuck to the rack!")

        # Publish velocity commands
        self.cmd_vel_pub.publish(cmd_vel)

    def dock_control_callback(self, request, response):
        # Reset flags
        self.is_docking = True
        self.dock_aligned = False
        
        # Set target parameters from request
        if request.orientation_dock:
            self.target_orientation = request.orientation
        if request.linear_dock and request.distance > 0:
            self.target_distance = request.distance
            
        self.get_logger().info("Docking initiated!")
        
        # Create a rate object for the service response
        rate = self.create_rate(2, self.get_clock())
        
        # Wait for alignment
        while not self.dock_aligned and self.is_docking:
            self.get_logger().info(f"Aligning... Left: {self.usrleft_value:.2f}m, Right: {self.usrright_value:.2f}m")
            rate.sleep()
        
        response.success = True
        response.message = "Docking completed successfully"
        return response

def main(args=None):
    rclpy.init(args=args)
    
    docking_controller = MyRobotDockingController()
    
    executor = MultiThreadedExecutor()
    executor.add_node(docking_controller)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        docking_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()