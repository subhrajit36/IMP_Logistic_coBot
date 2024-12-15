#!/usr/bin/env python3
"""
Example of moving the arm to ArUco marker positions using Servo with controlled speed and timestamp.
ros2 run pymoveit2 ex_move_to_marker.py
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import TwistStamped, Point
from pymoveit2.robots import ur5
from std_srvs.srv import Trigger

# Initialize a TwistStamped message
twist_msg = TwistStamped()
twist_msg.header.frame_id = 'base_link'
twist_msg.twist.linear.x = 0.0
twist_msg.twist.linear.y = 0.0
twist_msg.twist.linear.z = 0.0
twist_msg.twist.angular.x = 0.0
twist_msg.twist.angular.y = 0.0
twist_msg.twist.angular.z = 0.0

class MoveToMarker(Node):
    def __init__(self):
        super().__init__('move_to_marker')

        # Service client to start the servo
        self.servo_client = self.create_client(Trigger, '/servo_node/start_servo')
        self.wait_for_service(self.servo_client, '/servo_node/start_servo')

        # Publisher for delta_twist_cmds
        self.twist_pub = self.create_publisher(
            TwistStamped,
            '/servo_node/delta_twist_cmds',
            QoSProfile(depth=10)
        )

        # Subscriber to marker_coordinates topic
        self.marker_sub = self.create_subscription(
            Point,
            'marker_coordinates',
            self.marker_callback,
            10
        )

        # Current target marker position
        self.target_position = None

        # Timer to send servo commands
        self.timer = self.create_timer(0.1, self.send_servo_command)

        # Speed control
        self.MAX_SPEED = 0.1 * 10  # Maximum speed (m/s)
        self.current_speed = 0.0  # Initialize current speed

        # Start the servo
        self.start_servo()

    def wait_for_service(self, client, service_name):
        """Wait for a service to become available."""
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"Waiting for {service_name} service...")

    def start_servo(self):
        """Start the servo service."""
        request = Trigger.Request()
        future = self.servo_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info(f"Servo started: {future.result().message}")
        else:
            self.get_logger().warn(f"Failed to start servo: {future.result().message}")

    def marker_callback(self, msg):
        """Callback for marker_coordinates topic."""
        self.target_position = msg
        self.get_logger().info(f"Received marker position: x={msg.x}, y={msg.y}, z={msg.z}")

    def send_servo_command(self):
        """Send servo commands to move towards the target marker."""
        if self.target_position is None:
            return

        # Calculate the current speed
        raw_speed = math.sqrt(
            self.target_position.x**2 +
            self.target_position.y**2 +
            self.target_position.z**2
        )
        self.current_speed = min(raw_speed, self.MAX_SPEED)

        # Scale the velocities to respect MAX_SPEED
        if raw_speed > self.MAX_SPEED:
            scale = self.MAX_SPEED / raw_speed
            twist_msg.twist.linear.x = self.target_position.x * scale
            twist_msg.twist.linear.y = self.target_position.y * scale
            twist_msg.twist.linear.z = self.target_position.z * scale
        else:
            twist_msg.twist.linear.x = self.target_position.x
            twist_msg.twist.linear.y = self.target_position.y
            twist_msg.twist.linear.z = self.target_position.z

        # Add timestamp
        twist_msg.header.stamp = self.get_clock().now().to_msg()

        # Publish the Twist message
        self.twist_pub.publish(twist_msg)

        # Log the speed and timestamp
        self.get_logger().info(
            f"Publishing Twist: linear_x={twist_msg.twist.linear.x}, "
            f"linear_y={twist_msg.twist.linear.y}, linear_z={twist_msg.twist.linear.z}, "
            f"current_speed={self.current_speed:.2f} m/s, timestamp={twist_msg.header.stamp.sec}.{twist_msg.header.stamp.nanosec}"
        )


def main():
    rclpy.init()
    node = MoveToMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
