from copy import deepcopy
from typing import Optional, Tuple

from geometry_msgs.msg import TwistStamped
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.task import Future
from std_srvs.srv import Trigger
import math


class MoveIt2Servo:
    """
    Python interface for MoveIt 2 Servo that enables real-time control in Cartesian Space.
    This implementation is just a thin wrapper around TwistStamped message publisher.
    """

    def __init__(
        self,
        node: Node,
        frame_id: str,
        linear_speed: float = 1.0,
        angular_speed: float = 1.0,
        enable_at_init: bool = True,
        callback_group: Optional[CallbackGroup] = None,
    ):
        """
        Construct an instance of `MoveIt2Servo` interface.
        """
        self._node = node

        # Create publisher
        self.__twist_pub = self._node.create_publisher(
            msg_type=TwistStamped,
            topic="delta_twist_cmds",
            qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_ALL,
            ),
            callback_group=callback_group,
        )

        # Create service clients
        self.__start_service = self._node.create_client(
            srv_type=Trigger,
            srv_name="/servo_node/start_servo",
            callback_group=callback_group,
        )
        self.__stop_service = self._node.create_client(
            srv_type=Trigger,
            srv_name="/servo_node/stop_servo",
            callback_group=callback_group,
        )
        self.__trigger_req = Trigger.Request()
        self.__is_enabled = False

        # Initialize message based on passed arguments
        self.__twist_msg = TwistStamped()
        self.__twist_msg.header.frame_id = frame_id
        self.__twist_msg.twist.linear.x = linear_speed
        self.__twist_msg.twist.linear.y = linear_speed
        self.__twist_msg.twist.linear.z = linear_speed
        self.__twist_msg.twist.angular.x = angular_speed
        self.__twist_msg.twist.angular.y = angular_speed
        self.__twist_msg.twist.angular.z = angular_speed

    def __call__(
        self,
        linear: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        angular: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    ):
        """
        Callable that is identical to `MoveIt2Servo.servo()`.
        """
        self.servo(linear=linear, angular=angular)

    def servo(
        self,
        linear: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        angular: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        enable_if_disabled: bool = True,
    ):
        """
        Apply linear and angular twist using MoveIt 2 Servo.
        """
        if not self.is_enabled and enable_if_disabled:
            self.enable(sync=True)

        twist_msg = deepcopy(self.__twist_msg)
        twist_msg.header.stamp = self._node.get_clock().now().to_msg()
        twist_msg.twist.linear.x *= linear[0]
        twist_msg.twist.linear.y *= linear[1]
        twist_msg.twist.linear.z *= linear[2]
        twist_msg.twist.angular.x *= angular[0]
        twist_msg.twist.angular.y *= angular[1]
        twist_msg.twist.angular.z *= angular[2]
        self.__twist_pub.publish(twist_msg)

    def move_to_marker(self, target_position: Tuple[float, float, float], tolerance: float = 0.01):
        """
        Move the end-effector to the specified marker position.
        :param target_position: Target position as (x, y, z).
        :param tolerance: Distance tolerance to stop servoing.
        """
        current_position = [0.0, 0.0, 0.0]  # Mock current position; replace with actual state feedback.
        while True:
            # Calculate the distance to the target
            dx = target_position[0] - current_position[0]
            dy = target_position[1] - current_position[1]
            dz = target_position[2] - current_position[2]

            distance = math.sqrt(dx**2 + dy**2 + dz**2)
            if distance < tolerance:
                self._node.get_logger().info(f"Reached marker at {target_position}")
                self.stop()
                break

            # Normalize to get unit direction vector
            linear = (dx / distance, dy / distance, dz / distance)
            self.servo(linear=linear, angular=(0.0, 0.0, 0.0))

    def enable(self, wait_for_server_timeout_sec: Optional[float] = 1.0, sync: bool = False) -> bool:
        """
        Enable MoveIt 2 Servo server via async service call.
        """
        while not self.__start_service.wait_for_service(timeout_sec=wait_for_server_timeout_sec):
            self._node.get_logger().warn(
                f"Service '{self.__start_service.srv_name}' is not yet available..."
            )
            return False

        if sync:
            result = self.__start_service.call(self.__trigger_req)
            if not result.success:
                self._node.get_logger().error(
                    f"MoveIt Servo could not be enabled. ({result.message})"
                )
            self.__is_enabled = result.success
            return result.success
        else:
            start_service_future = self.__start_service.call_async(self.__trigger_req)
            start_service_future.add_done_callback(self.__enable_done_callback)
            return True

    def disable(self, wait_for_server_timeout_sec: Optional[float] = 1.0, sync: bool = False) -> bool:
        """
        Disable MoveIt 2 Servo server via async service call.
        """
        while not self.__stop_service.wait_for_service(timeout_sec=wait_for_server_timeout_sec):
            self._node.get_logger().warn(
                f"Service '{self.__stop_service.srv_name}' is not yet available..."
            )
            return False

        if sync:
            result = self.__stop_service.call(self.__trigger_req)
            if not result.success:
                self._node.get_logger().error(
                    f"MoveIt Servo could not be disabled. ({result.message})"
                )
            self.__is_enabled = not result.success
            return result.success
        else:
            stop_service_future = self.__stop_service.call_async(self.__trigger_req)
            stop_service_future.add_done_callback(self.__disable_done_callback)
            return True

    def stop(self):
        """
        Publish a zero-twist message to stop the robot.
        """
        self.servo(linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, 0.0))

    @property
    def is_enabled(self) -> bool:
        return self.__is_enabled
