#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
import time
import math
from payload_service.srv import PayloadSW
from ebot_docking.srv import DockSw  # Import custom service message

"""
Basic navigation demo to follow a given path after smoothing
"""

def quaternion_from_yaw(yaw):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return qz, qw

def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Set up the payload service client
    node = rclpy.create_node('payload_service_client')
    payload_req_cli = node.create_client(PayloadSW, '/payload_sw')

    # Wait for the service to be available
    while not payload_req_cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Payload service not available, waiting...')

    # Set up the docking service client
    docking_srv = node.create_client(DockSw, 'dock_control')

    # Wait for the docking service to be available
    while not docking_srv.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Docking service not available, waiting...')

    # Set our initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 1.84
    initial_pose.pose.position.y = -9.05
    initial_pose.pose.orientation.z, initial_pose.pose.orientation.w = quaternion_from_yaw(3.14)
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    # Define goal poses
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 0.20
    goal_pose.pose.position.y = -2.20
    yaw = 3.40
    goal_pose.pose.orientation.z, goal_pose.pose.orientation.w = quaternion_from_yaw(yaw)

    goal_pose4 = PoseStamped()
    goal_pose4.header.frame_id = 'map'
    goal_pose4.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose4.pose.position.x = 0.30   
    goal_pose4.pose.position.y = -2.60
    yaw = 2.90
    goal_pose4.pose.orientation.z, goal_pose4.pose.orientation.w = quaternion_from_yaw(yaw)

    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose2.pose.position.x = 2.30
    goal_pose2.pose.position.y = 2.55
    yaw_1 = -1.57
    goal_pose2.pose.orientation.z, goal_pose2.pose.orientation.w = quaternion_from_yaw(yaw_1)

    goal_pose3 = PoseStamped()
    goal_pose3.header.frame_id = 'map'
    goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose3.pose.position.x = -5.0  
    goal_pose3.pose.position.y = 2.89
    yaw_2 = -2.00
    goal_pose3.pose.orientation.z, goal_pose3.pose.orientation.w = quaternion_from_yaw(yaw_2)

    goal_poses = [goal_pose, goal_pose2, goal_pose4, goal_pose3]

    for index, goal_pose in enumerate(goal_poses):
        path = navigator.getPath(initial_pose, goal_pose)
        smoothed_path = navigator.smoothPath(path)

        # Follow path
        navigator.followPath(smoothed_path)

        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback:
                print(
                    'Estimated distance remaining to goal position: '
                    + '{0:.3f}'.format(feedback.distance_to_goal)
                    + '\nCurrent speed of the robot: '
                    + '{0:.3f}'.format(feedback.speed)
                )

        # Check result after each goal
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')

            # Trigger the payload service when reaching the first or third goal
            if index == 0 or index == 2:
                req = PayloadSW.Request()
                req.receive = True     
                req.drop = False
                future = payload_req_cli.call_async(req)
                rclpy.spin_until_future_complete(node, future)
                print('Payload service called to receive Boxes.')
                time.sleep(1)

            # Trigger the docking service at the second goal pose
            if index == 1:
                node.get_logger().info("Reaching goal_pose2, preparing to call docking service...")

                # Create and send the docking service request for Conveyor Belt 2
                docking_req = DockSw.Request()
                docking_req.startcmd = True
                docking_req.undocking = False
                docking_req.linear_dock = True
                docking_req.orientation_dock = True
                docking_req.distance = 0.5
                docking_req.orientation = -1.57
                docking_req.rack_no = "Conveyor Belt 2"

                docking_future = docking_srv.call_async(docking_req)
                rclpy.spin_until_future_complete(node, docking_future)

                if docking_future.result() is not None:
                    response = docking_future.result()
                    print("Docking service response: Success =", response.success)
                    print("Message from service:", response.message)

                    req = PayloadSW.Request()
                    req.receive = False     
                    req.drop = True
                    future = payload_req_cli.call_async(req)
                    rclpy.spin_until_future_complete(node, future)
                    print('Payload service called to drop Boxes.')
                    time.sleep(1)
                    
                    if response.success:
                        print("Docking succeeded. Holding position...")
                        time.sleep(1)
                    else:
                        print("Docking failed. Robot will hold position at the conveyor belt.")
                        break

            # Trigger the docking service at the fourth goal pose (Conveyor Belt 1)
            if index == 3:
                node.get_logger().info("Reaching goal_pose4, preparing to call docking service...")

                # Create and send the docking service request for Conveyor Belt 1
                docking_req = DockSw.Request()
                docking_req.startcmd = True
                docking_req.undocking = False
                docking_req.linear_dock = True
                docking_req.orientation_dock = True
                docking_req.distance = 0.5
                docking_req.orientation = -1.45
                docking_req.rack_no = "Conveyor Belt 1"

                docking_future = docking_srv.call_async(docking_req)
                rclpy.spin_until_future_complete(node, docking_future)

                if docking_future.result() is not None:
                    response = docking_future.result()
                    print("Docking service response: Success =", response.success)
                    print("Message from service:", response.message)

                    req = PayloadSW.Request()
                    req.receive = False     
                    req.drop = True
                    future = payload_req_cli.call_async(req)
                    rclpy.spin_until_future_complete(node, future)
                    print('Payload service called to drop Boxes.')
                    time.sleep(2)
                    
                    if response.success:
                        print("Docking succeeded. Holding position...")
                        time.sleep(1)
                    else:
                        print("Docking failed. Robot will hold position at the conveyor belt.")
                        break

        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
            break
        elif result == TaskResult.FAILED:
            print('Goal failed!')
            break
        else:
            print('Goal has an invalid return status!')

    navigator.lifecycleShutdown()
    node.destroy_node()
    rclpy.shutdown()
    exit(0)

if __name__ == '__main__':
    main()
