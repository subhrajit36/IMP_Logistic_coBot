#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
import time
import math
from payload_service.srv import PayloadSW
from ebot_docking.srv import DockSw  # Import custom service message
from payload_service.srv import PassingSRV
from nav_msgs.msg import Path

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

    # Set up the passing service client
    passing_srv_client = node.create_client(PassingSRV, '/passing_srv')

    # Wait for the passing service to be available
    while not passing_srv_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Passing service not available, waiting...')

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
    goal_pose_home = PoseStamped()
    goal_pose_home.header.frame_id = 'map'
    goal_pose_home.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose_home.pose.position.x = 0.75
    goal_pose_home.pose.position.y = -2.50
    yaw = -3.05
    goal_pose_home.pose.orientation.z, goal_pose_home.pose.orientation.w = quaternion_from_yaw(yaw)

    goal_pose_inter = PoseStamped()
    goal_pose_inter.header.frame_id = 'map'
    goal_pose_inter.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose_inter.pose.position.x = 0.70
    goal_pose_inter.pose.position.y = -2.25
    yaw = -3.05
    goal_pose_inter.pose.orientation.z, goal_pose_inter.pose.orientation.w = quaternion_from_yaw(yaw)

    goal_pose_inter2 = PoseStamped()
    goal_pose_inter2.header.frame_id = 'map'
    goal_pose_inter2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose_inter2.pose.position.x = 0.70
    goal_pose_inter2.pose.position.y = -2.35
    yaw = -3.05
    goal_pose_inter2.pose.orientation.z, goal_pose_inter2.pose.orientation.w = quaternion_from_yaw(yaw)

    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose2.pose.position.x = 2.38
    goal_pose2.pose.position.y = 2.55
    yaw_1 = -1.57
    goal_pose2.pose.orientation.z, goal_pose2.pose.orientation.w = quaternion_from_yaw(yaw_1)

    goal_pose3 = PoseStamped()
    goal_pose3.header.frame_id = 'map'
    goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose3.pose.position.x = -4.60
    goal_pose3.pose.position.y = 2.89
    yaw_2 = -1.57
    goal_pose3.pose.orientation.z, goal_pose3.pose.orientation.w = quaternion_from_yaw(yaw_2)

    goal_poses = [goal_pose_home, goal_pose2, goal_pose_inter, goal_pose3, goal_pose_inter2, goal_pose2]

    for index, goal_pose in enumerate(goal_poses):
        # Get the path from the navigator
        path = navigator.getPath(initial_pose, goal_pose)

        # Ensure the path is of type nav_msgs.msg.Path
        if not isinstance(path, Path):
            print("Error: Path is not of type nav_msgs.msg.Path")
            continue

        # Smooth the path
        smoothed_path = navigator.smoothPath(path)

        # Follow the smoothed path
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

            node.get_logger().info("Reaching goal_pose2, preparing to call docking service...")
            # Create and send the docking service request for Conveyor Belt 2
            docking_req = DockSw.Request()
            docking_req.startcmd = True
            docking_req.undocking = False
            docking_req.linear_dock = True
            docking_req.orientation_dock = True
            docking_req.distance = 0.05
            if index in [1, 5]:
                docking_req.orientation = -1.57
            elif index == 3:
                docking_req.orientation = -1.35
            else:
                docking_req.orientation = 3.14
            docking_req.rack_no = "Conveyor Belt 2"

            docking_future = docking_srv.call_async(docking_req)
            rclpy.spin_until_future_complete(node, docking_future)

            if docking_future.result() is not None:
                response = docking_future.result()
                print("Docking service response: Success =", response.success)
                print("Message from service:", response.message)
                print(index)

                if index % 2 != 0:
                    print(index)
                    if index == 1:
                        print('OOOOOOOOOOOOOOOOOOOOOOOO')
                        boxname = "box1"
                    if index == 3:
                        boxname = "box2"
                    if index == 5:
                        boxname = "box3"

                    req = PayloadSW.Request()
                    req.receive = False
                    req.drop = True
                    req.box_name = boxname
                    future = payload_req_cli.call_async(req)
                    rclpy.spin_until_future_complete(node, future)
                    print('Payload service called to drop Boxes.')
                    # time.sleep(20)
                else:
                    # Send the PassingSRV request
                    passing_req = PassingSRV.Request()
                    print("SUBHRAJIT NEED BOX")
                    passing_req.drop = True
                    print("SUBHRAJIT passing_req.drop")
                    print(passing_req.drop)
                    passing_future = passing_srv_client.call_async(passing_req)
                    rclpy.spin_until_future_complete(node, passing_future)

                    if passing_future.result() is not None:
                        passing_response = passing_future.result()
                        print("Passing service response: Success =", passing_response.success)
                        print("Message from Passing service:", passing_response.message)
                        if passing_response.success:
                            print("Arm operation completed successfully.")
                        else:
                            print("Arm operation failed:", passing_response.message)
                    else:
                        print("Failed to get response from Passing service.")

                if response.success:
                    print("Docking succeeded. Holding position...")
                    if index % 2 != 0:
                        time.sleep(1)
                    else:
                        time.sleep(2)
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