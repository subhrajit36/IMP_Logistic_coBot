#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
import time
import math

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

    # Set our initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 1.84
    initial_pose.pose.position.y = -9.05
    initial_pose.pose.orientation.z = 0.1
    initial_pose.pose.orientation.w = 3.14
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    # Define goal poses
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = -0.12
    goal_pose.pose.position.y = -2.35
    yaw = 3.40
    goal_pose.pose.orientation.z, goal_pose.pose.orientation.w = quaternion_from_yaw(yaw)
    # goal_pose.pose.orientation.w = 2.78

    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose2.pose.position.x = 1.86
    goal_pose2.pose.position.y = 2.56
    goal_pose2.pose.orientation.w = 0.97

    goal_pose3 = PoseStamped()
    goal_pose3.header.frame_id = 'map'
    goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose3.pose.position.x = -3.84  
    goal_pose3.pose.position.y = 2.64
    goal_pose3.pose.orientation.w = 2.78

    goal_poses = [goal_pose, goal_pose2, goal_pose3]

 
    for goal_pose in goal_poses:
        path = navigator.getPath(initial_pose, goal_pose)
        smoothed_path = navigator.smoothPath(path)

        # Follow path
        navigator.followPath(smoothed_path)

        i = 0
        while not navigator.isTaskComplete():
            i += 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print(
                    'Estimated distance remaining to goal position: '
                    + '{0:.3f}'.format(feedback.distance_to_goal)
                    + '\nCurrent speed of the robot: '
                    + '{0:.3f}'.format(feedback.speed)
                )
        print(goal_pose.pose.position.x)
        print(goal_pose.pose.position.y)
        print(goal_pose.pose.orientation.w)    

        # Check result after each goal
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
            time.sleep(3)
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
            break
        elif result == TaskResult.FAILED:
            print('Goal failed!')
            break
        else:
            print('Goal has an invalid return status!')


    navigator.lifecycleShutdown()
    exit(0)


if __name__ == '__main__':
    main()
