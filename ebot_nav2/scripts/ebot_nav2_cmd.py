#!/usr/bin/env python3

'''import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
import math

class EBotNav2Commander(Node):
    def __init__(self):
        super().__init__('ebot_nav2_commander')
        self.nav2_client = self.create_client(NavigateToPose, 'navigate_to_pose')
        while not self.nav2_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Navigate To Pose service not available, waiting...')

    def send_goal(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = pose[0]
        goal_msg.pose.pose.position.y = pose[1]

        # Convert yaw to quaternion
        quaternion = self.yaw_to_quaternion(pose[2])
        goal_msg.pose.pose.orientation = quaternion

        self.send_goal_request(goal_msg)

    def send_goal_request(self, goal_msg):
        self.get_logger().info('Sending goal request...')
        self.nav2_client.wait_for_service()
        self.nav2_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

    def feedback_callback(self, feedback_msg):
        # Handle feedback messages if needed
        pass

    def yaw_to_quaternion(self, yaw):
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = math.sin(yaw / 2.0)
        quaternion.w = math.cos(yaw / 2.0)
        return quaternion

def main():
    rclpy.init()
    ebot_nav2_commander = EBotNav2Commander()

    # List of poses to navigate to
    poses = [
        [1.8, 1.5, 1.57],
        [2.0, -7.0, -1.57],
        [-3.0, 2.5, 1.57]
    ]

    for pose in poses:
        ebot_nav2_commander.send_goal(pose)
        rclpy.spin_until_future_complete(ebot_nav2_commander.nav2_client.result_future)

        if ebot_nav2_commander.nav2_client.result_future.result.status == GoalStatus.STATUS_SUCCEEDED:
            ebot_nav2_commander.get_logger().info('Navigation to pose succeeded!')
        else:
            ebot_nav2_commander.get_logger().warning('Navigation to pose failed!')

    ebot_nav2_commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()'''







import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from nav2_msgs.action import NavigateToPose
import action_msgs.msg


class Nav2Commander(Node):
    def __init__(self):
        super().__init__('nav2_commander')
        self.nav2_action_client = self.create_client(NavigateToPose, 'NavigateToPose')

        while not self.nav2_action_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('NavigateToPose service not available, waiting...')
        self.req = NavigateToPose.Goal()

    def send_goal(self, pose):
        self.req.pose = pose
        self.future = self.nav2_action_client.call_async(self.req)

    def wait_for_result(self):
        while not self.future.done():
            rclpy.spin_once(self)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    nav2_commander = Nav2Commander()

    poses = [
        PoseStamped(pose=Pose(position=Point(x=1.8, y=1.5), orientation=Quaternion(z=1.57))),
        PoseStamped(pose=Pose(position=Point(x=2.0, y=-7.0), orientation=Quaternion(z=-1.57))),
        PoseStamped(pose=Pose(position=Point(x=-3.0, y=2.5), orientation=Quaternion(z=1.57))),
    ]

    for pose in poses:
        nav2_commander.send_goal(pose)
        nav2_commander.get_logger().info(f'Navigating to pose: {pose.pose}')
        result = nav2_commander.wait_for_result()

        if result.code != action_msgs.msg.GoalStatus.STATUS_SUCCEEDED:
            nav2_commander.get_logger().info(f'Failed to reach pose: {pose.pose}')
        else:
            nav2_commander.get_logger().info(f'Successfully reached pose: {pose.pose}')

    nav2_commander.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




'''import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav2_msgs.action import NavigateToPose
import action_msgs.msg
import sys


class Nav2Commander(Node):
    def __init__(self):
        super().__init__('nav2_commander')
        self.nav2_action_client = self.create_client(NavigateToPose, 'NavigateToPose')

        while not self.nav2_action_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('NavigateToPose service not available, waiting...')

    def send_goal(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.future = self.nav2_action_client.call_async(goal_msg)

    def wait_for_result(self):
        while not self.future.done():
            rclpy.spin_once(self)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    nav2_commander = Nav2Commander()

    # Define the goal pose [x, y, yaw]
    goal_pose = PoseStamped()
    goal_pose.pose.position.x = float(input("Enter X coordinate: "))
    goal_pose.pose.position.y = float(input("Enter Y coordinate: "))
    goal_pose.pose.orientation = Quaternion(z=float(input("Enter yaw (in radians): ")))

    nav2_commander.send_goal(goal_pose.pose)
    print(f'Navigating to pose: {goal_pose.pose}')

    result = nav2_commander.wait_for_result()

    if result.code != action_msgs.msg.GoalStatus.STATUS_SUCCEEDED:
        print(f'Failed to reach pose: {goal_pose.pose}')
    else:
        print(f'Successfully reached pose: {goal_pose.pose}')

    nav2_commander.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()'''
