#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import random

def create_goal_pose(x, y, qx, qy, qz, qw):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose

# List your goal poses here. (Update with your actual poses if needed.)
# Inside the lab (AHG 2.202)
# GOAL_POSES = [
#     create_goal_pose(0.1205, 10.4845, 0.0, 0.0, -0.7887, 0.6147),
#     create_goal_pose(-0.8106, 0.6233, 0.0, 0.0, 0.7591, 0.6510)
# ]

# Outside the lab
GOAL_POSES = [
    create_goal_pose(-54.5899, 21.1945, 0.0, 0.0, 0.07484, 0.9972),
    create_goal_pose(-54.5083, 6.1306, 0.0, 0.0, 0.08377, 0.9965),
    create_goal_pose(-61.6670, -10.2687, 0.0, 0.0, 0.64469, 0.7641),
    create_goal_pose(-31.1756, -8.5680, 0.0, 0.0, 0.72331, 0.6900),
    create_goal_pose(-4.9667, -8.5355, 0.0, 0.0, 0.99919, 0.0400),
    create_goal_pose(-7.8076, 20.9514, 0.0, 0.0, 0.99136, 0.1310)
    # create_goal_pose(0.12054237858048442, 10.484498713552897, 0.0, 0.0, -0.7887430095044403, 0.6147230798968576),
    # create_goal_pose(-0.8106196604177645, 0.6232977408179624, 0.0, 0.0, 0.7590877024865701, 0.6509883715809833)
]

class DoorToDoorNavigator(Node):
    def __init__(self):
        super().__init__('door_to_door_navigator')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info("Waiting for 'navigate_to_pose' action server...")
        self._action_client.wait_for_server()
        self.get_logger().info("Action server available. Starting navigation loop.")

        self.last_goal = None
        self.goal_in_progress = False

        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        if self.goal_in_progress:
            return
        available_goals = [g for g in GOAL_POSES if g != self.last_goal]
        if not available_goals:
            self.get_logger().warn("No available goals different from the last one!")
            return

        next_goal = random.choice(available_goals)
        self.send_goal(next_goal)

    def send_goal(self, goal_pose: PoseStamped):
        self.goal_in_progress = True
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.get_logger().info(
            f"Sending goal: position=({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})"
        )
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(
            lambda future: self.goal_response_callback(future, goal_pose)
        )

    def goal_response_callback(self, future, goal_pose):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal was rejected!")
            self.goal_in_progress = False
            return

        self.get_logger().info("Goal accepted, waiting for result...")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(
            lambda future: self.get_result_callback(future, goal_pose)
        )

    def get_result_callback(self, future, goal_pose):
        self.get_logger().info("Reached goal!")
        self.last_goal = goal_pose
        wait_timer = self.create_timer(5.0, lambda: self.reset_goal_in_progress(wait_timer))

    def reset_goal_in_progress(self, timer):
        timer.cancel()
        self.goal_in_progress = False

def main(args=None):
    rclpy.init(args=args)
    navigator = DoorToDoorNavigator()
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info("Shutting down door_to_door_navigator node...")
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
