import rclpy
from rclpy.action import ActionServer
import rclpy.action
from rclpy.node import Node
from control_msgs.action import GripperCommand
from .robotiq_gripper import RobotiqGripper
from sensor_msgs.msg import JointState
import time

class GripperController(Node):

    def __init__(self):
        super().__init__('gripper_controller')
        self.declare_parameter('robot_ip', '10.66.171.2')
        self.declare_parameter('joint_name', 'finger_joint')
        self.declare_parameter('publish_frequency', 5.0)
        port = 63352

        ip = self.get_parameter('robot_ip').get_parameter_value().string_value

        self.gripper = RobotiqGripper()
        self.gripper.connect(ip, port)
        self.gripper.activate()
        self.has_move_command = False
        self.current_goal = None
        self.current_position = 0

        self.action_server = ActionServer(
            self,
            GripperCommand,
            'gripper_controller/gripper_action',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
        )

        self.joint_name = self.get_parameter('joint_name').get_parameter_value().string_value
        self.publish_frequency = self.get_parameter('publish_frequency').get_parameter_value().double_value
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(1.0 / self.publish_frequency, self.publish_joint_state)

        self.get_logger().info("Gripper ready to recieve commands")

    def goal_callback(self, goal_handle):
        # Accept the incoming goal
        self.get_logger().info(f'Accepted goal request')
        return rclpy.action.GoalResponse.ACCEPT

    def execute_callback(self, goal_handle):
        # Execute the goal
        result = GripperCommand.Result()

        command = goal_handle.request.command
        position = (command.position / 0.65) * 255
        max_effort = command.max_effort * 255

        if max_effort == 0:
            max_effort = 25.0

        self.get_logger().info(f'Setting Position: {position} Max Effort: {max_effort}')

        self.current_goal = (position, max_effort)
        self.has_move_command = True

        goal_handle.succeed()

        # Set result and feedback
        result.position = position
        result.effort = max_effort

        # Return result
        return result
    
    def publish_joint_state(self):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = [self.joint_name]
        joint_state.position = [float(self.gripper.get_current_position()) / self.gripper.get_max_position() * 0.65]  # Example conversion
        joint_state.effort = [1.0]  # Update with actual effort if available
        joint_state.velocity = [0.0]  # Update with actual velocity if available

        self.publisher.publish(joint_state)


def main(args=None):
    rclpy.init(args=args)
    gripper_controller = GripperController()
    while rclpy.ok():
        rclpy.spin_once(gripper_controller)
        
        # socket communication has to be on main thread for some reason?
        if gripper_controller.has_move_command:
            gripper_controller.gripper.move_and_wait_for_pos(int(gripper_controller.current_goal[0]), 255, int(gripper_controller.current_goal[1]))
            gripper_controller.has_move_command = False

        gripper_controller.current_position = gripper_controller.gripper.get_current_position()

        time.sleep(0.1)

    gripper_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
