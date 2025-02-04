import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion
from pick_place_msgs.srv import ObjectManipulation

class RosInterface():
    def __init__(self, node):
        self._node = node
        self._client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        self._goal_handle = None
        self.goal_callback = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self._node)

        self.pick_client = self._node.create_client(ObjectManipulation, '/pick_object')
        self.place_client = self._node.create_client(ObjectManipulation, '/place_object')

        self.timer = self._node.create_timer(3.0, self.print_current_position)


    def send_goal(self, pose, callback):
        self._node.get_logger().info('Waiting for action server...')
        self._client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self._node.get_logger().info('Sending goal request...')
        self.goal_callback = callback
        self._goal_handle = self._client.send_goal_async(goal_msg)
        self._goal_handle.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._node.get_logger().info('Goal rejected :(')
            return

        self._node.get_logger().info('Goal accepted :)')
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self._node.get_logger().info(f'Goal result: {result}')
        self.goal_callback(result)

    def pick_object(self, label, callback):
        request = ObjectManipulation.Request()
        request.object_label = label

        self._node.get_logger().info(f'Picking object: {label}')
        self.pick_client.wait_for_service()
        future = self.pick_client.call_async(request)
        future.add_done_callback(callback)

    def place_object(self, label, callback):
        request = ObjectManipulation.Request()
        request.object_label = label

        self._node.get_logger().info(f'Placing object: {label}')
        self.place_client.wait_for_service()
        future = self.place_client.call_async(request)
        future.add_done_callback(callback)

    def print_current_position(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('map', 'base_link', now)

            pos = trans.transform.translation
            rot = trans.transform.rotation
            _, _, yaw = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])

            self._node.get_logger().info(f'Current position - x: {pos.x}, y: {pos.y}, yaw: {yaw}')
        except Exception as e:
            self._node.get_logger().warn(f'Could not transform: {e}')