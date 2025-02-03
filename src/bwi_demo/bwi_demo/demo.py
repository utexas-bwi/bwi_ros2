import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from .ros_interface import RosInterface
import yaml
import os
from ament_index_python.packages import get_package_share_directory
import time
from scipy.spatial.transform import Rotation as R

class DemoNode(Node):
    def __init__(self):
        super().__init__('demo_node')
        self.get_logger().info('Demo node initialized')

        self.target_object = "cracker box"
        self.landmarks = self.load_landmarks()
        self.ros_interface = RosInterface(self)
        self.current_landmark = "table1"
        self.holding_object = False

    def load_landmarks(self):
        self.get_logger().info('Loading landmarks...')
        package_share_directory = get_package_share_directory('bwi_demo')
        print(package_share_directory)
        filepath = os.path.join(package_share_directory, 'data', "landmarks.yaml")
        landmarks = None
        with open(filepath, 'r') as file:
            landmarks = yaml.safe_load(file)
        return landmarks
    
    def go_to_landmark(self, landmark_name, callback=None):
        if landmark_name not in self.landmarks:
            self.get_logger().warn(f'Landmark "{landmark_name}" not found')
            return

        landmark = self.landmarks[landmark_name]

        rotation = R.from_euler('xyz', [0, 0, landmark['yaw']])
        quaternion = rotation.as_quat()

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = landmark['x']
        pose.pose.position.y = landmark['y']
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]


        self.ros_interface.send_goal(pose, callback) 

    def manipulate_callback(self, future):
        result = future.result()
        self.get_logger().info('Manipulation result: %s' % result)
        if not result.success:
            self.get_logger().info("Failed manipulation, trying again!")
            self.destination_reached(None)
            return
        
        self.holding_object = not self.holding_object

        time.sleep(3)

        if not self.holding_object:
            self.ros_interface.pick_object(self.target_object, self.manipulate_callback)
        else:
            self.goto_next_landmark()

    def goto_next_landmark(self):
        if self.current_landmark == 'low_table1':
            self.current_landmark = 'low_table2'
        else:
            self.current_landmark = 'low_table1'
        self.go_to_landmark(self.current_landmark, self.destination_reached)

    def destination_reached(self, result):
        time.sleep(3)

        if not self.holding_object:
            self.ros_interface.pick_object(self.target_object, self.manipulate_callback)
        else:
            self.ros_interface.place_object('surface', self.manipulate_callback)
    
    def run_demo(self):
        self.current_landmark = 'low_table1'
        self.holding_object = False
        self.go_to_landmark(self.current_landmark, self.destination_reached)

    
def main(args=None):
    rclpy.init(args=args)
    node = DemoNode()
    #node.run_demo()
    rclpy.spin(node)
    rclpy.shutdown()