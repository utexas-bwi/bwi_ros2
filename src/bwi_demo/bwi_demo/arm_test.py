import rclpy
from rclpy.node import Node
from pick_place_msgs.srv import ObjectManipulation, EnableSceneCloud
from std_srvs.srv import Empty
from std_msgs.msg import Float32
import threading
import time

class ArmTestNode(Node):
    def __init__(self):
        super().__init__('arm_test_node')
        self.pick_client = self.create_client(ObjectManipulation, '/pick_object')
        self.place_client = self.create_client(ObjectManipulation, '/place_object')
        self.enable_octomap_client = self.create_client(EnableSceneCloud, '/pick_place/toggle_scene_cloud')
        self.clear_octomap_client = self.create_client(Empty, '/clear_octomap')

    def call_pick_service(self, obj):
        request = ObjectManipulation.Request()
        request.object_label = obj
        future = self.pick_client.call_async(request)
        future.add_done_callback(self.pick_response_callback)

    def call_place_service(self, surface):
        request = ObjectManipulation.Request()
        request.object_label = surface
        request.surface_height = Float32()
        request.surface_height.data = -0.08
        future = self.place_client.call_async(request)
        future.add_done_callback(self.place_response_callback)

    def pick_response_callback(self, future):
        result = future.result()
        self.get_logger().info(f"Pick service result: {result}")
        #time.sleep(3)
        #self.call_place_service()

    def place_response_callback(self, future):
        result = future.result()
        self.get_logger().info(f"Place service result: {result}")

    def call_octomap_service(self, enable):
        request = EnableSceneCloud.Request()
        request.cloud_enabled = enable
        future = self.enable_octomap_client.call_async(request)

    def call_clear_octomap_service(self):
        request = Empty.Request()
        future = self.clear_octomap_client.call_async(request)

def input_thread(arm_test_node):
    try:
        while True:
            obj = input("Enter the object to pick or 'place <surface>' to place on surface (or 'exit' to quit):\n")
            if obj.lower() == 'exit':
                rclpy.shutdown()
                break
            elif 'place' in obj.lower():
                surface = obj.split(' ')[1]
                arm_test_node.call_place_service(surface)
            elif obj == 'enable':
                arm_test_node.call_octomap_service(True)
            elif obj == 'disable':
                arm_test_node.call_octomap_service(False)
            elif obj == 'clear':
                arm_test_node.call_clear_octomap_service()
            else:
                arm_test_node.call_pick_service(obj)
    except KeyboardInterrupt:
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    arm_test_node = ArmTestNode()

    input_thread_instance = threading.Thread(target=input_thread, args=(arm_test_node,))
    input_thread_instance.start()

    try:
        while rclpy.ok():
            rclpy.spin_once(arm_test_node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    arm_test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()