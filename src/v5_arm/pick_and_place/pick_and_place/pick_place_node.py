#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Bool, Float32
from threading import Thread
from pick_and_place.object_detector import segment_object
from pick_and_place.object_detector import DetectionResult
from pick_place_msgs.srv import ObjectManipulation
from pick_place_msgs.srv import AddSceneObject
from pick_place_msgs.srv import EnableSceneCloud
from std_srvs.srv import Empty
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import cv2
import numpy as np
import cv_bridge
import PIL

import time

class PickPlaceNode(Node):

    high_depth_image = None
    high_rgb_image = None
    low_depth_image = None
    low_rgb_image = None
    bridge = None

    def __init__(self):
        super().__init__('pick_place_node')

        self.bridge = cv_bridge.CvBridge()

        self.callback_group = MutuallyExclusiveCallbackGroup()

        self.pick_srv = self.create_service(ObjectManipulation, '/pick_object', self.pick_callback)
        self.place_srv = self.create_service(ObjectManipulation, '/place_object', self.place_callback)

        self.rgb_subscription_high = self.create_subscription(Image,'high_cam/rgb/image_raw',self.high_rgb_image_callback,1)
        self.depth_subscription_high = self.create_subscription(Image,'high_cam/depth_to_rgb/image_raw',self.high_depth_image_callback,1)
        self.rgb_subscription_low = self.create_subscription(Image,'low_cam/rgb/image_raw',self.low_rgb_image_callback,1)
        self.depth_subscription_low = self.create_subscription(Image,'low_cam/depth_to_rgb/image_raw',self.low_depth_image_callback,1)
        
        self.toggle_scene_cloud = self.create_client(EnableSceneCloud, '/pick_place/toggle_scene_cloud', callback_group=self.callback_group)
        self.clear_octomap = self.create_client(Empty, '/clear_octomap', callback_group=self.callback_group)
        self.add_grasp_object = self.create_client(AddSceneObject, '/pick_place/add_grasp_object', callback_group=self.callback_group)
        self.add_surface_client = self.create_client(AddSceneObject, '/pick_place/add_surface', callback_group=self.callback_group)
        self.pick_scene_object = self.create_client(ObjectManipulation, '/pick_place/pick_scene_object', callback_group=self.callback_group)
        self.place_scene_object = self.create_client(ObjectManipulation, '/pick_place/place_scene_object', callback_group=self.callback_group)
        self.update_tracked_objects = self.create_client(Empty, 'pick_place/update_tracked_objects', callback_group=self.callback_group)


    def high_depth_image_callback(self, msg):
        self.high_depth_image = self.bridge.imgmsg_to_cv2(msg)

    def high_rgb_image_callback(self, msg):
        self.high_rgb_image = self.bridge.imgmsg_to_cv2(msg)

    def low_depth_image_callback(self, msg):
        self.low_depth_image = self.bridge.imgmsg_to_cv2(msg)
    
    def low_rgb_image_callback(self, msg):
        self.low_rgb_image = self.bridge.imgmsg_to_cv2(msg)

    def pick_callback(self, request, response):
        self.get_logger().info('Pick object: %s' % request.object_label)

        if self.low_depth_image is None:
            self.get_logger().info('Waiting for image messages')
            response.success = False
            response.error_message = "No depth image received"
            return response

        # detect object
        label = request.object_label
        surface = request.surface_label
        camera = request.camera
        depth_snapshot = None
        image = detection = None
        if camera == "high":
            depth_snapshot = self.high_depth_image.astype(np.uint16)
            image, detection = segment_object(self.high_rgb_image, label)
        elif camera == "low":
            depth_snapshot = self.low_depth_image.astype(np.uint16)
            image, detection = segment_object(self.low_rgb_image, label)
        
        if detection == None:
            self.get_logger().info("Object not found")
            response.success = False
            response.error_message = "Object not found"
            return response

        depth_masked = np.copy(depth_snapshot)
        segment = detection.mask
        depth_masked[segment == 0] = 0

        self.get_logger().info('Object found')

        # enable scene cloud
        self.call_and_wait(self.clear_octomap, Empty.Request())
        self.call_and_wait(self.toggle_scene_cloud, EnableSceneCloud.Request(cloud_enabled=True))

        # Add object to scene
        mask_msg = cv_bridge.CvBridge().cv2_to_imgmsg(depth_masked)
        frame_id_msg = String()
        frame_id_msg.data = f"{camera}_camera_base"
        resp = self.call_and_wait(self.add_grasp_object, AddSceneObject.Request(depth_image=mask_msg, frame_id=frame_id_msg))
        if resp.object_name != "grasp_target":
            response.success = False
            response.error_message = "Failed to add object from depth image"
            return response
        
        # remove segment from depth image
        depth_snapshot = self.low_depth_image.astype(np.uint16)
        depth_filtered = np.copy(depth_snapshot)
        depth_filtered[segment != 0] = 0

        # will add the surface hopefully
        self.call_and_wait(self.update_tracked_objects, Empty.Request())

        # TODO (Tanay) Uncomment this and add a marker for the upper edge of the surface to match the 
        # TODO collision surface
        #self.add_surface("table", depth_filtered)
        
        time.sleep(2) # wait for octomap to stop updating
        
        # Disable scene cloud
        self.call_and_wait(self.toggle_scene_cloud, EnableSceneCloud.Request(cloud_enabled=False))

        # Pick object
        resp = self.call_and_wait(self.pick_scene_object, ObjectManipulation.Request(object_label="grasp_target", surface_label=surface))
        if not resp.success:
            response.success = False
            response.error_message = resp.error_message
            return response
        
        self.call_and_wait(self.clear_octomap, Empty.Request())
        
        response.success = True
        response.error_message = ""

        return response
    
    def add_surface(self, label, depth_snapshot):
        image, detection = segment_object(self.low_rgb_image, label)
        low_cam = True
        if detection == None:
            image, detection = segment_object(self.high_rgb_image, label)
            low_cam = False
        print("pick and place py label: ", label)

        if detection == None:
            self.get_logger().info("Surface not found")
            return False
        
        masked_depth = np.copy(depth_snapshot)
        masked_depth[detection.mask == 0] = 0
        frame_id = "low_camera_base" if low_cam else "high_camera_base"
        frame_id_msg = String()
        frame_id_msg.data = frame_id
        resp = self.call_and_wait(self.add_surface_client, AddSceneObject.Request(depth_image=self.bridge.cv2_to_imgmsg(masked_depth), frame_id=frame_id_msg))
        return resp
    
    def place_callback(self, request, response):
        self.get_logger().info("**************Place Callback**************")
        self.get_logger().info('Place object: %s' % request.object_label)
        self.get_logger().info('On surface: %s' % request.surface_label)

        if self.low_depth_image is None:
            self.get_logger().info('Waiting for image messages')
            response.success = False
            response.error_message = "No depth image received"
            return response
        
        # enable scene cloud
        self.get_logger().info("************************")
        self.get_logger().info("Clearing octomap")
        self.call_and_wait(self.clear_octomap, Empty.Request())
        self.get_logger().info("************************")
        self.get_logger().info("Enabling scene cloud")
        self.call_and_wait(self.toggle_scene_cloud, EnableSceneCloud.Request(cloud_enabled=True))
        
        time.sleep(1) # wait for octomap to update

        # add surface
        #resp = self.add_surface(request.object_label.data, self.low_depth_image)
        #if resp.object_name != "surface":
        #    response.success = False
        #    response.error_message = "Failed to add surface object"
        #    return response
        self.get_logger().info("************************")
        self.get_logger().info("Updating Tracking objects")
        self.call_and_wait(self.update_tracked_objects, Empty.Request())
        
        # Disable scene cloud
        self.get_logger().info("************************")
        self.get_logger().info("Disable Scene Cloud")
        self.call_and_wait(self.toggle_scene_cloud, EnableSceneCloud.Request(cloud_enabled=False))

        print("====================================")
        print("Surface Label: ", request.surface_label) 
        print("====================================")
        time.sleep(2) # wait for octomap to stop updating


        # Place object
        resp = self.call_and_wait(self.place_scene_object, ObjectManipulation.Request(object_label=request.object_label, surface_label=request.surface_label))
        if not resp.success:
            response.success = False
            response.error_message = resp.error_message
            return response
        
        self.call_and_wait(self.clear_octomap, Empty.Request())

        response.success = True
        response.error_message = ""

        return response
    
    def call_and_wait(self, client, request): # cannot spin in callback, but multi thread executor allows waiting
        future = client.call_async(request)
        while rclpy.ok() and not future.done():
            time.sleep(0.1)

        return future.result()


def main(args=None):
    rclpy.init(args=args)
    pick_place_node = PickPlaceNode()
    
    executor = MultiThreadedExecutor(num_threads=6)  # Prevent deadlock when using services

    executor.add_node(pick_place_node)

    try:
        executor.spin()  # Spin the executor to process callbacks
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        pick_place_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()