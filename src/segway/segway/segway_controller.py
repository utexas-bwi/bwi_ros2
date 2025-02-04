import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from segway_msgs.msg import Status, AuxPower, ConfigCmd
from sensor_msgs.msg import BatteryState
import tf2_ros
from tf2_geometry_msgs import TransformStamped
from tf_transformations import quaternion_from_euler, euler_from_quaternion, unit_vector

class Controller(Node):

    def __init__(self):
        super().__init__('segway_controller')

        self.cmd_pub = self.create_publisher(Twist, '/segway/cmd_vel', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.cfg_pub = self.create_publisher(ConfigCmd, '/segway/gp_command', 10)
        self.bat_pub = self.create_publisher(BatteryState, '/segway_battery', 10)

        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/segway/feedback/wheel_odometry', self.odom_callback, 10)
        self.status_sub = self.create_subscription(Status, '/segway/feedback/status', self.status_callback, 10)
        self.bat_sub = self.create_subscription(AuxPower, '/segway/feedback/aux_power', self.battery_callback, 10)


        self.odom_frame_id = self.declare_parameter('odom_frame_id', "odom").get_parameter_value().string_value
        self.robot_frame_id = self.declare_parameter('robot_frame_id', "base_footprint").get_parameter_value().string_value
        self.publish_tf = self.declare_parameter('publish_tf', True).get_parameter_value().bool_value

        self.stop_timer = None

        self.target_linear_vel = 0.0
        self.linear_vel = 0.0
        self.target_angular_vel = 0.0
        self.angular_vel = 0.0

        self.terminate = False
        self.ready = False

        self.frame_rate = self.declare_parameter('cmd_publish_rate', 5).get_parameter_value().integer_value

        # Supplied in m/s^2
        self.linear_pos_accel_limit = self.declare_parameter('linear_pos_accel_limit', 1).get_parameter_value().double_value / self.frame_rate
        self.linear_neg_accel_limit = self.declare_parameter('linear_neg_accel_limit', 1).get_parameter_value().double_value / self.frame_rate
        self.angular_pos_accel_limit = self.declare_parameter('angular_pos_accel_limit', 1).get_parameter_value().double_value / self.frame_rate
        self.angular_neg_accel_limit = self.declare_parameter('angular_neg_accel_limit', 1).get_parameter_value().double_value / self.frame_rate

        # Initialize TF2 broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.timer = self.create_timer(1 / self.frame_rate, self.publish_vel)

    def status_callback(self, msg):
        if not self.ready:
            if msg.operational_state == 3:
                cfg_cmd = ConfigCmd()
                cfg_cmd.gp_cmd = 'GENERAL_PURPOSE_CMD_SET_OPERATIONAL_MODE'
                cfg_cmd.gp_param = 5  # TRACTOR_REQUEST
                self.cfg_pub.publish(cfg_cmd)
                time.sleep(1)
                
            self.get_logger().info("Segway Controller ready")
            self.ready = True

    def odom_callback(self, msg):
        msg.header.frame_id = self.odom_frame_id
        msg.child_frame_id = self.robot_frame_id

        # Modify position appropriately.
        #msg.pose.pose.position.x *= -1  # reverse x
        #msg.pose.pose.position.y *= -1  # reverse y

        quaternion = (msg.pose.pose.orientation.x,
                      msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z,
                      msg.pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        quaternion = quaternion_from_euler(euler[0], euler[1], euler[2] - math.pi)
        quaternion = unit_vector(quaternion)
        quat_msg = Quaternion()
        quat_msg.x = quaternion[0]
        quat_msg.y = quaternion[1]
        quat_msg.z = quaternion[2]
        quat_msg.w = quaternion[3]
        msg.pose.pose.orientation = quat_msg

        # Modify velocity appropriately.
        msg.twist.twist.linear.x *= -1

        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = self.get_clock().now().to_msg()
            tf_msg.header.frame_id = self.odom_frame_id
            tf_msg.child_frame_id = self.robot_frame_id
            tf_msg.transform.translation.x = msg.pose.pose.position.x
            tf_msg.transform.translation.y = msg.pose.pose.position.y
            tf_msg.transform.translation.z = msg.pose.pose.position.z
            tf_msg.transform.rotation = quat_msg
            self.tf_broadcaster.sendTransform(tf_msg)

        self.odom_pub.publish(msg)

    def timer_callback(self):
        self.target_linear_vel = 0
        self.target_angular_vel = 0

    def cmd_callback(self, msg):
        self.target_linear_vel = -msg.linear.x
        self.target_angular_vel = msg.angular.z

        if self.stop_timer is not None:
            self.stop_timer.cancel()
        self.stop_timer = self.create_timer(0.5, self.timer_callback)

    def battery_callback(self, msg):
        bat_msg = BatteryState()
        bat_msg.voltage = msg.aux_voltage[0]
        bat_msg.current = float('nan')
        bat_msg.charge = float('nan')
        bat_msg.capacity = float('nan')
        bat_msg.design_capacity = float('nan')
        bat_msg.percentage = float('nan')
        bat_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        bat_msg.present = True
        bat_msg.header = msg.header
        self.bat_pub.publish(bat_msg)

    def publish_vel(self):
        if rclpy.ok() and not self.terminate:

            if not self.ready:
                return

            if self.linear_vel < self.target_linear_vel:
                # Increase speed
                if self.linear_pos_accel_limit == 0.0 or self.target_linear_vel - self.linear_vel < self.linear_pos_accel_limit:
                    self.linear_vel = self.target_linear_vel
                else:
                    self.linear_vel += self.linear_pos_accel_limit
            elif self.linear_vel > self.target_linear_vel:
                # Decrease speed
                if self.linear_neg_accel_limit == 0.0 or self.linear_vel - self.target_linear_vel < self.linear_neg_accel_limit:
                    self.linear_vel = self.target_linear_vel
                else:
                    self.linear_vel -= self.linear_neg_accel_limit

            if self.angular_vel < self.target_angular_vel:
                # Increase speed
                if self.angular_pos_accel_limit == 0.0 or self.target_angular_vel - self.angular_vel < self.angular_pos_accel_limit:
                    self.angular_vel = self.target_angular_vel
                else:
                    self.angular_vel += self.angular_pos_accel_limit
            elif self.angular_vel > self.target_angular_vel:
                # Decrease speed
                if self.angular_neg_accel_limit == 0.0 or self.angular_vel - self.target_angular_vel < self.angular_neg_accel_limit:
                    self.angular_vel = self.target_angular_vel
                else:
                    self.angular_vel -= self.angular_neg_accel_limit

            twist = Twist()
            twist.linear.x = float(self.linear_vel)
            twist.angular.z = float(self.angular_vel)
            self.cmd_pub.publish(twist)
        else:
            self.get_logger().info("ROS not ready")

            # Shutdown base and switch to standby
            #cfg_cmd = ConfigCmd()
            #cfg_cmd.gp_cmd = 'GENERAL_PURPOSE_CMD_SET_OPERATIONAL_MODE'
            #cfg_cmd.gp_param = 4  # STANDBY_REQUEST
            #self.cfg_pub.publish(cfg_cmd)

def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    time.sleep(5) #wait for hardware connection
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()