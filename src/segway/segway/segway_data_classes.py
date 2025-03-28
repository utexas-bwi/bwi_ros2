from .utils import *
from segway_msgs.msg import *
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import NavSatFix, NavSatStatus, JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, Header
import math
import os
import tf2_ros
from tf_transformations import quaternion_from_euler, euler_from_quaternion

HAS_2_WHEELS = [2, 3, 5]

header_stamp = 0
platform_identifier = 0
machine_id = 1
has_external_imu = 0
has_brakes = 0
has_segway_bsa = 0
num_wheels = 2
wheel_circum = 0.542391 * math.pi


class RMP_Status:
    def __init__(self, node):
        self._node = node
        self._MsgData = Status()
        self._MsgPub = self._node.create_publisher(Status, '/segway/feedback/status', 10)
        self._MsgData.header.frame_id = ''
        self._seq = 0
        self.init = True
        self.op_mode = 0

    def parse(self, data):
        global header_stamp
        global platform_identifier
        global machine_id
        global has_external_imu
        global has_brakes
        global has_segway_bsa
        global num_wheels

        rmp_timestamp = round(convert_u32_to_float(data[8]), 2)

        if self.init:
            self.platform_identifier = data[12]
            machine_id = self.platform_identifier & 0x1F
            has_external_imu = (platform_identifier & 0x20) >> 5
            has_brakes = (self.platform_identifier & 0x40) >> 6
            has_segway_bsa = (self.platform_identifier & 0x80) >> 7
            self.start_time_machine = rmp_timestamp
            self.start_time_local = float(self._node.get_clock().now().nanoseconds) * 1e-9
            self.init = False

            if machine_id in HAS_2_WHEELS:
                num_wheels = 2
            else:
                num_wheels = 4

        header_stamp = self._node.get_clock().now().to_msg()

        self._MsgData.header.stamp = header_stamp
        
        temp = [data[0], data[1], data[2], data[3]]
        self._MsgData.fault_status_words = temp
        temp = [data[4], data[5], data[6], data[7]]
        self._MsgData.mcu_fault_status = temp
        self._MsgData.operational_time = rmp_timestamp
        self._MsgData.operational_state = data[9]
        self.op_mode = data[9]
        self._MsgData.dynamic_response = data[10]
        self._MsgData.machine_id = self.platform_identifier

        self._MsgPub.publish(self._MsgData)
        self._seq += 1

        return header_stamp


class RMP_Propulsion:
    def __init__(self, node):
        self._node = node
        self._MsgData = Propulsion()
        self._MsgPub = self._node.create_publisher(Propulsion, '/segway/feedback/propulsion', 10)
        self._MsgData.header.frame_id = ''
        self._seq = 0

    def parse(self, data):
        self._MsgData.header.stamp = header_stamp
        

        self._MsgData.min_propulsion_battery_soc = convert_u32_to_float(data[0])
        temp = [convert_u32_to_float(data[1]),
                convert_u32_to_float(data[2]),
                convert_u32_to_float(data[3]),
                convert_u32_to_float(data[4])]
        self._MsgData.mcu_battery_soc = temp
        temp = [convert_u32_to_float(data[5]),
                convert_u32_to_float(data[6]),
                convert_u32_to_float(data[7]),
                convert_u32_to_float(data[8])]
        self._MsgData.mcu_battery_temp_deg = temp

        temp = [convert_u32_to_float(data[9]),
                convert_u32_to_float(data[10]),
                convert_u32_to_float(data[11]),
                convert_u32_to_float(data[12])]
        self._MsgData.mcu_inst_power = temp
        temp = [convert_u32_to_float(data[13]),
                convert_u32_to_float(data[14]),
                convert_u32_to_float(data[15]),
                convert_u32_to_float(data[16])]
        self._MsgData.mcu_total_energy = temp
        temp = [convert_u32_to_float(data[17]),
                convert_u32_to_float(data[18]),
                convert_u32_to_float(data[19]),
                convert_u32_to_float(data[20])]
        self._MsgData.motor_current = temp

        self._MsgData.max_motor_current = convert_u32_to_float(data[21])

        temp = [convert_u32_to_float(data[22]),
                convert_u32_to_float(data[23]),
                convert_u32_to_float(data[24]),
                convert_u32_to_float(data[25])]
        self._MsgData.motor_current_limit = temp

        self._MsgData.min_motor_current_limit = convert_u32_to_float(data[26])

        self._MsgPub.publish(self._MsgData)
        self._seq += 1


class RMP_AuxPower:
    def __init__(self, node):
        self._node = node
        self._MsgData = AuxPower()
        self._MsgPub = self._node.create_publisher(AuxPower, '/segway/feedback/aux_power', 10)
        self._MsgData.header.frame_id = ''
        self._seq = 0

    def parse(self, data):
        self._MsgData.header.stamp = header_stamp
        

        temp = [convert_u32_to_float(data[0]),
                convert_u32_to_float(data[1])]
        self._MsgData.aux_soc = temp

        temp = [convert_u32_to_float(data[2]),
                convert_u32_to_float(data[3])]
        self._MsgData.aux_voltage = temp

        temp = [convert_u32_to_float(data[4]),
                convert_u32_to_float(data[5])]
        self._MsgData.aux_current = temp

        temp = [convert_u32_to_float(data[6]),
                convert_u32_to_float(data[7])]
        self._MsgData.aux_temp_deg = temp

        temp = [data[8], data[9]]
        self._MsgData.aux_sys_status = temp

        temp = [data[10], data[11]]
        self._MsgData.aux_bcu_status = temp

        temp = [data[12], data[13]]
        self._MsgData.aux_bcu_faults = temp

        self._MsgData.startup_7p2v_batt_voltage = convert_u32_to_float(data[14])

        self._MsgPub.publish(self._MsgData)
        self._seq += 1


class AHRS_Packet:
    def __init__(self, node):
        self._node = node
        self.accel_mps2 = [0.0] * 3
        self.gyro_rps = [0.0] * 3
        self.mag_T = [0.0] * 3
        self.rpy_rad = [0.0] * 3
        self.timestamp_sec = 0.0
        self.linear_accel_covariance = 0.098 * 0.098
        self.angular_velocity_covariance = 0.012 * 0.012
        self.orientation_covariance = 0.035 * 0.035
        self.magnetic_field_covariance = 0.000002 * 0.000002

        self.publish_data = False
        self._MsgData = Imu()
        self._MsgPub = self._node.create_publisher(Imu, '/segway/feedback/ext_imu', 10)
        self._MsgData.header.frame_id = '/segway/ext_imu_frame'
        self._seq = 0

        self._MagMsgData = MagneticField()
        self._MagMsgPub = self._node.create_publisher(MagneticField, '/segway/feedback/ext_mag_field', 10)
        self._MagMsgData.header.frame_id = '/segway/ext_imu_frame'
        self._Magseq = 0

    def parse_data(self, data):
        index = 0
        for i in range(0, 3):
            self.accel_mps2[i] = convert_u32_to_float(data[index]) * 9.81
            index += 1
        for i in range(0, 3):
            self.gyro_rps[i] = convert_u32_to_float(data[index])
            index += 1
        for i in range(0, 3):
            self.mag_T[i] = convert_u32_to_float(data[index]) * 0.0001
            index += 1
        for i in range(0, 3):
            self.rpy_rad[i] = convert_u32_to_float(data[index])
            index += 1

        self.timestamp_sec = convert_float_to_u32(convert_u32_to_float(data[index]) * 62500)

        self._MsgData.header.stamp = header_stamp
        self._MagMsgData.header.stamp = header_stamp

        roll = -self.rpy_rad[0]
        pitch = self.rpy_rad[1]
        yaw = self.rpy_rad[2]
        while yaw <= -math.pi:
            yaw += 2.0 * math.pi
        while yaw > math.pi:
            yaw -= 2.0 * math.pi
        yaw *= -1.0

        q = quaternion_from_euler(roll, pitch, yaw)

        self._MsgData.orientation.x = q[0]
        self._MsgData.orientation.y = q[1]
        self._MsgData.orientation.z = q[2]
        self._MsgData.orientation.w = q[3]

        self._MsgData.orientation_covariance[0] = self.orientation_covariance
        self._MsgData.orientation_covariance[4] = self.orientation_covariance
        self._MsgData.orientation_covariance[8] = self.orientation_covariance

        self._MsgData.linear_acceleration.x = -self.accel_mps2[0]
        self._MsgData.linear_acceleration.y = self.accel_mps2[1]
        self._MsgData.linear_acceleration.z = -self.accel_mps2[2]
        self._MsgData.linear_acceleration_covariance[0] = self.linear_accel_covariance
        self._MsgData.linear_acceleration_covariance[4] = self.linear_accel_covariance
        self._MsgData.linear_acceleration_covariance[8] = self.linear_accel_covariance

        self._MsgData.angular_velocity.x = -self.gyro_rps[0]
        self._MsgData.angular_velocity.y = self.gyro_rps[1]
        self._MsgData.angular_velocity.z = -self.gyro_rps[2]
        self._MsgData.angular_velocity_covariance[0] = self.angular_velocity_covariance
        self._MsgData.angular_velocity_covariance[4] = self.angular_velocity_covariance
        self._MsgData.angular_velocity_covariance[8] = self.angular_velocity_covariance

        self._MagMsgData.magnetic_field.x = -self.mag_T[0]
        self._MagMsgData.magnetic_field.y = self.mag_T[1]
        self._MagMsgData.magnetic_field.z = -self.mag_T[2]
        self._MagMsgData.magnetic_field_covariance[0] = self.magnetic_field_covariance
        self._MagMsgData.magnetic_field_covariance[4] = self.magnetic_field_covariance
        self._MagMsgData.magnetic_field_covariance[8] = self.magnetic_field_covariance

        if self.publish_data:
            self._MsgPub.publish(self._MsgData)
            self._MagMsgPub.publish(self._MagMsgData)
            self._seq += 1


class GPS_Packet:
    def __init__(self, node):
        self._node = node
        self.latitude_deg     = 0.0
        self.longitude_deg    = 0.0
        self.msl_height_m     = 0.0
        self.el_height_m      = 0.0
        self.horizontal_stdev = 0.0
        self.vertical_stdev   = 0.0
        self.valid_llh_flag   = 0
        
        self.fix_valid_flags        = 0
        self.fix_type               = 0
        self.fix_num_space_vehicles = 0
        
        self.hw_sensor_state   = 0
        self.hw_antenna_state  = 0
        self.hw_antenna_power  = 0
        self.hw_valid_hw_flags = 0
        
        self.publish_data = False
        
        self._HVMsgData = NavSatFix()
        self._HVMsgPub = self._node.create_publisher(NavSatFix, '/segway/feedback/gps/fix_3d', 10)
        self._HVMsgData.header.frame_id = 'segway/gps_frame'
        self._HVMsgData.status.service = NavSatStatus.SERVICE_GPS
        
        self._HMsgData = NavSatFix()
        self._HMsgPub = self._node.create_publisher(NavSatFix, '/segway/feedback/gps/fix_2d', 10)
        self._HMsgData.header.frame_id = 'segway/gps_frame'
        self._HMsgData.status.service = NavSatStatus.SERVICE_GPS
        self._seq = 0
        
        self.LAT_LON_FIX_VALID          = 0x0001
        self.ELLIPSOID_HEIGHT_FIX_VALID = 0x0002
        self.MSL_HEIGHT_FIX_VALID       = 0x0004
        self.HORIZONTAL_ACCURACY_VALID  = 0x0008
        self.VERTICAL_ACCURACY_VALID    = 0x0010

    def parse_data(self, data, header_stamp):
        self.latitude_deg     = convert_u64_to_double(data[0],data[1])
        self.longitude_deg    = convert_u64_to_double(data[2],data[3])
        self.el_height_m      = convert_u64_to_double(data[4],data[5])
        self.msl_height_m     = convert_u64_to_double(data[6],data[7])
        self.horizontal_stdev = convert_u32_to_float(data[8])
        self.vertical_stdev   = convert_u32_to_float(data[9])
        self.valid_llh_flag   = data[10]
        
        self.fix_valid_flags        = (data[11] & 0x0000FFFF) 
        self.fix_type               = (data[11] & 0xFF000000) >> 24 
        self.fix_num_space_vehicles = (data[11] & 0x00FF0000) >> 16
        
        self.hw_sensor_state   = (data[12] & 0xFF000000) >> 24
        self.hw_antenna_state  = (data[12] & 0x00FF0000) >> 16
        self.hw_antenna_power  = (data[12] & 0x0000FF00) >> 8
        self.hw_valid_hw_flags = (data[12] & 0x000000FF)
        
        self._HVMsgData.header.stamp = header_stamp
        self._HMsgData.header.stamp = header_stamp
        
        self._HVMsgData.latitude = self.latitude_deg
        self._HVMsgData.longitude = self.longitude_deg
        self._HVMsgData.altitude = self.el_height_m
        
        self._HMsgData.latitude = self.latitude_deg
        self._HMsgData.longitude = self.longitude_deg
        self._HMsgData.altitude = 0.0      
        
        if (self.LAT_LON_FIX_VALID == (self.LAT_LON_FIX_VALID & self.valid_llh_flag)):
            self._HMsgData.status.status = NavSatStatus.STATUS_FIX
            if (self.ELLIPSOID_HEIGHT_FIX_VALID == (self.ELLIPSOID_HEIGHT_FIX_VALID & self.valid_llh_flag)):
                self._HVMsgData.status.status = NavSatStatus.STATUS_FIX
            else:
                self._HVMsgData.status.status = NavSatStatus.STATUS_NO_FIX
        else:
            self._HVMsgData.status.status = NavSatStatus.STATUS_NO_FIX
            self._HMsgData.status.status = NavSatStatus.STATUS_NO_FIX             
        
        if (self.HORIZONTAL_ACCURACY_VALID == (self.HORIZONTAL_ACCURACY_VALID & self.valid_llh_flag)):
            self._HMsgData.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            self._HMsgData.position_covariance[0] = self.horizontal_stdev * self.horizontal_stdev
            self._HMsgData.position_covariance[4] = self.horizontal_stdev * self.horizontal_stdev
            self._HMsgData.position_covariance[8] = 100.0
            
            if (self.VERTICAL_ACCURACY_VALID == (self.VERTICAL_ACCURACY_VALID & self.valid_llh_flag)):
                self._HVMsgData.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
                self._HMsgData.position_covariance[0] = self.horizontal_stdev * self.horizontal_stdev
                self._HMsgData.position_covariance[4] = self.horizontal_stdev * self.horizontal_stdev
                self._HMsgData.position_covariance[8] = self.vertical_stdev * self.vertical_stdev
            else:
                self._HVMsgData.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        else:
            self._HVMsgData.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            self._HMsgData.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            
        if (self.publish_data):
            self._HMsgPub.publish(self._HMsgData)
            self._HVMsgPub.publish(self._HVMsgData)    
            self._seq+=1


class BSA_Packet:
    def __init__(self, node):
        self._node = node
        self.accel_mps2       = [0.0]*3
        self.gyro_rps         = [0.0]*3
        self.rpy_rad          = [0.0]*3
        self.linear_accel_covariance = 0.098 * 0.098
        self.angular_velocity_covariance = 0.012 * 0.012
        self.orientation_covariance = 0.035 * 0.035
        
        self.publish_data = False
        self._MsgData = Imu()
        self._MsgPub = self._node.create_publisher(Imu, '/segway/feedback/segway_imu', 10)
        self._StatData = Int32()
        self._StatPub = self._node.create_publisher(Int32, '/segway/feedback/segway_imu/stat', 10)
        self._MsgData.header.frame_id = 'segway/bsa_imu_frame'
        self._seq = 0

    def parse_data(self, data, header_stamp):
        self.accel_mps2[0] = convert_u32_to_float(data[0]) * 9.81
        self.accel_mps2[1] = convert_u32_to_float(data[1]) * 9.81
        self.accel_mps2[2] = 0.0
        self.rpy_rad[1] = convert_u32_to_float(data[5]) * (math.pi/180.0)
        self.gyro_rps[1] = convert_u32_to_float(data[6]) * (math.pi/180.0)
        self.rpy_rad[0] = convert_u32_to_float(data[7]) * (math.pi/180.0)
        self.gyro_rps[0] = convert_u32_to_float(data[8]) * (math.pi/180.0)
        self.gyro_rps[2] = -convert_u32_to_float(data[9]) * (math.pi/180.0)
        self.rpy_rad[2] = 0.0
        self.pse_data_valid = data[10]

        self._MsgData.header.stamp = header_stamp
        
        
        q = tf_transformations.quaternion_from_euler(self.rpy_rad[0], self.rpy_rad[1], self.rpy_rad[2], 'sxyz')
        
        self._MsgData.orientation.x = q[0]
        self._MsgData.orientation.y = q[1]
        self._MsgData.orientation.z = q[2]
        self._MsgData.orientation.w = q[3]
        
        self._MsgData.orientation_covariance[0] = self.orientation_covariance
        self._MsgData.orientation_covariance[4] = self.orientation_covariance
        self._MsgData.orientation_covariance[8] = 9999.0

        self._MsgData.angular_velocity.x = self.gyro_rps[0]
        self._MsgData.angular_velocity.y = self.gyro_rps[1]
        self._MsgData.angular_velocity.z = self.gyro_rps[2]
        
        self._MsgData.angular_velocity_covariance[0] = self.angular_velocity_covariance
        self._MsgData.angular_velocity_covariance[4] = self.angular_velocity_covariance
        self._MsgData.angular_velocity_covariance[8] = self.angular_velocity_covariance
        
        self._MsgData.linear_acceleration.x = self.accel_mps2[0]
        self._MsgData.linear_acceleration.y = self.accel_mps2[1]
        self._MsgData.linear_acceleration.z = self.accel_mps2[2]
        
        self._MsgData.linear_acceleration_covariance[0] = self.linear_accel_covariance
        self._MsgData.linear_acceleration_covariance[4] = self.linear_accel_covariance
        self._MsgData.linear_acceleration_covariance[8] = self.linear_accel_covariance
        
        if (self.publish_data):
            self._MsgPub.publish(self._MsgData)
            self._StatPub.publish(self.pse_data_valid)
            self._seq+=1        
        
class RMP_IMU:

    def __init__(self, node):
        self._node = node
        self.status = 0
        self.errors = 0

        imu_src = self._node.declare_parameter('ext_imu_src', 'none').value
        if imu_src == '3dm_gx3':
            self.ahrs = AHRS_Packet()
            self.gps = GPS_Packet()
        elif imu_src == 'um7_imu':
            self._um7_sub = self._node.create_subscription(Imu, '/um7/data', self.ExternalImuCallback, 10)
            self._um7_pub = self._node.create_publisher(Imu, '/segway/feedback/ext_imu', 10)
            self._um7_data = Imu()

        try:
            if os.environ.get('SEGWAY_HAS_BSA') == 'true':
                self.bsa = BSA_Packet()
        except KeyError:
            pass

    def ExternalImuCallback(self, imu_data):
        self._um7_data.header = imu_data.header
        self._um7_data.orientation = imu_data.orientation
        self._um7_data.orientation_covariance[0] = 0.035 * 0.035
        self._um7_data.orientation_covariance[4] = 0.035 * 0.035
        self._um7_data.orientation_covariance[8] = 0.035 * 0.035

        self._um7_data.linear_acceleration = imu_data.linear_acceleration
        self._um7_data.linear_acceleration_covariance[0] = 0.098 * 0.098
        self._um7_data.linear_acceleration_covariance[4] = 0.098 * 0.098
        self._um7_data.linear_acceleration_covariance[8] = 0.098 * 0.098

        self._um7_data.angular_velocity = imu_data.angular_velocity
        self._um7_data.angular_velocity_covariance[0] = 0.012 * 0.012
        self._um7_data.angular_velocity_covariance[4] = 0.012 * 0.012
        self._um7_data.angular_velocity_covariance[8] = 0.012 * 0.012

        self._um7_pub.publish(self._um7_data)

    def parse_data(self, data):
        if has_segway_bsa:
            self.bsa.parse_data(data[0:11])
        if has_external_imu:
            self.ahrs.parse_data(data[11:24])
            self.gps.parse_data(data[24:37])
            self.status = data[38] & 0xFF
            self.missed_ahrs_messages = (self.status & 0xFF000000) >> 24
            self.missed_gps_messages = (self.status & 0x00FF0000) >> 16

            if not rclpy.ok() and self.status == 2:
                self.ahrs.publish_data = True
                self.gps.publish_data = True

class RMP_Dynamics:

    def __init__(self, node):
        self._node = node
        self._use_platform_odometry = self._node.declare_parameter('use_platform_odometry', False).value
        self._MsgData = Dynamics()
        self._MsgPub = self._node.create_publisher(Dynamics, '/segway/feedback/dynamics', 10)
        self._jointStatePub = self._node.create_publisher(JointState, 'rmp_joint_states', 10)
        self._jointStateMsg = JointState()
        
        num_wheels = 2  # Set this variable to the appropriate number of wheels
        if num_wheels == 2:
            names = ['left_wheel', 'right_wheel']
        else:
            names = ['left_front_wheel', 'right_front_wheel', 'left_rear_wheel', 'right_rear_wheel']
        
        self.has_rear_caster = os.environ.get("SEGWAY_HAS_REAR_CASTER", "false").lower() == "true"
        self.has_front_caster = os.environ.get("SEGWAY_HAS_FRONT_CASTER", "false").lower() == "true"
                
        if self.has_rear_caster:
            names.append('rear_caster_swivel')
            names.append('rear_caster_wheel')
            
        if self.has_front_caster:
            names.append('front_caster_swivel')
            names.append('front_caster_wheel')
            
        self._jointStateMsg.name = names
        
        self._MsgData.header.frame_id = ''
        self._jointStateMsg.header.frame_id = ''

        self._OdomData = Odometry()
        if not self._use_platform_odometry:
            self._OdomPub = self._node.create_publisher(Odometry, '/segway/feedback/wheel_odometry', 10)
            self._node.create_subscription(Odometry, '/segway/odometry/local_filtered', self._update_odom_yaw, 10)
        else:
            self._OdomPub = self._node.create_publisher(Odometry, '/segway/odometry/local_filtered', 10)
        
        self._OdomData.header.frame_id = 'odom'
        self._OdomData.child_frame_id = 'segway/base_link'
        
        self._OdomData.pose.covariance = [0.00017, 0.0, 0.0, 0.0, 0.0, 0.0,
                                          0.0, 0.00017, 0.0, 0.0, 0.0, 0.0,
                                          0.0, 0.0, 0.00017, 0.0, 0.0, 0.0,
                                          0.0, 0.0, 0.0, 0.00000, 0.0, 0.0,
                                          0.0, 0.0, 0.0, 0.0, 0.00000, 0.0,
                                          0.0, 0.0, 0.0, 0.0, 0.0, 0.00017]
             
        self._OdomData.twist.covariance = [0.00017, 0.0, 0.0, 0.0, 0.0, 0.0,
                                           0.0, 0.00017, 0.0, 0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.00017, 0.0, 0.0, 0.0,
                                           0.0, 0.0, 0.0, 0.00000, 0.0, 0.0,
                                           0.0, 0.0, 0.0, 0.0, 0.00000, 0.0,
                                           0.0, 0.0, 0.0, 0.0, 0.0, 0.00017]
        
        self._seq = 0
        
    def _update_odom_yaw(self, msg):
        (r, p, y) = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self._MsgData.odom_yaw_angle_rad = y

    def parse(self, data):
        header_stamp = self._node.get_clock().now().to_msg()
        self._MsgData.header.stamp = header_stamp
        self._OdomData.header.stamp = header_stamp
        self._jointStateMsg.header.stamp = header_stamp
        self._MsgData.x_vel_target_mps = convert_u32_to_float(data[0])
        self._MsgData.y_vel_target_mps = convert_u32_to_float(data[1])
        self._MsgData.yaw_rate_target_rps = convert_u32_to_float(data[2])
        self._MsgData.yaw_rate_limit_rps = convert_u32_to_float(data[3])
        self._MsgData.vel_limit_mps = convert_u32_to_float(data[4])
        
        temp = [convert_u32_to_float(data[5]),
                convert_u32_to_float(data[6]),
                convert_u32_to_float(data[7]),
                convert_u32_to_float(data[8])]
        self._MsgData.wheel_vel_mps = temp
        joint_vel = [(-1.0 * temp[i]) for i in range(num_wheels)]
        temp = [convert_u32_to_float(data[9]),
                convert_u32_to_float(data[10]),
                convert_u32_to_float(data[11]),
                convert_u32_to_float(data[12])]
        self._MsgData.wheel_pos_m = temp
        joint_pos = [(-1.0 * ((temp[i] / wheel_circum) % 1.0) * (2 * math.pi)) for i in range(num_wheels)]
        self._MsgData.x_accel_mps2 = convert_u32_to_float(data[13])
        self._MsgData.y_accel_mps2 = convert_u32_to_float(data[14])
        self._MsgData.yaw_accel_mps2 = convert_u32_to_float(data[15])
        self._OdomData.twist.twist.linear.x = convert_u32_to_float(data[16])
        self._OdomData.twist.twist.linear.y = convert_u32_to_float(data[17])
        self._OdomData.twist.twist.linear.z = 0.0
        self._OdomData.twist.twist.angular.x = 0.0
        self._OdomData.twist.twist.angular.y = 0.0
        self._OdomData.twist.twist.angular.z = convert_u32_to_float(data[18])
        self._OdomData.pose.pose.position.x = convert_u32_to_float(data[19])
        self._OdomData.pose.pose.position.y = convert_u32_to_float(data[20])
        self._OdomData.pose.pose.position.z = 0.0
        self._MsgData.yaw_angle_rad = convert_u32_to_float(data[21])
        rot = quaternion_from_euler(0, 0, convert_u32_to_float(data[21]))
        self._OdomData.pose.pose.orientation.x = rot[0]
        self._OdomData.pose.pose.orientation.y = rot[1]
        self._OdomData.pose.pose.orientation.z = rot[2]
        self._OdomData.pose.pose.orientation.w = rot[3]
        
        x = self._OdomData.pose.pose.position.x
        y = self._OdomData.pose.pose.position.y
        z = self._OdomData.pose.pose.position.z  
        
        """
        Just use zeros for the joints
        """
        if self.has_rear_caster:
            for i in range(2):
                joint_vel.append(0.0)
                joint_pos.append(0.0)
        if self.has_front_caster:
            for i in range(2):
                joint_vel.append(0.0)
                joint_pos.append(0.0)

        self._jointStateMsg.velocity = joint_vel
        self._jointStateMsg.position = joint_pos

        if rclpy.ok():
            self._OdomPub.publish(self._OdomData)
            self._MsgPub.publish(self._MsgData)
            self._jointStatePub.publish(self._jointStateMsg)
            if self._use_platform_odometry:
                br = tf2_ros.TransformBroadcaster(self)
                t = geometry_msgs.msg.TransformStamped()
                t.header.stamp = header_stamp
                t.header.frame_id = "odom"

class RMP_Configuration:

    def __init__(self, node):
        self._node = node
        self._MsgData = Configuration()
        self._MsgPub = self._node.create_publisher(Configuration, '/segway/feedback/configuration', 10)
        self._MsgData.header = Header()
        self._MsgData.header.frame_id = ''
        self._seq = 0
        self.configuration_feedback = [0] * 16

    def set_teleop_config(self, data):
        self._MsgData.teleop_vel_limit_mps = convert_u32_to_float(data[0])
        self._MsgData.teleop_accel_limit_mps2 = convert_u32_to_float(data[1])
        self._MsgData.teleop_yaw_rate_limit_rps = convert_u32_to_float(data[2])
        self._MsgData.teleop_yaw_accel_limit_rps2 = convert_u32_to_float(data[3])

    def parse(self, data):
        global wheel_circum
        header_stamp = self._node.get_clock().now().to_msg()
        
        self._MsgData.header.stamp = header_stamp
        

        self.configuration_feedback = data
        self._MsgData.vel_limit_mps = convert_u32_to_float(data[0])
        self._MsgData.accel_limit_mps2 = convert_u32_to_float(data[1])
        self._MsgData.decel_limit_mps2 = convert_u32_to_float(data[2])
        self._MsgData.dtz_decel_limit_mps2 = convert_u32_to_float(data[3])
        self._MsgData.yaw_rate_limit_rps = convert_u32_to_float(data[4])
        self._MsgData.yaw_accel_limit_rps2 = convert_u32_to_float(data[5])
        self._MsgData.lateral_accel_limit_mps2 = convert_u32_to_float(data[6])
        self._MsgData.tire_diameter_m = convert_u32_to_float(data[7])
        wheel_circum = self._MsgData.tire_diameter_m * math.pi
        self._MsgData.wheelbase_length_m = convert_u32_to_float(data[8])
        self._MsgData.wheel_track_width_m = convert_u32_to_float(data[9])
        self._MsgData.gear_ratio = convert_u32_to_float(data[10])
        self._MsgData.config_bitmap = data[11]
        self._MsgData.eth_ip_address = numToDottedQuad(data[12])
        self._MsgData.eth_port_number = data[13]
        self._MsgData.eth_subnet_mask = numToDottedQuad(data[14])
        self._MsgData.eth_gateway = numToDottedQuad(data[15])

        if rclpy.ok():
            self._MsgPub.publish(self._MsgData)
            self._seq += 1

class RMP_DATA:
    def __init__(self, node):

        self.status = RMP_Status(node)
        self.propulsion = RMP_Propulsion(node)
        self.auxiliary_power = RMP_AuxPower(node)
        self.config_param = RMP_Configuration(node)
        self.dynamics = RMP_Dynamics(node)
        self.imu = RMP_IMU(node)

