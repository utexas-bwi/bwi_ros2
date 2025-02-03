import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters, ListParameters
from geometry_msgs.msg import Twist
from segway_msgs.msg import ConfigCmd
from std_msgs.msg import String
from .system_defines import *
from .utils import *
from .io_eth import IoEthThread
from .io_usb import IoUsbThread
from .segway_data_classes import RMP_DATA
from segway_msgs.msg import Faultlog
import re
import os
import select
import threading
import multiprocessing
import time

#TODO: put params in config file or something
params = {
    "teleop_vel_limit_mps": 0.5,
    "teleop_accel_limit_mps2": 0.5,
    "teleop_yaw_rate_limit_rps": 1.0,
    "teleop_yaw_accel_limit_rps2": 1.0,
    "vel_limit_mps": 1.0,
    "accel_limit_mps2": 0.5,
    "decel_limit_mps2": 0.5,
    "dtz_decel_limit_mps2": 1.0,
    "yaw_rate_limit_rps": 0.8,
    "yaw_accel_limit_rps2": 1.0,
    "lateral_accel_limit_mps2": 4.905,
    "tire_rolling_diameter_m": 0.46228,
    "wheel_base_length_m": 0.62,
    "wheel_track_width_m": 0.48,
    "gear_ratio": 24.2667,
    "enable_audio": True,
    "motion_while_charging": False,
    "balance_mode_enabled": False,
    "balace_gains": 0x0,
    "vel_ctl_input_filter": 0x0,
    "yaw_ctl_input_filter": 0x0,
    "torqe_limit": 100.0
}

# Dictionary for all RMP configuration command IDs
command_ids = {
    "GENERAL_PURPOSE_CMD_NONE": 0,
    "GENERAL_PURPOSE_CMD_SET_AUDIO_COMMAND": 1,
    "GENERAL_PURPOSE_CMD_SET_OPERATIONAL_MODE": 2,
    "GENERAL_PURPOSE_CMD_SEND_SP_FAULTLOG": 3,
    "GENERAL_PURPOSE_CMD_RESET_INTEGRATORS": 4,
    "GENERAL_PURPOSE_CMD_RESET_PARAMS_TO_DEFAULT": 5
}

class SegwayHardwareInterface(Node):

    def __init__(self):
        super().__init__('segway_hardware_interface')

        # Variables to track communication frequency for debugging
        self.summer = 0
        self.samp = 0
        self.avg_freq = 0.0
        self.start_frequency_samp = False
        self.need_to_terminate = False
        self.flush_rcvd_data = True
        self.update_base_local_planner = False
        self.last_move_base_update = self.get_clock().now().to_msg().sec
        self.terminate_mutex = None

        interface = self.declare_parameter("interface", "eth")

        # Ensure a valid platform
        self.platform = self.declare_parameter('platform', "RMP_220").value
        if self.platform not in SUPPORTED_PLATFORMS:
            self.get_logger().error(f"Platform defined is not supported: {self.platform}")
            return

        # The 440 platforms use the same drivers but different URDF
        pattern = re.compile("RMP_440")
        if pattern.search(self.platform):
            self.platform = "RMP_440"

        # Initialize the publishers for RMP
        self.rmp_data = RMP_DATA(self)

        # Initialize faultlog related items
        self.is_init = True
        self.extracting_faultlog = False

        # Initialize the parameter server and dynamic reconfigure
        self.param_server_initialized = False

        # Create the thread to run RMP communication
        interface = self.get_parameter('interface').value or 'eth'
        self.tx_queue_ = multiprocessing.Queue()
        self.rx_queue_ = multiprocessing.Queue()
        if interface == 'eth':
            self.comm = IoEthThread(("10.66.171.5", 8080),
                                    self.tx_queue_, self.rx_queue_, max_packet_size=1248)
        elif interface == 'usb':
            self.comm = IoUsbThread('ttyACM0', self.tx_queue_, self.rx_queue_, max_packet_size=1248)

        if not self.comm.link_up:
            self.get_logger().error("Could not open socket for RMP...")
            self.comm.Close()
            return

        # Start the receive handler thread
        self.terminate_mutex = threading.RLock()
        self.last_rsp_rcvd = self.get_clock().now().to_msg().sec
        self._rcv_thread = threading.Thread(target=self._run)
        self._rcv_thread.start()

        # Start streaming continuous data
        self.get_logger().info("Stopping the data stream")
        if not self._continuous_data(False):
            self.get_logger().error("Could not stop RMP communication stream")
            self.__del__()
            return
        
        self.faultlog_pub = self.create_publisher(Faultlog, "segway/feedback/faultlog", 10)

        # Extract the faultlog at startup
        self.flush_rcvd_data = False
        self.get_logger().info("Extracting the faultlog")
        self.extracting_faultlog = True

        if not self._extract_faultlog():
            self.get_logger().error("Could not get retrieve RMP faultlog")
            self.__del__()
            return

        # Start streaming continuous data
        self.get_logger().info("Starting the data stream")
        if not self._continuous_data(True):
            self.get_logger().error("Could not start RMP communication stream")
            self.__del__()
            return
        
        # send default configuration
        self.set_params()

        self.start_frequency_samp = True

        time.sleep(1)

        # Indicate the driver is up with motor audio
        cmds = [GENERAL_PURPOSE_CMD_ID, [GENERAL_PURPOSE_CMD_SET_AUDIO_COMMAND, MOTOR_AUDIO_PLAY_EXIT_ALARM_SONG]]
        self._add_command_to_queue(cmds)

        self.create_subscription(Twist, "segway/cmd_vel", self._add_motion_command_to_queue, 10)
        self.create_subscription(ConfigCmd, "segway/gp_command", self._send_command, 10)

        self.get_logger().info("Segway Driver is up and running")
        

    def __del__(self):
        self.get_logger().error("Segway Driver has called the __del__ method, terminating")

        if self.terminate_mutex == None:
            return

        with self.terminate_mutex:
            self.need_to_terminate = True

        assert self._rcv_thread
        self._rcv_thread.join()

    def _run(self):
        while rclpy.ok():
            with self.terminate_mutex:
                if self.need_to_terminate:
                    break

            result = select.select([self.rx_queue_._reader], [], [], 0.5)
            if len(result[0]) > 0:
                try:
                    data = result[0][0].recv()
                    self._handle_rsp(data)
                except:
                    self.get_logger().info("Select did not return interface data")

        self.comm.Close()
        self.tx_queue_.close()
        self.rx_queue_.close()

    def _send_command(self, command):
        self.get_logger().info(f"Received command: {command.gp_cmd}, {command.gp_param}")
        cmds = [GENERAL_PURPOSE_CMD_ID, [command_ids[command.gp_cmd], command.gp_param]]
        self._add_command_to_queue(cmds)

    def _add_command_to_queue(self, command):
        cmd_bytes = generate_cmd_bytes(command)
        self.tx_queue_.put(cmd_bytes)

    def _update_rcv_frq(self):
        time_seconds = float(self.get_clock().now().nanoseconds) * 1e-9
        if self.start_frequency_samp:
            self.samp += 1
            self.summer += 1.0 / (time_seconds - self.last_rsp_rcvd)
            self.avg_freq = self.summer / self.samp
        self.last_rsp_rcvd = time_seconds

    def _handle_rsp(self, data_bytes):
        self._update_rcv_frq()
        if self.flush_rcvd_data:
           return
        
        valid_data, rsp_data = validate_response(data_bytes)

        if not valid_data:
            self.get_logger().error("Bad RMP packet")
            return

        if self.extracting_faultlog:
            if len(rsp_data) == NUMBER_OF_FAULTLOG_WORDS:
                self.extracting_faultlog = False
                try:
                    faultlog_msg = Faultlog()
                    faultlog_msg.data = rsp_data
                    self.faultlog_pub.publish(faultlog_msg)
                    self.get_logger().info("Faultlog Recieved")
                except Exception as e:
                    self.get_logger().error("Could not publish faultlog" + str(e))

        elif len(rsp_data) == NUMBER_OF_RMP_RSP_WORDS:
            self.rmp_data.status.parse(rsp_data[START_STATUS_BLOCK:END_STATUS_BLOCK])
            self.rmp_data.auxiliary_power.parse(rsp_data[START_AUX_POWER_BLOCK:END_AUX_POWER_BLOCK])
            self.rmp_data.propulsion.parse(rsp_data[START_PROPULSION_POWER_BLOCK:END_PROPULSION_POWER_BLOCK])
            self.rmp_data.dynamics.parse(rsp_data[START_DYNAMICS_BLOCK:END_DYNAMICS_BLOCK])
            self.rmp_data.config_param.parse(rsp_data[START_CONFIG_BLOCK:END_CONFIG_BLOCK])
            self.rmp_data.imu.parse_data(rsp_data[START_IMU_BLOCK:END_IMU_BLOCK])

            self.get_logger().debug("Feedback received from RMP")

    def _add_motion_command_to_queue(self, command):
        cmds = [MOTION_CMD_ID, [
            convert_float_to_u32(command.linear.x),
            convert_float_to_u32(command.linear.y),
            convert_float_to_u32(command.angular.z)
        ]]
        self._add_command_to_queue(cmds)

    def _add_config_command_to_queue(self, command):
        try:
            cmds = [GENERAL_PURPOSE_CMD_ID, [command_ids[command.id], command.value]]
            self._add_command_to_queue(cmds)
        except KeyError:
            self.get_logger().error("Invalid command key received, please verify yaml is correct")

    
    def set_params(self):
        
        """
        Create the configuration bitmap from the appropriate variables
        """
        config_bitmap = (((params["enable_audio"]^1) << AUDIO_SILENCE_REQUEST_SHIFT)|
                         ((params["motion_while_charging"]^1) << DISABLE_AC_PRESENT_CSI_SHIFT)|
                         (params["balace_gains"] << BALANCE_GAIN_SCHEDULE_SHIFT)|
                         (params["balance_mode_enabled"] << BALANCE_MODE_LOCKOUT_SHIFT) |
                         (params["vel_ctl_input_filter"] << VEL_CTL_FILTER_SHIFT) |
                         (params["yaw_ctl_input_filter"] << YAW_CTL_FILTER_SHIFT))
        
        """
        Define the configuration parameters for all the platforms
        """
        self.valid_config_cmd = [LOAD_MACH_CONFIG_CMD_ID,
                         [convert_float_to_u32(params["vel_limit_mps"]),
                          convert_float_to_u32(params["accel_limit_mps2"]),
                          convert_float_to_u32(params["decel_limit_mps2"]),
                          convert_float_to_u32(params["dtz_decel_limit_mps2"]),
                          convert_float_to_u32(params["yaw_rate_limit_rps"]),
                          convert_float_to_u32(params["yaw_accel_limit_rps2"]),
                          convert_float_to_u32(params["lateral_accel_limit_mps2"]),
                          convert_float_to_u32(params["tire_rolling_diameter_m"]),
                          convert_float_to_u32(params["wheel_base_length_m"]),
                          convert_float_to_u32(params["wheel_track_width_m"]),
                          convert_float_to_u32(params["gear_ratio"]),
                          config_bitmap]]
        
        self._add_command_to_queue(self.valid_config_cmd)
        self.get_logger().info("Sent config update command")
        
        cmd = [GENERAL_PURPOSE_CMD_ID,
                [GENERAL_PURPOSE_CMD_SET_TORQUE_LIMIT,
                convert_float_to_u32(params["torqe_limit"]/100.0)]]
        self._add_command_to_queue(cmd)


    def _continuous_data(self, start_cont):
        set_continuous = [GENERAL_PURPOSE_CMD_ID, [GENERAL_PURPOSE_CMD_SEND_CONTINUOUS_DATA, start_cont]]
        ret = False
        
        if start_cont:
            start_time = float(self.get_clock().now().nanoseconds) * 1e-9
            while ((self.get_clock().now().seconds_nanoseconds()[0] - start_time) < 3.0) and (self.rmp_data.status.init):
                self._add_command_to_queue(set_continuous)
                time.sleep(0.1)
            ret = not self.rmp_data.status.init
        else:
            start_time = float(self.get_clock().now().nanoseconds) * 1e-9
            while ((float(self.get_clock().now().nanoseconds) * 1e-9 - start_time) < 3.0) and not ret:
                self._add_command_to_queue(set_continuous)
                if ((float(self.get_clock().now().nanoseconds) * 1e-9 - self.last_rsp_rcvd) > 0.1):
                    ret = True
                time.sleep(0.2)
            self.rmp_data.status.init = True

        return ret
    
    def _extract_faultlog(self):
        start_time = self.get_clock().now().seconds_nanoseconds()[0]
        while ((self.get_clock().now().seconds_nanoseconds()[0] - start_time) < 3.0) and self.extracting_faultlog:
            self._add_command_to_queue([GENERAL_PURPOSE_CMD_ID, [GENERAL_PURPOSE_CMD_SEND_SP_FAULTLOG, 0]]) 
            time.sleep(0.5)
            
        return not self.extracting_faultlog
        

def main(args=None):
    rclpy.init(args=args)
    segway_driver = SegwayHardwareInterface()
    executor = MultiThreadedExecutor()
    rclpy.spin(segway_driver, executor)
    segway_driver.__del__()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

