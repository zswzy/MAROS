import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition, VehicleGlobalPosition
from geometry_msgs.msg import Twist, Vector3
from math import pi
from std_msgs.msg import Bool
import time
from math import pi
from pymap3d import ned2geodetic, geodetic2ned

LAT_REF = 48.614056
LON_REF = 2.424111
ALT_REF = 87.47

# math functions
def sat(x, upper, lower):
    return max(min(x, upper), lower)

# main class
class ArmControl(Node):
    def __init__(self):
        super().__init__('arm_control')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create subscriptions
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            self.vehicle_status_callback,
            qos_profile)
        
        self.status_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.vehicle_local_position_callback,
            qos_profile)
        
        self.status_sub = self.create_subscription(
            VehicleGlobalPosition,
            '/fmu/out/vehicle_global_position',
            self.vehicle_global_position_callback,
            qos_profile)
        
        # self.offboard_velocity_sub = self.create_subscription(
        #     Twist,
        #     '/offboard_velocity_cmd',
        #     self.offboard_velocity_callback,
        #     qos_profile)
        
        # self.attitude_sub = self.create_subscription(
        #     VehicleAttitude,
        #     '/fmu/out/vehicle_attitude',
        #     self.attitude_callback,
        #     qos_profile)
        
        # self.my_bool_sub = self.create_subscription(
        #     Bool,
        #     '/arm_message',
        #     self.arm_message_callback,
        #     qos_profile)

        #Create publishers
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        # self.publisher_velocity = self.create_publisher(Twist, '/fmu/in/setpoint_velocity/cmd_vel_unstamped', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)

        #creates callback function for the arm timer
        # period is arbitrary, just should be more than 2Hz
        arm_timer_period = 0.1 # seconds
        self.arm_timer_ = self.create_timer(arm_timer_period, self.arm_timer_callback)

        # creates callback function for the command loop
        # period is arbitrary, just should be more than 2Hz. Because live controls rely on this, a higher frequency is recommended
        # commands in cmdloop_callback won't be executed if the vehicle is not in offboard mode
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)



        self.current_state = "IDLE"
        self.last_state = self.current_state
        self.arm_message = False
        self.flightCheck = False
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_ARMED  # ARMING_STATE_DISARMED=1, ARMING_STATE_ARMED=2
        self.failsafe = False
        self.myCnt = 0
        self.offboardMode = False

        # flight state
        self.x_local = 0
        self.y_local = 0
        self.z_local = 0
        self.x = 0  # in reference coord
        self.y = 0
        self.z = 0
        self.vx = 0
        self.vy = 0
        self.vz = 0


    # ---------------------------------------------------------------------------
    # def arm_message_callback(self, msg):
    #     self.arm_message = msg.data
    #     self.get_logger().info(f"Arm Message: {self.arm_message}")

    # ---------------------------------------------------------------------------
    #callback function that arms, takes off, and switches to offboard mode
    #implements a finite state machine
    def arm_timer_callback(self):

        match self.current_state:
            case "IDLE":
                # if(self.flightCheck and self.arm_state == VehicleStatus.ARMING_STATE_DISARMED):
                if(self.flightCheck):
                    self.current_state = "ARMING"
                    self.get_logger().info(f"Arming")

            case "ARMING":
                if(not(self.flightCheck)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Arming, Flight Check Failed")
                elif(self.arm_state == VehicleStatus.ARMING_STATE_ARMED and self.myCnt > 10):
                    self.current_state = "TAKEOFF"
                    self.get_logger().info(f"Arming, Takeoff")
                self.arm() #send arm command

            case "TAKEOFF":
                if(not(self.flightCheck)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Takeoff, Flight Check Failed")
                elif(self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF):
                    self.current_state = "LOITER"
                    self.get_logger().info(f"Takeoff, Loiter")
                self.arm() #send arm command
                self.take_off() #send takeoff command

            # waits in this state while taking off, and the 
            # moment VehicleStatus switches to Loiter state it will switch to offboard
            case "LOITER": 
                if(not(self.flightCheck)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Loiter, Flight Check Failed")
                elif(self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER):
                    self.current_state = "OFFBOARD"
                    self.get_logger().info(f"Loiter, Offboard")
                self.arm()

            case "OFFBOARD":
                if(not(self.flightCheck) or self.arm_state != VehicleStatus.ARMING_STATE_ARMED or self.failsafe == True):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Offboard, Flight Check Failed")
                self.state_offboard()
                self.arm()
                

        if(self.arm_state != VehicleStatus.ARMING_STATE_ARMED):
            self.arm_message = False

        if (self.last_state != self.current_state):
            self.last_state = self.current_state
            self.get_logger().info(self.current_state)

        self.myCnt += 1

    # ---------------------------------------------------------------------------
    # Arms the vehicle
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        # self.get_logger().info("Arm command send")

    # ---------------------------------------------------------------------------
    # Takes off the vehicle to a user specified altitude (meters)
    def take_off(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1 = 1.0, param7=5.0+ALT_REF) # param7 is altitude in meters
        self.get_logger().info("Takeoff command send")

    # ---------------------------------------------------------------------------
    def state_offboard(self):
        self.myCnt = 0
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
        # self.get_logger().info("Offboard command send")
        self.offboardMode = True   

    # ---------------------------------------------------------------------------
    #receives and sets vehicle status values 
    def vehicle_status_callback(self, msg):

        if (msg.nav_state != self.nav_state):
            self.get_logger().info(f"NAV_STATUS: {msg.nav_state}")
        
        if (msg.arming_state != self.arm_state):
            self.get_logger().info(f"ARM STATUS: {msg.arming_state}")

        if (msg.failsafe != self.failsafe):
            self.get_logger().info(f"FAILSAFE: {msg.failsafe}")
        
        if (msg.pre_flight_checks_pass != self.flightCheck):
            self.get_logger().info(f"FlightCheck: {msg.pre_flight_checks_pass}")

        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.failsafe = msg.failsafe
        self.flightCheck = msg.pre_flight_checks_pass

    # ---------------------------------------------------------------------------
    # receives local position
    def vehicle_local_position_callback(self, msg):
        self.x_local = msg.x
        self.y_local = msg.y
        self.z_local = msg.z
        self.vx = msg.vx
        self.vy = msg.vy
        self.vz = msg.vz

    # ---------------------------------------------------------------------------
    # receives global position
    def vehicle_global_position_callback(self, msg):
        self.lon = msg.lon
        self.lat = msg.lat
        self.alt = msg.alt
        # translate to reference coord
        self.x, self.y, _ = geodetic2ned(self.lat, self.lon, 0, LAT_REF, LON_REF, 0)
        self.z = self.z_local

    # ---------------------------------------------------------------------------
    #publishes command to /fmu/in/vehicle_command
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7    # altitude value in takeoff command
        msg.command = command  # command ID
        msg.target_system = 1  # system which should execute the command
        msg.target_component = 1  # component which should execute the command, 0 for all components
        msg.source_system = 1  # system sending the command
        msg.source_component = 1  # component sending the command
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.vehicle_command_publisher_.publish(msg)

    # ---------------------------------------------------------------------------
    #publishes offboard control modes and velocity as trajectory setpoints
    def cmdloop_callback(self):
        if(self.offboardMode == True):
            # Publish offboard control modes
            offboard_msg = OffboardControlMode()
            offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            offboard_msg.position = False
            offboard_msg.velocity = False
            offboard_msg.acceleration = True
            self.publisher_offboard_mode.publish(offboard_msg)            

            # Compute velocity in the world frame
            # cos_yaw = np.cos(self.trueYaw)
            # sin_yaw = np.sin(self.trueYaw)
            # velocity_world_x = (self.velocity.x * cos_yaw - self.velocity.y * sin_yaw)
            # velocity_world_y = (self.velocity.x * sin_yaw + self.velocity.y * cos_yaw)

            # Create and publish TrajectorySetpoint message with NaN values for position and acceleration
            # target latitude and longitude
            # change this, if in leader-following mode, to the dynamic target lat/lon
            lat_des = 48.614219
            lon_des = 2.423866
            # x_error, y_error, _ = geodetic2ned(lat_des, lon_des, 0, self.lat, self.lon, 0)
            x_des, y_des, _ = geodetic2ned(lat_des, lon_des, 0, LAT_REF, LON_REF, 0)

            # x_des = -10
            # y_des = 10
            z_des = -10
            # vx_command = sat(0.4*(x_des-self.x),1,-1)
            # vy_command = sat(0.4*(y_des-self.y),1,-1)
            # vz_command = sat(0.1*(z_des-self.z),0.5,-0.5)
            ax_command = sat(1.5*(x_des-self.x) + 2.5*(-self.vx), 1,-1)
            ay_command = sat(1.5*(y_des-self.y) + 2.5*(-self.vy), 1,-1)
            # ax_command = sat(1.5*x_error + 2.5*(-self.vx), 1,-1)
            # ay_command = sat(1.5*y_error + 2.5*(-self.vy), 1,-1)
            
            # print(ax_command)
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            trajectory_msg.velocity[0] = float('nan')
            trajectory_msg.velocity[1] = float('nan')
            trajectory_msg.velocity[2] = float('nan')
            trajectory_msg.position[0] = float('nan')
            trajectory_msg.position[1] = float('nan')
            trajectory_msg.position[2] = z_des
            trajectory_msg.acceleration[0] = ax_command
            trajectory_msg.acceleration[1] = ay_command
            trajectory_msg.acceleration[2] = float('nan')
            trajectory_msg.yaw = pi
            trajectory_msg.yawspeed = float('nan')

            self.publisher_trajectory.publish(trajectory_msg)




def main(args=None):
    rclpy.init(args=args)

    arm_control = ArmControl()

    rclpy.spin(arm_control)

    arm_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()