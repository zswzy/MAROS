# formation control
# demo file, for test

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

ADJ = [
    [0, 0, 0, 0],
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0]
]

Leader_ADJ = [1, 0, 0, 0]

# 三架 follower 的编队偏移（以 leader 的 NED 定义：北、东、下，单位：米）
NUM_FOLLOWER = 4
NUM_AGENT = NUM_FOLLOWER + 1
FORMATION = [
    (3.0, 0.0, 0.0),
    (-3.0, 0.0, 0.0),
    (0.0, 3.0, 0.0),
    (0.0, -3.0, 0.0),
]

FOLLOWERS = ["px4_1", "px4_2", "px4_3", "px4_4"]

# math functions
def sat(x, upper, lower):
    return max(min(x, upper), lower)

# main class
class FormationDemo(Node):
    def __init__(self):
        super().__init__('formation_demo')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ------------------------------------------------------------------
        # Create subscriptions
        # index starts from 0 (leader), 1 (follower ), 2, ...
        self.status_sub = [self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            lambda msg, i=0: self.vehicle_status_callback(i, msg),
            qos_profile) ]
        
        self.local_position_sub = [self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            lambda msg, i=0: self.vehicle_local_position_callback(i, msg),
            qos_profile)]
        
        self.global_position_sub = [self.create_subscription(
            VehicleGlobalPosition,
            '/fmu/out/vehicle_global_position',
            lambda msg, i=0: self.vehicle_global_position_callback(i, msg),
            qos_profile)]
        
        for i in range(1, NUM_AGENT):
            # i starts from 1 (follower)
            self.status_sub.append( self.create_subscription(
                VehicleStatus,
                f"/px4_{i}/fmu/out/vehicle_status_v1",
                lambda msg, i=i: self.vehicle_status_callback(i, msg),
                qos_profile,
            ) )

            self.local_position_sub.append( self.create_subscription(
                VehicleLocalPosition,
                f"/px4_{i}/fmu/out/vehicle_local_position_v1",
                lambda msg, i=i: self.vehicle_local_position_callback(i, msg),
                qos_profile,
            ) )

            self.global_position_sub.append( self.create_subscription(
                VehicleGlobalPosition,
                f"/px4_{i}/fmu/out/vehicle_global_position",
                lambda msg, i=i: self.vehicle_global_position_callback(i, msg),
                qos_profile,
            ) )
            
        
            
        # ------------------------------------------------------------------
        # Create publishers, only for the followers
        self.publisher_offboard_mode = [ self.create_publisher(
            OffboardControlMode, 
            f'/{fol}/fmu/in/offboard_control_mode', 
            qos_profile) for fol in FOLLOWERS ]
        
        self.publisher_trajectory = [ self.create_publisher(
            TrajectorySetpoint, 
            f'/{fol}/fmu/in/trajectory_setpoint', 
            qos_profile) for fol in FOLLOWERS]
        
        self.vehicle_command_publisher_ = [ self.create_publisher(
            VehicleCommand, 
            f"/{fol}/fmu/in/vehicle_command", 
            10) for fol in FOLLOWERS]

        # ------------------------------------------------------------------
        # creates callback function for the arm timer
        # period is arbitrary, just should be more than 2Hz
        arm_timer_period = 0.05 # seconds
        self.arm_timer_ = self.create_timer(arm_timer_period, self.arm_timer_callback) # arm for all followers

        # creates callback function for the command loop
        # period is arbitrary, just should be more than 2Hz. Because live controls rely on this, a higher frequency is recommended
        # commands in cmdloop_callback won't be executed if the vehicle is not in offboard mode
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)  # command for all followers



        self.current_state = ["IDLE" for _ in range(NUM_AGENT)]
        self.last_state = self.current_state
        self.arm_message = [False for _ in range(NUM_AGENT)]
        self.flightCheck = [False for f_ in range(NUM_AGENT)]
        self.nav_state = [VehicleStatus.NAVIGATION_STATE_MAX for _ in range(NUM_AGENT)]
        self.arm_state = [VehicleStatus.ARMING_STATE_ARMED for _ in range(NUM_AGENT)]  # ARMING_STATE_DISARMED=1, ARMING_STATE_ARMED=2
        self.failsafe = [False for _ in range(NUM_AGENT)]
        self.myCnt = [0 for _ in range(NUM_AGENT)]
        self.offboardMode = [False for _ in range(NUM_AGENT)]

        # flight state
        self.x_local = [0 for _ in range(NUM_AGENT)]
        self.y_local = [0 for _ in range(NUM_AGENT)]
        self.z_local = [0 for _ in range(NUM_AGENT)]
        self.x = [0 for _ in range(NUM_AGENT)]  # related to ref LLA
        self.y = [0 for _ in range(NUM_AGENT)]
        self.z = [0 for _ in range(NUM_AGENT)]
        self.vx = [0 for _ in range(NUM_AGENT)]
        self.vy = [0 for _ in range(NUM_AGENT)]
        self.vz = [0 for _ in range(NUM_AGENT)]
        self.lat = [0 for _ in range(NUM_AGENT)]
        self.lon = [0 for _ in range(NUM_AGENT)]
        self.alt = [0 for _ in range(NUM_AGENT)]


    # ---------------------------------------------------------------------------
    # def arm_message_callback(self, msg):
    #     self.arm_message = msg.data
    #     self.get_logger().info(f"Arm Message: {self.arm_message}")

    # ---------------------------------------------------------------------------
    #callback function that arms, takes off, and switches to offboard mode
    #implements a finite state machine
    def arm_timer_callback(self):
        # only for followers
        for i in range(1, NUM_AGENT):
            # i starts from 1 (follower 1)
            match self.current_state[i]:
                case "IDLE":
                    # if(self.flightCheck and self.arm_state == VehicleStatus.ARMING_STATE_DISARMED):
                    if(self.flightCheck[i]):
                        self.current_state[i] = "ARMING"
                        self.get_logger().info(f"uav [{i}] Arming")

                case "ARMING":
                    if(not(self.flightCheck[i])):
                        self.current_state[i] = "IDLE"
                        self.get_logger().info(f"uav [{i}] Arming, Flight Check Failed")
                    elif(self.arm_state[i] == VehicleStatus.ARMING_STATE_ARMED and self.myCnt[i] > 10):
                        self.current_state[i] = "TAKEOFF"
                        self.get_logger().info(f"uav [{i}] Arming, Takeoff")
                    self.arm(i) #send arm command

                case "TAKEOFF":
                    if(not(self.flightCheck[i])):
                        self.current_state[i] = "IDLE"
                        self.get_logger().info(f"uav [{i}] Takeoff, Flight Check Failed")
                    elif(self.nav_state[i] == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF):
                        self.current_state[i] = "LOITER"
                        self.get_logger().info(f"uav [{i}] Takeoff, Loiter")
                    self.arm(i) #send arm command
                    self.take_off(i) #send takeoff command

                # waits in this state while taking off, and the 
                # moment VehicleStatus switches to Loiter state it will switch to offboard
                case "LOITER": 
                    if(not(self.flightCheck[i])):
                        self.current_state[i] = "IDLE"
                        self.get_logger().info(f"uav [{i}] Loiter, Flight Check Failed")
                    elif(self.nav_state[i] == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER):
                        self.current_state[i] = "OFFBOARD"
                        self.get_logger().info(f"uav [{i}] Loiter, Offboard")
                    self.arm(i)

                case "OFFBOARD":
                    if(not(self.flightCheck[i]) or self.arm_state[i] != VehicleStatus.ARMING_STATE_ARMED or self.failsafe[i] == True):
                        self.current_state[i] = "IDLE"
                        self.get_logger().info(f"uav [{i}] Offboard, Flight Check Failed")
                    self.state_offboard(i)
                    self.arm(i)
                    

            if(self.arm_state[i] != VehicleStatus.ARMING_STATE_ARMED):
                self.arm_message[i] = False

            if (self.last_state[i] != self.current_state[i]):
                self.last_state[i] = self.current_state[i]
                self.get_logger().info(f"uav [{i}] {self.current_state}")

            self.myCnt[i] += 1

    # ---------------------------------------------------------------------------
    # Arms the vehicle
    def arm(self, i):
        # i starts from 1 (follower)
        self.publish_vehicle_command(i, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        # self.get_logger().info(f"uav [{i}] Arm command send")

    # ---------------------------------------------------------------------------
    # Takes off the vehicle to a user specified altitude (meters)
    def take_off(self, i):
        # i starts from 1 (follower)
        self.publish_vehicle_command(i, VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1 = 1.0, param7=5.0+ALT_REF) # param7 is altitude in meters
        self.get_logger().info(f"uav [{i}] Takeoff command send")

    # ---------------------------------------------------------------------------
    def state_offboard(self, i):
        # i starts from 1 (follower)
        self.myCnt[i] = 0
        self.publish_vehicle_command(i, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
        # self.get_logger().info(f"uav [{i}]Offboard command send")
        self.offboardMode[i] = True   

    # ---------------------------------------------------------------------------
    #receives and sets vehicle status values 
    def vehicle_status_callback(self, i:int, msg):
        # i starts from 0(leader)

        if (msg.nav_state != self.nav_state[i]):
            self.get_logger().info(f"uav [{i}] NAV_STATUS: {msg.nav_state}")
        
        if (msg.arming_state != self.arm_state[i]):
            self.get_logger().info(f"uav [{i}] ARM STATUS: {msg.arming_state}")

        if (msg.failsafe != self.failsafe[i]):
            self.get_logger().info(f"uav [{i}] FAILSAFE: {msg.failsafe}")
        
        if (msg.pre_flight_checks_pass != self.flightCheck[i]):
            self.get_logger().info(f"uav [{i}] FlightCheck: {msg.pre_flight_checks_pass}")

        self.nav_state[i] = msg.nav_state
        self.arm_state[i] = msg.arming_state
        self.failsafe[i] = msg.failsafe
        self.flightCheck[i] = msg.pre_flight_checks_pass

    # ---------------------------------------------------------------------------
    # receives local position
    def vehicle_local_position_callback(self, i: int, msg):
        # i starts from 0 (leader), 1, ...
        self.x_local[i] = msg.x
        self.y_local[i] = msg.y
        self.z_local[i] = msg.z
        self.vx[i] = msg.vx
        self.vy[i] = msg.vy
        self.vz[i] = msg.vz

    # ---------------------------------------------------------------------------
    # receives global position
    def vehicle_global_position_callback(self, i: int, msg):
        # i starts from 0 (leader), 1, ...
        self.lon[i] = msg.lon
        self.lat[i] = msg.lat
        self.alt[i] = msg.alt
        # translate to reference coord
        self.x[i], self.y[i], _ = geodetic2ned(self.lat[i], self.lon[i], 0, LAT_REF, LON_REF, 0)
        self.z[i] = self.z_local[i]


    # ---------------------------------------------------------------------------
    #publishes command to /fmu/in/vehicle_command
    def publish_vehicle_command(self, i, command, param1=0.0, param2=0.0, param7=0.0):
        # i start from 1
        # node_agent = 1 (leader), 2(follower 1), 3, ...
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7    # altitude value in takeoff command
        msg.command = command  # command ID
        msg.target_system = 1+i  # system which should execute the command
        msg.target_component = 1  # component which should execute the command, 0 for all components
        msg.source_system = 1  # system sending the command
        msg.source_component = 1  # component sending the command
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.vehicle_command_publisher_[i-1].publish(msg)

    # ---------------------------------------------------------------------------
    #publishes offboard control modes and velocity as trajectory setpoints
    def cmdloop_callback(self):
        for i in range(1, NUM_AGENT):
            if(self.offboardMode[i] == True):
                # Publish offboard control modes
                offboard_msg = OffboardControlMode()
                offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
                offboard_msg.position = False
                offboard_msg.velocity = True
                offboard_msg.acceleration = False
                self.publisher_offboard_mode[i-1].publish(offboard_msg)            

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
                # x_des, y_des, _ = geodetic2ned(lat_des, lon_des, 0, LAT_REF, LON_REF, 0)

                # x_des = -10
                # y_des = 10
                z_des = -5-i*1
                # vx_command = sat(0.4*(x_des-self.x),1,-1)
                # vy_command = sat(0.4*(y_des-self.y),1,-1)
                # vz_command = sat(0.1*(z_des-self.z),0.5,-0.5)
                # ax_command = sat(1.5*(x_des-self.x[i]) + 2.5*(-self.vx[i]), 1,-1)
                # ay_command = sat(1.5*(y_des-self.y[i]) + 2.5*(-self.vy[i]), 1,-1)
                # ax_command = sat(1.5*x_error + 2.5*(-self.vx), 1,-1)
                # ay_command = sat(1.5*y_error + 2.5*(-self.vy), 1,-1)
                # ax_command, ay_command = self.control_fixed_point(i, lat_des, lon_des)
                ax_command, ay_command = self.control_formation_centralized(i)
                ax_command, ay_command = self.control_formation_distributed(i)


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

                self.publisher_trajectory[i-1].publish(trajectory_msg)

    # ---------------------------------------------------------------------------
    # control follower to the a fixed point
    def control_fixed_point(self, i, lat_des, lon_des):
        # i starts from 1 (follower 1)
        x_des, y_des, _ = geodetic2ned(lat_des, lon_des, 0, LAT_REF, LON_REF, 0)
        ax_command = sat(1.5*(x_des-self.x[i]) + 2.5*(-self.vx[i]), 1,-1)
        ay_command = sat(1.5*(y_des-self.y[i]) + 2.5*(-self.vy[i]), 1,-1)
        return ax_command, ay_command
    
    # ---------------------------------------------------------------------------
    # centralized formation control
    def control_formation_centralized(self, i):
        # i starts from 1 (follower 1)
        fx, fy, fz = FORMATION[i-1]

        ax_command = 1.5*(self.x[0]+fx-self.x[i]) + 2.5*(self.vx[0]-self.vx[i])
        ay_command = 1.5*(self.y[0]+fy-self.y[i]) + 2.5*(self.vy[0]-self.vy[i])

        # ax_command = sat(ax_command , 1,-1)
        # ay_command = sat(ay_command, 1,-1)
        return ax_command, ay_command
    
    # ---------------------------------------------------------------------------
    # centralized formation control
    def control_formation_distributed(self, i):
        # i starts from 1 (follower 1)
        fx, fy, _ = FORMATION[i-1]

        # z = sum aij*(xj-xi-(fj-fi)) + di*(x0-xi+fi)
        zx = [0, 0]
        zy = [0, 0]
        for j in range(1, NUM_AGENT):
            if ADJ[i-1][j-1] != 0:
                fxj, fyj, _ = FORMATION[j-1]
                zx[0] = zx[0] + (self.x[j]-self.x[i]-(fxj-fx))
                zx[1] = zx[1] + (self.vx[j]-self.vx[i])
                zy[0] = zy[0] + (self.y[j]-self.y[i]-(fyj-fy))
                zy[1] = zy[1] + (self.vy[j]-self.vy[i])

        # leader's connectivity
        if Leader_ADJ[i-1] != 0:
            zx[0] = zx[0] + (self.x[0]-self.x[i]+fx)
            zx[1] = zx[1] + (self.vx[0]-self.vx[i])
            zy[0] = zy[0] + (self.y[0]-self.y[i]+fy)
            zy[1] = zy[1] + (self.vy[0]-self.vy[i])

        ax_command = 1.5*zx[0] + 2.5*zx[1]
        ay_command = 1.5*zy[0] + 2.5*zy[1]

        # ax_command = sat(ax_command , 1,-1)
        # ay_command = sat(ay_command, 1,-1)
        return ax_command, ay_command
        
        


def main(args=None):
    rclpy.init(args=args)

    formation_demo = FormationDemo()

    rclpy.spin(formation_demo)

    formation_demo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()