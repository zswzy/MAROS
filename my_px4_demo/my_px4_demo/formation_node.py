# formation control
# event-triggered control
# useable node, based ont he formation_etm.py
# ONLY to be used with the launch file

# 运行节点时，可以直接传参数，例如：
# ros2 run my_px4_demo formation_node --ros-args -p i:=1

import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from sensor_msgs.msg import Temperature
from std_msgs.msg import Float64

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
from math import pi, atan2
from pymap3d import ned2geodetic, geodetic2ned
import numpy as np

LAT_REF = 48.614056
LON_REF = 2.424111
ALT_REF = 87.47

ADJ = [
    [0, 0, 0, 0],
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0]
]

# NEIGHBORS = [  # the i-th length is a list of neighbor index (starts from 1, 2, ...N)
#     [2, 4],
#     [1, 3],
#     [2, 4],
#     [3, 1]
# ]  

NEIGHBORS = [  # the i-th length is a list of neighbor index (starts from 1, 2, ...N)
    [],
    [1],
    [2],
    [3]
]  

# K = 0.5*np.array([1.6211, 3.8376])
# K = np.array([1.5, 2.5])
K = np.array([1.93, 4.2661])
P = np.array([  [1.7622, 1.9299],
                [1.9299, 4.2661]])
A = np.array([[0, 1],
              [0, 0]])
B = np.array([[0],
              [1]])
M1 = P @ B @ B.T @ P
M2 = P @ A + A.T @ P







Leader_ADJ = [1, 0, 0, 0]

# 三架 follower 的编队偏移（以 leader 的 NED 定义：北、东、下，单位：米）
NUM_FOLLOWER = 4
NUM_AGENT = NUM_FOLLOWER + 1
# FORMATION = [
#     (15.0, 0.0, 0.0),
#     (0.0, 15.0, 0.0),
#     (-15.0, 0.0, 0.0),
#     (0.0, -15.0, 0.0),
# ]

FORMATION = [
    (30.0, 30.0, 0.0),
    (-30.0, 30.0, 0.0),
    (-30.0, -30.0, 0.0),
    (30.0, -30.0, 0.0),
]

FOLLOWERS = ["px4_1", "px4_2", "px4_3", "px4_4"]

'''
Temperature message for event-triggered monitoring
float64 temperature          # value of theta - the clock-like variable
float64 variance             # =1 if triggered, =0 otherwise
'''

# math functions
def sat(x, upper, lower):
    return max(min(x, upper), lower)

# state class
class DroneState():
    def __init__(self):
        self.x = 0.0  # related to ref LLA
        self.y = 0.0
        self.z = 0.0
        self.x_local = 0.0
        self.y_local = 0.0
        self.z_local = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.lat = LAT_REF
        self.lon = LON_REF
        self.alt = ALT_REF

        self.x_sample = 0.0  # related to ref LLA
        self.y_sample = 0.0
        self.z_sample = 0.0
        self.vx_sample = 0.0
        self.vy_sample = 0.0
        self.vz_sample = 0.0

        self.yaw = 0.0

        

# main class
class FormationNode(Node):
    def __init__(self):
        super().__init__('formation_node')

        # 声明参数并给出默认值
        self.declare_parameter('i', 1) # i is the agent numbering (1,2,3,...,N)
        # px4_i
        # 读取参数
        self.i = self.get_parameter('i').get_parameter_value().integer_value

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        #
        self.connect_leader = (Leader_ADJ[self.i-1]==1)
        self.neighbor_table = NEIGHBORS[self.i-1]  # a list of neighbor index

        # ------------------------------------------------------------------
        # Create subscriptions
        self.status_sub = self.create_subscription(  # status
                VehicleStatus,
                f"/px4_{self.i}/fmu/out/vehicle_status_v1",
                self.vehicle_status_callback,
                qos_profile,
            )

        # index starts from 0 (leader), 1 (first neighbor ), 2(second neighbor ), ...
        # this shows the accesibility of the neighors state
        # self state
        self.create_subscription(
                VehicleLocalPosition,
                f"/px4_{self.i}/fmu/out/vehicle_local_position_v1",
                self.agent_local_position_callback,
                qos_profile,
            )

        self.create_subscription(
                VehicleGlobalPosition,
                f"/px4_{self.i}/fmu/out/vehicle_global_position",
                self.agent_global_position_callback,
                qos_profile,
            ) 
        

        # leader state
        if self.connect_leader:
            self.create_subscription(
                VehicleLocalPosition,
                '/fmu/out/vehicle_local_position_v1',
                self.leader_local_position_callback,
                qos_profile)
            
            self.create_subscription(
                VehicleGlobalPosition,
                '/fmu/out/vehicle_global_position',
                self.leader_global_position_callback,
                qos_profile)
        
        # neighbor state, sampled
        for j in range(len(self.neighbor_table)):  # j=0,1,...N_i-1 (number of neighbors)
            # j starts from 0 (follower)
            self.create_subscription(
                VehicleLocalPosition,
                f"/px4_{self.neighbor_table[j]}/fmu/out/vehicle_sampled_local_position",
                lambda msg, j=j: self.neighbor_sampled_local_position_callback(j, msg),
                qos_profile,
            )

            self.create_subscription(
                VehicleGlobalPosition,
                f"/px4_{self.neighbor_table[j]}/fmu/out/vehicle_sampled_global_position",
                lambda msg, j=j: self.neighbor_sampled_global_position_callback(j, msg),
                qos_profile,
            )


        # for adjusting theta_bar
        self.create_subscription(
            Float64,
            f"/etm/theta_bar",
            self.etm_adjuster_callback,
            qos_profile,
        )

        # for getting leader's yaw
        self.create_subscription(
            VehicleLocalPosition,
            f"fmu/out/vehicle_local_position_v1",
            self.leader_attitude_callback,
            qos_profile,
        )
            
        
            
        # ------------------------------------------------------------------
        # Create publishers, only for the followers
        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode, 
            f'/px4_{self.i}/fmu/in/offboard_control_mode', 
            qos_profile) 
        
        self.publisher_trajectory = self.create_publisher(
            TrajectorySetpoint, 
            f'/px4_{self.i}/fmu/in/trajectory_setpoint', 
            qos_profile)
        
        self.vehicle_command_publisher_ = self.create_publisher(
            VehicleCommand, 
            f"/px4_{self.i}/fmu/in/vehicle_command", 
            10)
        
        self.event_info_publisher_ = self.create_publisher(  # event-triggered monitoring
            Temperature, 
            f"/px4_{self.i}/event_info", 
            qos_profile)
        
        # publish sampled global position
        self.publisher_sampled_global_position = self.create_publisher(  # event-triggered monitoring
            VehicleGlobalPosition, 
            f"/px4_{self.i}/fmu/out/vehicle_sampled_global_position", 
            qos_profile)
        
        # publish sampled local position
        self.publisher_sampled_local_position = self.create_publisher(  # event-triggered monitoring
            VehicleLocalPosition, 
            f"/px4_{self.i}/fmu/out/vehicle_sampled_local_position", 
            qos_profile)

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

        # timer for print statistic info (IET, ...)
        self.print_timer = self.create_timer(1.0, self.info_callback)

        self.current_state = "IDLE"
        self.last_state = self.current_state
        self.arm_message = False
        self.flightCheck = False
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_ARMED  # ARMING_STATE_DISARMED=1, ARMING_STATE_ARMED=2
        self.failsafe = False
        self.myCnt = 0
        self.offboardMode = False
        self.offboard_start_time = 0
        self.event_number = 0

        self.theta_initial = 100
        self.theta = self.theta_initial

        self.yaw_des = 0.0
        self.leader_yaw = 0.0

        # true flight state
        self.agent_state = DroneState()
        if Leader_ADJ[self.i-1] == 1:
            self.leader_state = DroneState()
        self.neighbor_state = [DroneState() for _ in self.neighbor_table]


    # ---------------------------------------------------------------------------
    # def arm_message_callback(self, msg):
    #     self.arm_message = msg.data
    #     self.get_logger().info(f"Arm Message: {self.arm_message}")

    # ---------------------------------------------------------------------------
    #callback function that arms, takes off, and switches to offboard mode
    #implements a finite state machine
    def arm_timer_callback(self):
        # only for followers
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
                    self.offboard_start_time = int(Clock().now().nanoseconds / 1000)
                    self.get_logger().info(f"Loiter, Offboard")
                self.arm()

            case "OFFBOARD":
                if(not(self.flightCheck) or self.arm_state != VehicleStatus.ARMING_STATE_ARMED or self.failsafe == True):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Offboard, Flight Check Failed")
                self.state_offboard()
                self.arm()
                

        if (self.arm_state != VehicleStatus.ARMING_STATE_ARMED):
            self.arm_message = False

        if (self.last_state != self.current_state):
            self.last_state = self.current_state
            self.get_logger().info(f"{self.current_state}")

        self.myCnt += 1

    # ---------------------------------------------------------------------------
    # Arms the vehicle
    def arm(self):
        # i starts from 1 (follower)
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        # self.get_logger().info(f"uav [{i}] Arm command send")

    # ---------------------------------------------------------------------------
    # Takes off the vehicle to a user specified altitude (meters)
    def take_off(self):
        # i starts from 1 (follower)
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1 = 1.0, param7=5.0+ALT_REF) # param7 is altitude in meters
        self.get_logger().info(f"Takeoff command send")

    # ---------------------------------------------------------------------------
    def state_offboard(self):
        # i starts from 1 (follower)
        self.myCnt = 0
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
        # self.get_logger().info(f"uav [{i}]Offboard command send")
        self.offboardMode = True   

    # ---------------------------------------------------------------------------
    #receives and sets vehicle status values 
    def vehicle_status_callback(self, msg):
        # i starts from 0(leader)

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
    # receives self local position
    def agent_local_position_callback(self, msg):
        self.agent_state.x_local = msg.x
        self.agent_state.y_local = msg.y
        self.agent_state.z_local = msg.z
        self.agent_state.vx = msg.vx
        self.agent_state.vy = msg.vy
        self.agent_state.vz = msg.vz

    # ---------------------------------------------------------------------------
    # receives leader local position
    def leader_local_position_callback(self, msg):
        self.leader_state.x_local = msg.x
        self.leader_state.y_local = msg.y
        self.leader_state.z_local = msg.z
        self.leader_state.vx = msg.vx
        self.leader_state.vy = msg.vy
        self.leader_state.vz = msg.vz


    # ---------------------------------------------------------------------------
    # receives neighbor local position
    def neighbor_sampled_local_position_callback(self, j: int, msg):
        # j=0,1,,..N_i-1
        # self.neighbor_state[j].x_local = msg.x
        # self.neighbor_state[j].y_local = msg.y
        # self.neighbor_state[j].z_local = msg.z
        self.neighbor_state[j].vx_sample = msg.vx
        self.neighbor_state[j].vy_sample = msg.vy
        # self.neighbor_state[j].vz = msg.vz
            

    # ---------------------------------------------------------------------------
    # receives lself global position
    def agent_global_position_callback(self, msg):
        # leader
        self.agent_state.lon = msg.lon
        self.agent_state.lat = msg.lat
        self.agent_state.alt = msg.alt
        # translate to reference coord
        self.agent_state.x, self.agent_state.y, _ = geodetic2ned(self.agent_state.lat, self.agent_state.lon, 0, LAT_REF, LON_REF, 0)
        self.agent_state.z = self.agent_state.z_local

    # ---------------------------------------------------------------------------
    # receives leader global position
    def leader_global_position_callback(self, msg):
        # leader
        self.leader_state.lon = msg.lon
        self.leader_state.lat = msg.lat
        self.leader_state.alt = msg.alt
        # translate to reference coord
        self.leader_state.x, self.leader_state.y, _ = geodetic2ned(self.leader_state.lat, self.leader_state.lon, 0, LAT_REF, LON_REF, 0)
        self.leader_state.z = self.leader_state.z_local

    # ---------------------------------------------------------------------------
    # receives nonleader neighobor global position
    def neighbor_sampled_global_position_callback(self, j: int, msg):
        # j starts from 0,1,2,...,N_i-1
        self.neighbor_state[j].lon = msg.lon
        self.neighbor_state[j].lat = msg.lat
        # self.neighbor_state[j].alt = msg.alt
        # translate to reference coord
        self.neighbor_state[j].x_sample, self.neighbor_state[j].y_sample, _ = geodetic2ned(self.neighbor_state[j].lat, self.neighbor_state[j].lon, 0, LAT_REF, LON_REF, 0)
        # self.neighbor_state[j].z = self.neighbor_state[j].z_local

    # ---------------------------------------------------------------------------
    # receives theta_bar
    def etm_adjuster_callback(self, msg):
        self.theta_initial = msg.data
        self.get_logger().info(f"theta_bar has been set to {self.theta_initial}")

    def leader_attitude_callback(self, msg):
        self.leader_yaw = msg.heading


    # ---------------------------------------------------------------------------
    #publishes command to /fmu/in/vehicle_command
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        # i start from 1
        # node_agent = 1 (leader), 2(follower 1), 3, ...
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7    # altitude value in takeoff command
        msg.command = command  # command ID
        msg.target_system = 1+self.i  # system which should execute the command
        msg.target_component = 1  # component which should execute the command, 0 for all components
        msg.source_system = 1  # system sending the command
        msg.source_component = 1  # component sending the command
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.vehicle_command_publisher_.publish(msg)

    # ---------------------------------------------------------------------------
    #publishes event-triggered info to /{fol}/event_info
    def publish_event_info(self, theta=10, trigger=0):
        # i start from 1
        # node_agent = 1 (leader), 2(follower 1), 3, ...
        
        msg = Temperature()
        msg.temperature = float(theta)
        msg.variance = float(trigger)
        self.event_info_publisher_.publish(msg)

    # ---------------------------------------------------------------------------
    # publishes event-sampled global position lat, lon
    def publish_sampled_global_position(self):
        msg = VehicleGlobalPosition()
        msg.lat = self.agent_state.lat
        msg.lon = self.agent_state.lon
        self.publisher_sampled_global_position.publish(msg)

    # ---------------------------------------------------------------------------
    # publishes event-sampled local position vx, vy
    def publish_sampled_local_position(self):
        msg = VehicleLocalPosition()
        msg.vx = self.agent_state.vx
        msg.vy = self.agent_state.vy
        self.publisher_sampled_local_position.publish(msg)

    # ---------------------------------------------------------------------------
    #publishes offboard control modes and velocity as trajectory setpoints
    # main callback loop
    def cmdloop_callback(self):
        # publish offboard mode
        if(self.offboardMode == True):
            # Publish offboard control modes
            offboard_msg = OffboardControlMode()
            offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            offboard_msg.position = False
            offboard_msg.velocity = False
            offboard_msg.acceleration = True
            self.publisher_offboard_mode.publish(offboard_msg)        

        # determine event-triggered sample
        if(self.offboardMode == True):
            # measurement error
            # ex = self.agent_state.x_sample-self.agent_state.x
            # evx = self.agent_state.vx_sample-self.agent_state.vx
            # ey = self.agent_state.y_sample-self.agent_state.y
            # evy = self.agent_state.vy_sample-self.agent_state.vy
            # # pre-calculate command
            # _, _, zx, zy = self.control_event_triggered()

            trigger = 0
            self.event_sampling('y')
            # if ex**2+evx**2 + ey**2+evy**2 >= 0.05*(zx[0]**2+zx[1]**2+zy[0]**2+zy[1]**2): # event rule
            if self.event_triggered_rule():
            # if True:
                self.theta = self.theta_initial
                self.event_sampling('x')
                self.event_number += 1
                trigger = 1
                # publish sampled state
                self.publish_sampled_global_position()
                self.publish_sampled_local_position()

            self.publish_event_info(theta = self.theta, trigger = trigger)

                    

        # determine final control signals and publish
        if(self.offboardMode == True):
            # Create and publish TrajectorySetpoint message with NaN values for position and velocity
            z_des = -3-self.i*1
            ax_command, ay_command, _, _ = self.control_event_triggered()

            if self.agent_state.vx**2+ self.agent_state.vx**2 >= 0.1:
                alpha_yaw_des = 0.01
                self.yaw_des = alpha_yaw_des*atan2(self.agent_state.vy, self.agent_state.vx) + (1-alpha_yaw_des)*self.yaw_des

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
            trajectory_msg.yaw = float(0)
            trajectory_msg.yawspeed = float('nan')

            self.publisher_trajectory.publish(trajectory_msg)

    # ---------------------------------------------------------------------------
    # event-rule
    def event_triggered_rule(self):
        # output: true (trigger) or false
        tau = 0.01
        kappa = 2
        delta_max = 1
        delta_min = 1
        alpha = 4.5433

        ex = self.agent_state.x_sample-self.agent_state.x
        evx = self.agent_state.vx_sample-self.agent_state.vx
        # ey = self.agent_state.y_sample-self.agent_state.y
        # evy = self.agent_state.vy_sample-self.agent_state.vy
        # pre-calculate command
        _, _, zx, _ = self.control_event_triggered()
        e = np.array([[ex],[evx]])
        z = np.array(zx).reshape((2,1))

        if ex**2+evx**2 <=0.0001:
            omega = -tau
        else:
            omega = -( (kappa/delta_min-delta_min)*np.linalg.norm(e)**2 + e.T @  (2*delta_max*M1+self.theta*M2) @ e) / (e.T @ P @ e) \
            - (2*(1-self.theta)*e.T @ M1 @ z)/(e.T @ P @ e) - delta_min/(e.T @ P @ e) * (1/kappa - 1/delta_max**2)* np.linalg.norm(z)**2
            omega = omega[0]

            omega = -1000 - self.theta*alpha \
                - (2*(1-self.theta)*e.T @ M1 @ z)/(e.T @ P @ e) - delta_min/(e.T @ P @ e) * (1/kappa - 1/delta_max**2)* np.linalg.norm(z)**2
            omega = omega[0]

        self.theta = self.theta + omega*0.02

        if self.theta <= 0:
            return True
        else:
            return False


    # ---------------------------------------------------------------------------
    # print flight infomation
    def info_callback(self):
        if(self.offboardMode == True) and (self.event_number!=0) :
            self.get_logger().info(f'average IET: {(Clock().now().nanoseconds / 1000 - self.offboard_start_time)/1000000/self.event_number:.3f} s')

    # ---------------------------------------------------------------------------
    # event-triggered distributed formation control
    def control_event_triggered(self):
        # in the basis of control_formation_distributed, use event-based state
        # i starts from 1 (follower 1)
        fx, fy, _ = FORMATION[self.i-1]

        # z = sum aij*(xj-xi-(fj-fi)) + di*(x0-xi+fi)
        zx = [0, 0]
        zy = [0, 0]
        for j, fol in enumerate(self.neighbor_table):
            # j: neighbors order: 0, 1,2,...,Ni-1
            # fol: agent index
            fxj, fyj, _ = FORMATION[fol-1]
            # print(f"fxj: {fxj}, fyj: {fyj}")
            # print(f"neighbor: {j}-th neighbor, index={fol}")
            # print(f"neighbor x: {self.neighbor_state[j].x_sample}, self.x: {self.agent_state.x_sample}")
            zx[0] = zx[0] + (self.neighbor_state[j].x_sample-self.agent_state.x_sample-(fxj-fx))
            zx[1] = zx[1] + (self.neighbor_state[j].vx_sample-self.agent_state.vx_sample)
            zy[0] = zy[0] + (self.neighbor_state[j].y_sample-self.agent_state.y_sample-(fyj-fy))
            zy[1] = zy[1] + (self.neighbor_state[j].vy_sample-self.agent_state.vy_sample)

        # leader's connectivity
        # no event sampling for leader
        if self.connect_leader:
            # print(f"leader x: {self.leader_state.x}, self x: {self.agent_state.x_sample}")
            zx[0] = zx[0] + (self.leader_state.x-self.agent_state.x_sample+fx)
            zx[1] = zx[1] + (self.leader_state.vx-self.agent_state.vx_sample)
            zy[0] = zy[0] + (self.leader_state.y-self.agent_state.y_sample+fy)
            zy[1] = zy[1] + (self.leader_state.vy-self.agent_state.vy_sample)



        ax_command = K[0]*zx[0] + K[1]*zx[1]
        ay_command = K[0]*zy[0] + K[1]*zy[1]

        ax_command = sat(ax_command, 5, -5)
        ay_command = sat(ay_command, 5, -5)

        # ax_command = sat(ax_command , 1,-1)
        # ay_command = sat(ay_command, 1,-1)
        return ax_command, ay_command, zx, zy

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
    # distributed formation control
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
        
    # ---------------------------------------------------------------------------
    # event-triggered sampling
    def event_sampling(self):
        self.agent_state.x_sample = self.agent_state.x
        self.agent_state.vx_sample = self.agent_state.vx
        self.agent_state.y_sample = self.agent_state.y
        self.agent_state.vy_sample = self.agent_state.vy

    # event-triggered sampling
    def event_sampling(self, axis='x'):
        if axis == 'x':
            self.agent_state.x_sample = self.agent_state.x
            self.agent_state.vx_sample = self.agent_state.vx
        elif axis == 'y':
            self.agent_state.y_sample = self.agent_state.y
            self.agent_state.vy_sample = self.agent_state.vy



    


def main(args=None):
    rclpy.init(args=args)

    formation_node = FormationNode()

    rclpy.spin(formation_node)

    formation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()