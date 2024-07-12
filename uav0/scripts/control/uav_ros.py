#! /usr/bin/python3
import rospy
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32MultiArray

from control.utils import *


class UAV_ROS:
    def __init__(self,
                 m: float = 1.5,
                 dt: float = 0.01,
                 time_max: float = 30.,
                 pos0: np.ndarray = np.zeros(3),
                 offset: np.ndarray = np.zeros(3),
                 group='/uav0',
                 use_ros_param: bool = False,
                 name: str = '~uav0_parameters'):
        self.g = 9.8
        self.kt = 1e-3
        if use_ros_param:
            _p = rospy.get_param(name)
            self.m = _p['m']
            self.pos0 = np.array(_p['pos0'])
            self.offset = np.array(_p['offset'])
            self.dt = _p['dt']
            self.time_max = _p['time_max']
            self.group = _p['group']
        else:
            self.m = m  # 无人机质量
            self.pos0 = pos0
            self.offset = offset
            self.dt = dt
            self.time_max = time_max  # 每回合最大时间
            self.group = group
        
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.att = np.zeros(3)
        self.pqr = np.zeros(3)
        
        self.n = 0  # 记录走过的拍数
        self.time = 0.  # 当前时间
        
        '''control'''
        self.throttle = self.m * self.g  # 油门
        self.phi_d = 0.
        self.theta_d = 0.
        '''control'''
        
        self.current_state = State()  # monitor uav status
        self.ctrl_param = Float32MultiArray(data=[0., 0., 0., 0., 0., 0., 0., 0., 0.])  # 9 维
        self.nn_input = Float32MultiArray(data=[0., 0., 0., 0., 0., 0.]) # 6 维
        self.pose = PoseStamped()  # publish offboard [x_d y_d z_d] cmd
        self.uav_odom = Odometry()  # subscribe uav state x y z vx vy vz phi theta psi p q r
        self.ctrl_cmd = AttitudeTarget()  # publish offboard expected [phi_d theta_d psi_d throttle] cmd
        self.voltage = 11.4  # subscribe voltage from the battery
        self.global_flag = 0  # UAV working mode monitoring
        # 0: connect to onboard computer, arm, load parameters, prepare
        # 1: approaching and initialization
        # 2: control by FNTSMC ([phi_d theta_d psi_d throttle])
        # 3: finish and switch OFFBOARD to position
        
        self.state_sub = rospy.Subscriber(self.group + "/mavros/state", State, callback=self.state_cb)
        self.ctrl_param_sub = rospy.Subscriber(self.group + "/ctrl_param", Float32MultiArray, callback=self.ctrl_param_cb)
        
        self.uav_vel_sub = rospy.Subscriber(self.group + "/mavros/local_position/odom", Odometry, callback=self.uav_odom_cb)
        self.uav_battery_sub = rospy.Subscriber(self.group + "/mavros/battery", BatteryState, callback=self.uav_battery_cb)
        '''topic subscribe'''
        
        self.local_pos_pub = rospy.Publisher(self.group + "/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.nn_input_state_pub = rospy.Publisher(self.group + "/nn_input_rl", Float32MultiArray, queue_size=10)
        self.uav_att_throttle_pub = rospy.Publisher(self.group + "/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=10)
        '''Publish 位置指令给 UAV'''
        
        '''arming service'''
        rospy.wait_for_service(self.group + "/mavros/cmd/arming")  # 等待解锁电机的 service 建立
        self.arming_client = rospy.ServiceProxy(self.group + "/mavros/cmd/arming", CommandBool)
        
        '''working mode service'''
        rospy.wait_for_service(self.group + "/mavros/set_mode")  # 等待设置 UAV 工作模式的 service 建立
        self.set_mode_client = rospy.ServiceProxy(self.group + "/mavros/set_mode", SetMode)
        
        self.rate = rospy.Rate(1 / self.dt)
        self.offb_set_mode = SetModeRequest()  # 先设置工作模式为 offboard
        self.arm_cmd = CommandBoolRequest()
    
    def rk44(self, action: list, uav_state: np.ndarray):
        self.phi_d = action[0]
        self.theta_d = action[1]
        self.throttle = action[2]
        
        self.pos[:] = uav_state[0:3]
        self.vel[:] = uav_state[3:6]
        self.att[:] = uav_state[6:9]
        self.pqr[:] = uav_state[9:12]
        
        self.n += 1  # 拍数 +1
        self.time += self.dt
    
    def uav_state_call_back(self):
        return np.concatenate((self.pos, self.vel, self.att, self.pqr))
    
    def uav_pos_vel_call_back(self):
        return np.concatenate((self.pos, self.vel))
    
    def uav_att_pqr_call_back(self):
        return np.concatenate((self.att, self.pqr))
    
    def T_pqr_2_dot_att(self):
        return np.array([[1, np.sin(self.att[0]) * np.tan(self.att[1]), np.cos(self.att[0]) * np.tan(self.att[1])],
                         [0, np.cos(self.att[0]), -np.sin(self.att[0])],
                         [0, np.sin(self.att[0]) / np.cos(self.att[1]), np.cos(self.att[0]) / np.cos(self.att[1])]])
    
    def uav_dot_att(self):
        return np.dot(self.T_pqr_2_dot_att(), self.pqr)
    
    def set_state(self, xx: np.ndarray):
        self.pos[:] = xx[0:3]
        self.vel[:] = xx[3:6]
        self.att[:] = xx[6:9]
        self.pqr[:] = xx[9:12]
    
    def eta(self):
        return self.pos
    
    def dot_eta(self):
        return self.vel
    
    def A(self):
        return self.throttle / self.m * np.array([C(self.att[0]) * C(self.att[2]) * S(self.att[1]) + S(self.att[0]) * S(self.att[2]),
                                                  C(self.att[0]) * S(self.att[2]) * S(self.att[1]) - S(self.att[0]) * C(self.att[2]),
                                                  C(self.att[0]) * C(self.att[1])]) - np.array([0., 0., self.g])
    
    def ctrl_param_cb(self, msg):
        self.ctrl_param = msg
    
    def state_cb(self, msg):
        self.current_state = msg
    
    def uav_odom_cb(self, msg: Odometry):
        self.uav_odom.pose.pose.position.x = msg.pose.pose.position.x + self.offset[0]
        self.uav_odom.pose.pose.position.y = msg.pose.pose.position.y + self.offset[1]
        self.uav_odom.pose.pose.position.z = msg.pose.pose.position.z + self.offset[2]
        self.uav_odom.pose.pose.orientation.x = msg.pose.pose.orientation.x
        self.uav_odom.pose.pose.orientation.y = msg.pose.pose.orientation.y
        self.uav_odom.pose.pose.orientation.z = msg.pose.pose.orientation.z
        self.uav_odom.pose.pose.orientation.w = msg.pose.pose.orientation.w
        
        self.uav_odom.twist.twist.linear.x = msg.twist.twist.linear.x
        self.uav_odom.twist.twist.linear.y = msg.twist.twist.linear.y
        self.uav_odom.twist.twist.linear.z = msg.twist.twist.linear.z
        self.uav_odom.twist.twist.angular.x = msg.twist.twist.angular.x
        self.uav_odom.twist.twist.angular.y = msg.twist.twist.angular.y
        self.uav_odom.twist.twist.angular.z = msg.twist.twist.angular.z
    
    def uav_battery_cb(self, msg: BatteryState):
        self.voltage = msg.voltage
    
    def approaching(self):
        self.pose.pose.position.x = self.pos0[0] - self.offset[0]
        self.pose.pose.position.y = self.pos0[1] - self.offset[1]
        self.pose.pose.position.z = self.pos0[2] - self.offset[2]
        
        cmd_q = tf.transformations.quaternion_from_euler(0., 0., 0)
        self.pose.pose.orientation.x = cmd_q[0]
        self.pose.pose.orientation.y = cmd_q[1]
        self.pose.pose.orientation.z = cmd_q[2]
        self.pose.pose.orientation.w = cmd_q[3]
        
        self.set_state(uav_odom_2_uav_state(self.uav_odom))
        self.local_pos_pub.publish(self.pose)
        if ((np.linalg.norm(self.pos0 - self.pos) < 0.2) and  # 位置误差
                (np.linalg.norm(self.vel) < 0.1) and  # 速度
                (np.linalg.norm(self.att[2]) < deg2rad(5))):  # 偏航角
            self.global_flag = 2
        else:
            self.global_flag = 1
    
    def connect(self):
        while (not rospy.is_shutdown()) and (not self.current_state.connected):
            self.rate.sleep()
        
        self.pose.pose.position.x = self.uav_odom.pose.pose.position.x
        self.pose.pose.position.y = self.uav_odom.pose.pose.position.y
        self.pose.pose.position.z = self.uav_odom.pose.pose.position.z
        
        for i in range(100):
            if rospy.is_shutdown():
                break
            self.local_pos_pub.publish(self.pose)
            self.rate.sleep()
    
    def offboard_arm(self):
        self.offb_set_mode.custom_mode = 'OFFBOARD'
        self.arm_cmd.value = True  # 通过指令将电机解锁
        
        while (self.current_state.mode != "OFFBOARD") and (not rospy.is_shutdown()):  # 等待
            if self.set_mode_client.call(self.offb_set_mode).mode_sent:
                print('Switching to OFFBOARD mode is available...waiting for 1 seconds')
                break
            self.local_pos_pub.publish(self.pose)
            self.rate.sleep()
        
        while (not self.current_state.armed) and (not rospy.is_shutdown()):
            if self.arming_client.call(self.arm_cmd).success:
                print('UAV is armed now.')
                break
            self.local_pos_pub.publish(self.pose)
            self.rate.sleep()
    
    def publish_ctrl_cmd(self, ctrl, psi_d, use_gazebo):
        phi_d, theta_d, uf = uo_2_ref_angle_throttle(ctrl,
                                                     self.att,
                                                     psi_d,
                                                     self.m,
                                                     self.g,
                                                     limit=[np.pi / 4, np.pi / 4],
                                                     att_limitation=True)
        self.ctrl_cmd.header.stamp = rospy.Time.now()
        self.ctrl_cmd.type_mask = AttitudeTarget.IGNORE_ROLL_RATE + AttitudeTarget.IGNORE_PITCH_RATE + AttitudeTarget.IGNORE_YAW_RATE
        cmd_q = tf.transformations.quaternion_from_euler(phi_d, theta_d, psi_d, axes='sxyz')
        # cmd_q = euler_2_quaternion(phi_d, theta_d, psi_d)
        self.ctrl_cmd.orientation.x = cmd_q[0]
        self.ctrl_cmd.orientation.y = cmd_q[1]
        self.ctrl_cmd.orientation.z = cmd_q[2]
        self.ctrl_cmd.orientation.w = cmd_q[3]
        self.ctrl_cmd.thrust = thrust_2_throttle(uf, use_gazebo)
        self.uav_att_throttle_pub.publish(self.ctrl_cmd)
        return phi_d, theta_d, uf
