import os, sys

import numpy as np
import rospy
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState
from tf.transformations import quaternion_matrix
from std_msgs.msg import Float32MultiArray
from uav0.msg import uav_msg

from control.utils import *


class UAV_ROS_Consensus:
    def __init__(self,
                 m: float = 0.722,
                 dt: float = 0.01,
                 pos0: np.ndarray = np.zeros(3),
                 offset: np.ndarray = np.zeros(3),
                 uav_existance: list = None,
                 adj: list = None,
                 d: float = 0.,
                 b: float = 0.,
                 group='',
                 use_ros_param: bool = False,
                 name: str = ''):
        self.g = 9.8
        self.kt = 1e-3
        if uav_existance is None:
            uav_existance = [1, 0, 0, 0]
        self.uav_existance = uav_existance
        if use_ros_param:
            _p = rospy.get_param(name)
            self.adj = _p['adj']
            self.b = _p['b']
            self.d = _p['d']
            self.m = _p['m']
            self.pos0 = np.array(_p['pos0'])
            self.offset = np.array(_p['offset'])
            self.dt = _p['dt']
            self.group = _p['group']
        else:
            if adj is None:
                adj = [0, 0, 0, 0]
            self.adj = adj
            self.d = d
            self.b = b
            self.m = m
            self.pos0 = pos0
            self.offset = offset
            self.group = group
            self.dt = dt
        
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.att = np.zeros(3)
        self.pqr = np.zeros(3)
        
        self.n = 0  # 记录走过的拍数
        self.time = 0.  # 当前时间
        # self.time_max = time_max
        
        '''control'''
        self.throttle = self.m * self.g  # 油门
        self.phi_d = 0.
        self.theta_d = 0.
        self.dot_phi_d = 0.
        self.dot_theta_d = 0.
        self.consensus_e = np.zeros(3)
        self.consensus_de = np.zeros(3)
        self.lambda_eta = np.zeros(3)
        '''control'''
        
        self.current_state = State()  # monitor uav status
        self.ctrl_param = Float32MultiArray(data=[0., 0., 0., 0., 0., 0., 0., 0., 0.])  # 9 维
        self.nn_input = Float32MultiArray(data=[0., 0., 0., 0., 0., 0.])  # 6 维
        self.pose = PoseStamped()
        self.uav_odom = Odometry()
        self.ctrl_cmd = AttitudeTarget()
        self.voltage = 11.4
        self.global_flag = 0
        
        '''OK 标志位'''
        self.uav_msg = [uav_msg() for _ in range(4)]
        if self.uav_existance[1] == 0:  # 如果 1 号无人机本身不存在，就默认它准备好了
            self.uav_msg[1].are_you_ok.data = True
            self.uav_msg[1].finish.data = True
        if self.uav_existance[2] == 0:  # 如果 2 号无人机本身不存在，就默认它准备好了
            self.uav_msg[2].are_you_ok.data = True
            self.uav_msg[2].finish.data = True
        if self.uav_existance[3] == 0:  # 如果 3 号无人机本身不存在，就默认它准备好了
            self.uav_msg[3].are_you_ok.data = True
            self.uav_msg[3].finish.data = True
        
        '''OK 标志位 pub 或者 sub'''
        self.uav_msg_0_pub = rospy.Publisher(self.group + "/uav_msg", uav_msg, queue_size=10)
        self.uav_msg_1_sub = rospy.Subscriber("/uav1/uav_msg", uav_msg, callback=self.uav_msg_1_cb)
        self.uav_msg_2_sub = rospy.Subscriber("/uav2/uav_msg", uav_msg, callback=self.uav_msg_2_cb)
        self.uav_msg_3_sub = rospy.Subscriber("/uav3/uav_msg", uav_msg, callback=self.uav_msg_3_cb)
        '''OK 标志位 pub 或者 sub'''
        
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
    
    def ctrl_param_cb(self, msg: Float32MultiArray):
        self.ctrl_param = msg
    
    def uav_msg_1_cb(self, msg: uav_msg):
        self.uav_msg[1] = msg
    
    def uav_msg_2_cb(self, msg: uav_msg):
        self.uav_msg[2] = msg
    
    def uav_msg_3_cb(self, msg: uav_msg):
        self.uav_msg[3] = msg
    
    def cal_consensus_e(self, nu: np.ndarray, eta_d: np.ndarray):
        # l1 = np.zeros(3)    # 它自己
        # l2 = self.adj[1] * (self.eta() - nu - self.uav_msg[1].eta + self.uav_msg[1].nu) \
        #     if self.uav_existance[1] == 1 else np.zeros(3)
        # l3 = self.adj[2] * (self.eta() - nu - self.uav_msg[2].eta + self.uav_msg[2].nu) \
        #     if self.uav_existance[2] == 1 else np.zeros(3)
        # l4 = self.adj[3] * (self.eta() - nu - self.uav_msg[3].eta + self.uav_msg[3].nu) \
        #     if self.uav_existance[3] == 1 else np.zeros(3)
        # self.consensus_e = l1 + l2 + l3 + l4 + self.b * (self.eta() - eta_d - nu)
        
        e1 = (self.d + self.b) * (self.eta() - nu) - self.b * eta_d
        
        l1 = self.adj[0] * (self.eta() - nu)  # uav0
        l2 = self.adj[1] * (np.array(self.uav_msg[1].eta) - np.array(self.uav_msg[1].nu)) \
            if self.uav_existance[1] == 1 else np.zeros(3)
        l3 = self.adj[2] * (np.array(self.uav_msg[2].eta) - np.array(self.uav_msg[2].nu)) \
            if self.uav_existance[2] == 1 else np.zeros(3)
        l4 = self.adj[3] * (np.array(self.uav_msg[3].eta) - np.array(self.uav_msg[3].nu)) \
            if self.uav_existance[3] == 1 else np.zeros(3)
        Lambda = l1 + l2 + l3 + l4
        self.consensus_e = e1 - Lambda
    
    def cal_consensus_de(self, dot_nu: np.ndarray, dot_eta_d: np.ndarray):
        # dl1 = np.zeros(3)
        # dl2 = self.adj[1] * (np.array(self.uav_msg[1].dot_eta) - np.array(self.uav_msg[1].dot_nu)) \
        #     if self.uav_existance[1] == 1 else np.zeros(3)
        # dl3 = self.adj[2] * (np.array(self.uav_msg[2].dot_eta) - np.array(self.uav_msg[2].dot_nu)) \
        #     if self.uav_existance[2] == 1 else np.zeros(3)
        # dl4 = self.adj[3] * (np.array(self.uav_msg[3].dot_eta) - np.array(self.uav_msg[3].dot_nu)) \
        #     if self.uav_existance[3] == 1 else np.zeros(3)
        # self.consensus_de = dl1 + dl2 + dl3 + dl4 + self.b * (self.dot_eta() - dot_eta_d - dot_nu)
        
        dot_e1 = (self.d + self.b) * (self.dot_eta() - dot_nu) - self.b * dot_eta_d
        dl1 = self.adj[0] * (self.dot_eta() - dot_nu)
        dl2 = self.adj[1] * (np.array(self.uav_msg[1].dot_eta) - np.array(self.uav_msg[1].dot_nu)) \
            if self.uav_existance[1] == 1 else np.zeros(3)
        dl3 = self.adj[2] * (np.array(self.uav_msg[2].dot_eta) - np.array(self.uav_msg[2].dot_nu)) \
            if self.uav_existance[2] == 1 else np.zeros(3)
        dl4 = self.adj[3] * (np.array(self.uav_msg[3].dot_eta) - np.array(self.uav_msg[3].dot_nu)) \
            if self.uav_existance[3] == 1 else np.zeros(3)
        
        dot_Lambda = dl1 + dl2 + dl3 + dl4
        self.consensus_de = dot_e1 - dot_Lambda
    
    def cal_Lambda_eta(self, dot2_eat_d: np.ndarray, dot2_nu: np.ndarray, obs: np.ndarray):
        lambda_eta = -self.b * dot2_eat_d + (self.d + self.b) * (-self.kt / self.m * self.dot_eta() + obs - dot2_nu)
        le1 = self.adj[0] * (np.array(self.uav_msg[0].second_order_dynamic) - np.array(self.uav_msg[0].dot2_nu))
        le2 = self.adj[1] * (np.array(self.uav_msg[1].second_order_dynamic) - np.array(self.uav_msg[1].dot2_nu)) \
            if self.uav_existance[1] == 1 else np.zeros(3)
        le3 = self.adj[2] * (np.array(self.uav_msg[2].second_order_dynamic) - np.array(self.uav_msg[2].dot2_nu)) \
            if self.uav_existance[2] == 1 else np.zeros(3)
        le4 = self.adj[3] * (np.array(self.uav_msg[3].second_order_dynamic) - np.array(self.uav_msg[3].dot2_nu)) \
            if self.uav_existance[3] == 1 else np.zeros(3)
        # lambda_eta -= (self.adj[0] * (np.array(self.uav_msg_0.second_order_dynamic) - np.array(self.uav_msg_0.dot2_nu)) +
        #                self.adj[1] * (np.array(self.uav_msg_1.second_order_dynamic) - np.array(self.uav_msg_1.dot2_nu)) +
        #                self.adj[2] * (np.array(self.uav_msg_2.second_order_dynamic) - np.array(self.uav_msg_2.dot2_nu)) +
        #                self.adj[3] * (np.array(self.uav_msg_3.second_order_dynamic) - np.array(self.uav_msg_3.dot2_nu)))
        lambda_eta -= le1 + le2 + le3 + le4
        self.lambda_eta = lambda_eta.copy()
    
    def check_other_uav_ok(self):
        return (self.uav_msg[1].are_you_ok.data and
                self.uav_msg[2].are_you_ok.data and
                self.uav_msg[3].are_you_ok.data)
    
    def uav_msg_publish(self,
                        ref: np.ndarray,
                        dot_ref: np.ndarray,
                        nu: np.ndarray,
                        dot_nu: np.ndarray,
                        dot2_nu: np.ndarray,
                        ctrl: np.ndarray,
                        obs: np.ndarray):
        # self.uav_msg[0].are_you_ok.data = True if (self.global_flag == 2 or self.global_flag == 3) else False
        self.uav_msg[0].finish.data = True if self.global_flag == 3 else False
        self.uav_msg[0].eta = self.eta().tolist()
        self.uav_msg[0].dot_eta = self.dot_eta().tolist()
        self.uav_msg[0].ref = ref.tolist()
        self.uav_msg[0].dot_ref = dot_ref.tolist()
        self.uav_msg[0].nu = nu.tolist()
        self.uav_msg[0].dot_nu = dot_nu.tolist()
        self.uav_msg[0].dot2_nu = dot2_nu.tolist()
        self.uav_msg[0].second_order_dynamic = (-self.kt / self.m * self.dot_eta() + ctrl + obs).tolist()
        self.uav_msg[0].name = 'uav0'
        self.uav_msg_0_pub.publish(self.uav_msg[0])
    
    def nn_input_publish(self):
        self.nn_input.data = np.concatenate((self.consensus_e, self.consensus_de)).tolist()
        self.nn_input_state_pub.publish(self.nn_input)
    
    def state_cb(self, msg: State):
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
        if ((np.linalg.norm(self.pos0 - self.pos) < 0.3) and  # 位置误差
                (np.linalg.norm(self.vel) < 0.2) and  # 速度
                (np.linalg.norm(self.att[2]) < deg2rad(5))):  # 偏航角
            self.uav_msg[0].are_you_ok.data = False
            return True
        else:
            self.uav_msg[0].are_you_ok.data = True
            return False
    
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
    
    def publish_ctrl_cmd(self, ctrl, psi_d, phi_d_old, theta_d_old, dt, att_limit, dot_att_limit, use_gazebo):
        # phi_d, theta_d, uf = uo_2_ref_angle_throttle(ctrl,
        #                                              self.att,
        #                                              psi_d,
        #                                              self.m,
        #                                              self.g,
        #                                              limit=[np.pi / 4, np.pi / 4],
        #                                              att_limitation=True)
        phi_d, theta_d, dot_phi_d, dot_theta_d, uf = uo_2_ref_angle_throttle2(control=ctrl,
                                                                              attitude=self.att,
                                                                              psi_d=psi_d,
                                                                              m=self.m,
                                                                              g=self.g,
                                                                              phi_d_old=phi_d_old,
                                                                              theta_d_old=theta_d_old,
                                                                              dt=dt,
                                                                              att_limit=att_limit,
                                                                              dot_att_limit=dot_att_limit)
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
        self.phi_d = phi_d
        self.theta_d = theta_d
        self.dot_phi_d = dot_phi_d
        self.dot_theta_d = dot_theta_d
        return phi_d, theta_d, dot_phi_d, dot_theta_d, uf
