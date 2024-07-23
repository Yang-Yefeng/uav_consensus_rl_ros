import numpy as np
from typing import Union
import pandas as pd

import torch
import torch.nn as nn
from torch.distributions import Normal

import tf
from tf.transformations import quaternion_matrix
from nav_msgs.msg import Odometry


def deg2rad(deg: Union[np.ndarray, float]):
    """
    :brief:         omit
    :param deg:     degree
    :return:        radian
    """
    return deg * np.pi / 180.0


def rad2deg(rad: Union[np.ndarray, float]):
    """
    :brief:         omit
    :param rad:     radian
    :return:        degree
    """
    return rad * 180.0 / np.pi


def C(x: Union[np.ndarray, float, list]):
    return np.cos(x)


def S(x: Union[np.ndarray, float, list]):
    return np.sin(x)


def uo_2_ref_angle_throttle(control: np.ndarray,
                            attitude: np.ndarray,
                            psi_d: float,
                            m: float,
                            g: float,
                            limit=None,
                            att_limitation: bool = False):
    # [phi, theta, psi] = attitude
    ux = control[0]
    uy = control[1]
    uz = control[2]
    
    uf = m * np.sqrt(ux ** 2 + uy ** 2 + (uz + g) ** 2)
    # phi_d = np.arcsin(m * (ux * np.sin(psi_d) - uy * np.cos(psi_d)) / uf)
    # theta_d = np.arctan((ux * np.cos(psi_d) + uy * np.sin(psi_d)) / (uz + g))
    phi_d = np.arcsin(m * (ux * np.sin(attitude[2]) - uy * np.cos(attitude[2])) / uf)
    theta_d = np.arctan((ux * np.cos(attitude[2]) + uy * np.sin(attitude[2])) / (uz + g))
    
    if att_limitation and (limit is not None):
        phi_d = max(min(phi_d, limit[0]), -limit[0])
        theta_d = max(min(theta_d, limit[1]), -limit[1])
    return phi_d, theta_d, uf


def uo_2_ref_angle_throttle2(control: np.ndarray,
                             attitude: np.ndarray,
                             psi_d: float,
                             m: float,
                             g: float,
                             phi_d_old: float,
                             theta_d_old: float,
                             dt: float,
                             att_limit: list,
                             dot_att_limit: list):
    # [phi, theta, psi] = attitude
    ux = control[0]
    uy = control[1]
    uz = control[2]
    
    # uf = m * np.sqrt(ux ** 2 + uy ** 2 + (uz + g) ** 2)
    uf = (uz + g) * m / (np.cos(attitude[0]) * np.cos(attitude[1]))
    
    asin_phi_d = np.clip((ux * np.sin(attitude[2]) - uy * np.cos(attitude[2])) * m / uf, -1, 1)
    phi_d = np.arcsin(asin_phi_d)
    if att_limit is not None:
        phi_d = np.clip(phi_d, -att_limit[0], att_limit[0])
    
    dot_phi_d = (phi_d - phi_d_old) / dt
    if dot_att_limit is not None:
        dot_phi_d = np.clip(dot_phi_d, -dot_att_limit[0], dot_att_limit[0])
    
    # theta_d = np.arctan((ux * np.cos(attitude[2]) + uy * np.sin(attitude[2])) / (uz + g))
    
    asin_theta_d = np.clip((ux * np.cos(attitude[2]) + uy * np.sin(attitude[2])) * m / (uf * np.cos(phi_d)), -1, 1)
    theta_d = np.arcsin(asin_theta_d)
    if att_limit is not None:
        theta_d = np.clip(theta_d, -att_limit[1], att_limit[1])
    
    dot_theta_d = (theta_d - theta_d_old) / dt
    if dot_att_limit is not None:
        dot_theta_d = np.clip(dot_theta_d, -dot_att_limit[1], dot_att_limit[1])
    
    return phi_d, theta_d, dot_phi_d, dot_theta_d, uf


def ref_uav(time: float, amplitude: np.ndarray, period: np.ndarray, bias_a: np.ndarray, bias_phase: np.ndarray):
    w = 2 * np.pi / period
    _r = amplitude * np.sin(w * time + bias_phase) + bias_a
    _dr = amplitude * w * np.cos(w * time + bias_phase)
    _ddr = -amplitude * w ** 2 * np.sin(w * time + bias_phase)
    return _r, _dr, _ddr


def ref_uav_sequence(dt: float,
                     tm: float,
                     amplitude: np.ndarray,
                     period: np.ndarray,
                     bias_a: np.ndarray,
                     bias_phase: np.ndarray):
    w = 2 * np.pi / period
    N = int(np.round(tm / dt))
    _r = np.zeros((N, 4))
    _dr = np.zeros((N, 4))
    _ddr = np.zeros((N, 4))
    for i in range(N):
        _r[i, :] = amplitude * np.sin(w * i * dt + bias_phase) + bias_a
        _dr[i, :] = amplitude * w * np.cos(w * i * dt + bias_phase)
        _ddr[i, :] = -amplitude * w ** 2 * np.sin(w * i * dt + bias_phase)
    return _r, _dr, _ddr


def ref_uav_sequence_with_dead(dt: float,
                               tm: float,
                               t_miemie: float,
                               amplitude: np.ndarray,
                               period: np.ndarray,
                               bias_a: np.ndarray,
                               bias_phase: np.ndarray):
    w = 2 * np.pi / period
    N = int(np.round(tm / dt))
    _r = np.zeros((N, 4))
    _dr = np.zeros((N, 4))
    _ddr = np.zeros((N, 4))
    _r0 = amplitude * np.sin(bias_phase) + bias_a
    for i in range(N):
        _r[i, :] = amplitude * np.sin(w * i * dt + bias_phase) + bias_a
        _dr[i, :] = amplitude * w * np.cos(w * i * dt + bias_phase)
        _ddr[i, :] = -amplitude * w ** 2 * np.sin(w * i * dt + bias_phase)
    
    N1 = int(np.round(t_miemie / dt))
    _r_miemie = np.tile(_r0, (N1, 1))
    _dr_miemie = np.zeros((N1, 4))
    _ddr_miemie = np.zeros((N1, 4))
    
    return np.concatenate((_r_miemie, _r)), np.concatenate((_dr_miemie, _dr)), np.concatenate((_ddr_miemie, _ddr))


def ref_uav_sequence_Bernoulli_with_dead(dt: float,
                                         tm: float,
                                         t_miemie: float,
                                         amplitude: np.ndarray,
                                         period: np.ndarray,
                                         bias_a: np.ndarray,
                                         bias_phase: np.ndarray):
    w = 2 * np.pi / period
    N = int(np.round(tm / dt))
    _r = np.zeros((N, 4))
    _dr = np.zeros((N, 4))
    _ddr = np.zeros((N, 4))
    _r0 = amplitude * np.sin(bias_phase) + bias_a
    for i in range(N):
        _r[i, 0] = amplitude[0] * np.cos(w[0] * i * dt + bias_phase[0]) + bias_a[0]
        _r[i, 1] = amplitude[1] * np.sin(2 * w[1] * i * dt + bias_phase[1]) / 2 + bias_a[1]
        _r[i, 2: 4] = amplitude[2: 4] * np.sin(w[2: 4] * i * dt + bias_phase[2: 4]) + bias_a[2: 4]
        
        _dr[i, 0] = -amplitude[0] * w[0] * np.sin(w[0] * i * dt + bias_phase[0])
        _dr[i, 1] = amplitude[1] * w[1] * np.cos(2 * w[1] * i * dt + bias_phase[1])
        _dr[i, 2: 4] = amplitude[2: 4] * w[2: 4] * np.cos(w[2: 4] * i * dt + bias_phase[2: 4])
        
        _ddr[i, 0] = -amplitude[0] * w[0] ** 2 * np.cos(w[0] * i * dt + bias_phase[0])
        _ddr[i, 1] = - 2 * amplitude[1] * w[1] ** 2 * np.sin(2 * w[1] * i * dt + bias_phase[1])
        _ddr[i, 2: 4] = -amplitude[2: 4] * w[2: 4] ** 2 * np.sin(w[2: 4] * i * dt + bias_phase[2: 4])
    
    N1 = int(np.round(t_miemie / dt))
    _r_miemie = np.tile(_r0, (N1, 1))
    _dr_miemie = np.zeros((N1, 4))
    _ddr_miemie = np.zeros((N1, 4))
    
    return np.concatenate((_r_miemie, _r)), np.concatenate((_dr_miemie, _dr)), np.concatenate((_ddr_miemie, _ddr))


def offset_uav_sequence(dt: float, tm: float, A: np.ndarray, T: np.ndarray, ba: np.ndarray, bp: np.ndarray):
    N = int(np.round(tm / dt))
    _off = np.zeros((N, 3))
    _doff = np.zeros((N, 3))
    _ddoff = np.zeros((N, 3))
    w = 2 * np.pi / T
    for i in range(N):
        _off[i, :] = A * np.sin(w * i * dt + bp) + ba
        _doff[i, :] = A * w * np.cos(w * i * dt + bp)
        _ddoff[i, :] = -A * w ** 2 * np.sin(w * i * dt + bp)
    return _off, _doff, _ddoff


def offset_uav_sequence_with_dead(dt: float, tm: float, t_miemie: float, A: np.ndarray, T: np.ndarray, ba: np.ndarray, bp: np.ndarray):
    N = int(np.round(tm / dt))
    _off = np.zeros((N, 3))
    _doff = np.zeros((N, 3))
    _ddoff = np.zeros((N, 3))
    w = 2 * np.pi / T
    _off0 = A * np.sin(bp) + ba
    for i in range(N):
        _off[i, :] = A * np.sin(w * i * dt + bp) + ba
        _doff[i, :] = A * w * np.cos(w * i * dt + bp)
        _ddoff[i, :] = -A * w ** 2 * np.sin(w * i * dt + bp)
    
    N1 = int(np.round(t_miemie / dt))
    _off_miemie = np.tile(_off0, (N1, 1))
    _doff_miemie = np.zeros((N1, 3))
    _ddoff_miemie = np.zeros((N1, 3))
    
    return np.concatenate((_off_miemie, _off)), np.concatenate((_doff_miemie, _doff)), np.concatenate((_ddoff_miemie, _ddoff))


def ref_uav_set_point_sequence_with_dead(dt:float, time_max:float, t_miemie:float, center:np.ndarray):
    _p = center.shape[0]
    step = int(time_max / _p / dt)
    NN = int((time_max + t_miemie) / dt)
    N = int(t_miemie / dt)
    ref = None
    for i in range(_p):
        if ref is None:
            ref = np.tile(center[i], (step, 1))
        else:
            ref = np.vstack((ref, np.tile(center[i], (step, 1))))
    ref = np.vstack((np.tile(center[0], (N, 1)), ref))
    return ref, np.zeros((NN, 4)), np.zeros((NN, 4))


def offset_set_point_sequence_with_dead(dt: float, time_max: float, t_miemie: float, offset: np.ndarray):
    _p = offset.shape[0]
    step = int(time_max / _p / dt)
    NN = int((time_max + t_miemie) / dt)
    N = int(t_miemie / dt)
    ref = None
    for i in range(_p):
        if ref is None:
            ref = np.tile(offset[i], (step, 1))
        else:
            ref = np.vstack((ref, np.tile(offset[i], (step, 1))))
    ref = np.vstack((np.tile(offset[0], (N, 1)), ref))
    return ref, np.zeros((NN, 3)), np.zeros((NN, 3))


def euler_2_quaternion(phi, theta, psi):
    w = C(phi / 2) * C(theta / 2) * C(psi / 2) + S(phi / 2) * S(theta / 2) * S(psi / 2)
    x = S(phi / 2) * C(theta / 2) * C(psi / 2) - C(phi / 2) * S(theta / 2) * S(psi / 2)
    y = C(phi / 2) * S(theta / 2) * C(psi / 2) + S(phi / 2) * C(theta / 2) * S(psi / 2)
    z = C(phi / 2) * C(theta / 2) * S(psi / 2) - S(phi / 2) * S(theta / 2) * C(psi / 2)
    return [x, y, z, w]


def uav_odom_2_uav_state(odom: Odometry) -> np.ndarray:
    _orientation = odom.pose.pose.orientation
    _w = _orientation.w
    _x = _orientation.x
    _y = _orientation.y
    _z = _orientation.z
    rpy = tf.transformations.euler_from_quaternion([_x, _y, _z, _w])
    _uav_state = np.array([
        odom.pose.pose.position.x,  # x
        odom.pose.pose.position.y,  # y
        odom.pose.pose.position.z,  # z
        odom.twist.twist.linear.x,  # vx
        odom.twist.twist.linear.y,  # vy
        odom.twist.twist.linear.z,  # vz
        rpy[0],  # phi
        rpy[1],  # theta
        rpy[2],  # psi
        odom.twist.twist.angular.x,  # p
        odom.twist.twist.angular.y,  # q
        odom.twist.twist.angular.z  # r
    ])
    return _uav_state


def thrust_2_throttle(thrust: float, use_gazebo: bool):
    """线性模型"""
    if use_gazebo:
        k = 0.56 / 1.5 / 9.8
    else:
        k = 0.31 / 0.715 / 9.8
    _throttle = max(min(k * thrust, 0.9), 0.10)
    '''线性模型'''
    return _throttle


class RunningMeanStd:
    # Dynamically calculate mean and std
    def __init__(self, shape):  # shape:the dimension of input data
        self.n = 0
        self.mean = np.zeros(shape)
        self.S = np.zeros(shape)
        self.std = np.sqrt(self.S)
    
    def update(self, x):
        x = np.array(x)
        self.n += 1
        if self.n == 1:
            self.mean = x
            self.std = x
        else:
            old_mean = self.mean.copy()
            self.mean = old_mean + (x - old_mean) / self.n
            self.S = self.S + (x - old_mean) * (x - self.mean)
            self.std = np.sqrt(self.S / self.n)


class Normalization:
    def __init__(self, shape):
        self.running_ms = RunningMeanStd(shape=shape)
    
    def __call__(self, x, update=True):
        # Whether to update the mean and std,during the evaluating, update=False
        if update:
            self.running_ms.update(x)
        x = (x - self.running_ms.mean) / (self.running_ms.std + 1e-8)
        return x


def orthogonal_init(layer, gain=1.0):
    nn.init.orthogonal_(layer.weight, gain=gain)
    nn.init.constant_(layer.bias, 0)


class PPOActor_Gaussian(nn.Module):
    def __init__(self,
                 state_dim: int = 3,
                 action_dim: int = 3,
                 a_min: np.ndarray = np.zeros(3),
                 a_max: np.ndarray = np.ones(3),
                 init_std: float = 0.5,
                 use_orthogonal_init: bool = True):
        super(PPOActor_Gaussian, self).__init__()
        self.fc1 = nn.Linear(state_dim, 64)
        self.fc2 = nn.Linear(64, 64)
        self.fc3 = nn.Linear(64, 32)
        self.mean_layer = nn.Linear(32, action_dim)
        # self.log_std = nn.Parameter(torch.zeros(1, action_dim))  # We use 'nn.Parameter' to train log_std automatically
        # self.log_std = nn.Parameter(np.log(init_std) * torch.ones(action_dim))  # We use 'nn.Parameter' to train log_std automatically
        self.activate_func = nn.Tanh()
        self.a_min = torch.tensor(a_min, dtype=torch.float)
        self.a_max = torch.tensor(a_max, dtype=torch.float)
        self.off = (self.a_min + self.a_max) / 2.0
        self.gain = self.a_max - self.off
        self.action_dim = action_dim
        self.std = torch.tensor(init_std, dtype=torch.float)
        
        if use_orthogonal_init:
            # print("------use_orthogonal_init------")
            self.orthogonal_init_all()
    
    def orthogonal_init_all(self):
        orthogonal_init(self.fc1)
        orthogonal_init(self.fc2)
        orthogonal_init(self.fc3)
        orthogonal_init(self.mean_layer, gain=0.01)
    
    def forward(self, s):
        s = self.activate_func(self.fc1(s))
        s = self.activate_func(self.fc2(s))
        s = self.activate_func(self.fc3(s))
        # mean = torch.tanh(self.mean_layer(s)) * self.gain + self.off
        mean = torch.relu(self.mean_layer(s))
        return mean
    
    def get_dist(self, s):
        mean = self.forward(s)
        # mean = torch.tensor(mean, dtype=torch.float)
        # log_std = self.log_std.expand_as(mean)
        # std = torch.exp(log_std)
        std = self.std.expand_as(mean)
        dist = Normal(mean, std)  # Get the Gaussian distribution
        # std = self.std.expand_as(mean)
        # dist = Normal(mean, std)
        return dist
    
    def evaluate(self, state):
        with torch.no_grad():
            t_state = torch.unsqueeze(torch.tensor(state, dtype=torch.float), 0)
            action_mean = self.forward(t_state)
        return action_mean.detach().cpu().numpy().flatten()


def get_normalizer_from_file(dim, path, file):
    norm = Normalization(dim)
    data = pd.read_csv(path + file, header=0).to_numpy()
    norm.running_ms.n = data[0, 0]
    norm.running_ms.mean = data[:, 1]
    norm.running_ms.std = data[:, 2]
    norm.running_ms.S = data[:, 3]
    norm.running_ms.n = data[0, 4]
    norm.running_ms.mean = data[:, 5]
    norm.running_ms.std = data[:, 6]
    norm.running_ms.S = data[:, 7]
    return norm
