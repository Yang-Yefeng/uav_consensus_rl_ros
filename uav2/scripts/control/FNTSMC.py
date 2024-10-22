import numpy as np
import rospy

class fntsmc_param:
    def __init__(self,
                 k1: np.ndarray = np.zeros(3),
                 k2: np.ndarray = np.zeros(3),
                 k3: np.ndarray = np.zeros(3),
                 k4: np.ndarray = np.zeros(3),
                 alpha1: np.ndarray = 1.01 * np.ones(3),
                 alpha2: np.ndarray = 1.01 * np.ones(3),
                 k_com_pos: np.ndarray = np.zeros(3),
                 k_com_vel: np.ndarray = np.zeros(3),
                 k_com_acc: np.ndarray = np.zeros(3),
                 k_yyf_p: np.ndarray = np.zeros(3),
                 k_yyf_i: np.ndarray = np.zeros(3),
                 k_yyf_d: np.ndarray = np.zeros(3),
                 dim: int = 3,
                 dt: float = 0.01
                 ):
        self.k1 = k1
        self.k2 = k2
        self.k3 = k3
        self.k4 = k4
        self.alpha1 = alpha1
        self.alpha2 = alpha2
        
        self.k_com_pos = k_com_pos
        self.k_com_vel = k_com_vel
        self.k_com_acc = k_com_acc
        self.k_yyf_p = k_yyf_p
        self.k_yyf_i = k_yyf_i
        self.k_yyf_d = k_yyf_d
        
        self.dim = dim
        self.dt = dt
    
    def load_param_from_yaml(self, name: str):
        _p = rospy.get_param(name)
        self.k1 = np.array(_p['k1']).astype(float)
        self.k2 = np.array(_p['k2']).astype(float)
        self.k3 = np.array(_p['k3']).astype(float)
        self.k4 = np.array(_p['k4']).astype(float)
        self.alpha1 = np.array(_p['alpha1']).astype(float)
        self.alpha2 = np.array(_p['alpha2']).astype(float)
        self.k_com_pos = np.array(_p['k_com_pos']).astype(float)
        self.k_com_vel = np.array(_p['k_com_vel']).astype(float)
        self.k_com_acc = np.array(_p['k_com_acc']).astype(float)
        self.k_yyf_p = np.array(_p['k_yyf_p']).astype(float)
        self.k_yyf_i = np.array(_p['k_yyf_i']).astype(float)
        self.k_yyf_d = np.array(_p['k_yyf_d']).astype(float)
        self.dim = np.array(_p['dim']).astype(int)
        self.dt = np.array(_p['dt']).astype(float)


class fntsmc:
    def __init__(self,
                 param: fntsmc_param = None,
                 k1: np.ndarray = np.array([0.3, 0.3, 1.]),
                 k2: np.ndarray = np.array([0.5, 0.5, 1.]),
                 k3: np.ndarray = np.array([0.05, 0.05, 0.05]),
                 k4: np.ndarray = np.array([6, 6, 6]),
                 alpha1: np.ndarray = np.array([1.01, 1.01, 1.01]),
                 alpha2: np.ndarray = np.array([1.01, 1.01, 1.01]),
                 k_com_pos: np.ndarray = np.zeros(3),
                 k_com_vel: np.ndarray = np.zeros(3),
                 k_com_acc: np.ndarray = np.zeros(3),
                 yyf_p: np.ndarray = np.zeros(3),
                 yyf_i: np.ndarray = np.zeros(3),
                 yyf_d: np.ndarray = np.zeros(3),
                 dim: int = 3,
                 dt: float = 0.01):
        self.k1 = k1.copy() if param is None else param.k1.copy()
        self.k2 = k2.copy() if param is None else param.k2.copy()
        self.k3 = k3.copy() if param is None else param.k3.copy()
        self.k4 = k4.copy() if param is None else param.k4.copy()
        self.alpha1 = alpha1.copy() if param is None else param.alpha1.copy()
        self.alpha2 = alpha2.copy() if param is None else param.alpha2.copy()
        
        self.k_com_pos = k_com_pos if param is None else param.k_com_pos.copy()
        self.k_com_vel = k_com_vel if param is None else param.k_com_vel.copy()
        self.k_com_acc = k_com_acc if param is None else param.k_com_acc.copy()
        self.k_yyf_p = yyf_p if param is None else param.k_yyf_p.copy()
        self.k_yyf_i = yyf_i if param is None else param.k_yyf_i.copy()
        self.k_yyf_d = yyf_d if param is None else param.k_yyf_d.copy()
        
        self.dt = dt if param is None else param.dt
        self.dim = dim if param is None else param.dim
        self.s = np.zeros(self.dim)
        self.control_in = np.zeros(self.dim)
        self.control_out = np.zeros(self.dim)
        
        self.yyf_i = np.zeros(3)
        self.yyf_d = np.zeros(3)
        self.yyf_p = np.zeros(3)
    
    def print_param(self):
        print('========================')
        print('k1: ', self.k1)
        print('k2: ', self.k2)
        print('k3: ', self.k3)
        print('k4: ', self.k4)
        print('alpha1: ', self.alpha1)
        print('alpha2: ', self.alpha2)
        print('========================\n')
    
    @staticmethod
    def sig(x, a, kt=5):
        return np.fabs(x) ** a * np.tanh(kt * x)
    
    def control_update_outer(self,
                             e: np.ndarray,
                             de: np.ndarray,
                             dot_eta: np.ndarray,
                             kt: float,
                             m: float,
                             ref: np.ndarray,
                             d_ref: np.ndarray,
                             dd_ref: np.ndarray,
                             obs: np.ndarray,
                             e_max: float = 0.2,
                             dot_e_max: float = 0.5):
        
        if e_max is not None:  # 增加对位置误差的输入饱和
            e = np.clip(e, -e_max, e_max)
        if dot_e_max is not None:  # 增加对速度误差的输入饱和
            de = np.clip(de, -dot_e_max, dot_e_max)
        
        self.s = (de + self.k_com_vel * d_ref) + self.k1 * (e + self.k_com_pos * ref) + self.k2 * self.sig(e + self.k_com_pos * ref, self.alpha1, kt=5)
        u1 = -kt / m * dot_eta - dd_ref
        u2 = (self.k1 + self.k2 * self.alpha1 * self.sig(e + self.k_com_pos * ref, self.alpha1 - 1)) * (de + self.k_com_vel * d_ref)
        u3 = obs + self.k3 * np.tanh(5 * self.s) + self.k4 * self.sig(self.s, self.alpha2)
        
        self.yyf_i += self.k_yyf_i * e
        self.yyf_p = self.k_yyf_p * e
        self.yyf_d = self.k_yyf_d * de
        
        self.control_out = -(u1 + u2 + u3 + self.yyf_i + self.yyf_p + self.yyf_d + self.k_com_acc*dd_ref)
    
    def get_param_from_actor(self, action_from_actor: np.ndarray, update_z: bool = False):
        if np.min(action_from_actor) < 0:
            print('ERROR!!!!')
        if update_z:
            for i in range(3):  # 分别对应 k1: 0 1 2, k2: 3 4 5, k4: 6 7 8
                if action_from_actor[i] > 0:
                    self.k1[i] = action_from_actor[i]
                if action_from_actor[i + 3] > 0:
                    self.k2[i] = action_from_actor[i + 3]
                if action_from_actor[i + 6] > 0:
                    self.k4[i] = action_from_actor[i + 6]
        else:
            for i in range(2):  # 分别对应 k1: 0 1 2, k2: 3 4 5, k4: 6 7 8
                if action_from_actor[i] > 0:
                    self.k1[i] = action_from_actor[i]
                if action_from_actor[i + 3] > 0:
                    self.k2[i] = action_from_actor[i + 3]
                if action_from_actor[i + 6] > 0:
                    self.k4[i] = action_from_actor[i + 6]
    
    def fntsmc_reset_with_new_param(self, param: fntsmc_param):
        self.k1 = param.k1
        self.k2 = param.k2
        self.k3 = param.k3
        self.k4 = param.k4
        self.alpha1 = param.alpha1
        self.alpha2 = param.alpha2
        self.dim = param.dim
        self.dt = param.dt
        
        self.s = np.zeros(self.dim)
        self.control_in = np.zeros(self.dim)
        self.control_out = np.zeros(self.dim)
    
    def reset(self):
        self.s = np.zeros(self.dim)
        self.control_in = np.zeros(self.dim)
        self.control_out = np.zeros(self.dim)


class fntsmc_consensus(fntsmc):
    def __init__(self,
                 param: fntsmc_param = None,  # 参数
                 k1: np.ndarray = np.zeros(3),  # 状态误差收敛参数，k2 为 e 的增益
                 k2: np.ndarray = np.zeros(3),  # 状态误差收敛参数，k3 为 sig(e)^\alpha 的增益
                 k3: np.ndarray = np.zeros(3),  # 滑模误差收敛参数，k4 为补偿观测器的增益，理论上是充分小就行
                 k4: np.ndarray = np.zeros(3),  # 滑模误差收敛参数，k5 控制滑模收敛速度
                 alpha1: np.ndarray = 1.01 * np.ones(3),  # 状态误差收敛指数
                 alpha2: np.ndarray = 1.01 * np.ones(3),  # 滑模误差收敛指数
                 k_com_pos: np.ndarray = np.zeros(3),
                 k_com_vel: np.ndarray = np.zeros(3),
                 k_com_acc: np.ndarray = np.zeros(3),
                 yyf_p: np.ndarray = np.zeros(3),
                 yyf_i: np.ndarray = np.zeros(3),
                 yyf_d: np.ndarray = np.zeros(3),
                 dim: int = 3,
                 dt: float = 0.01):
        super(fntsmc_consensus, self).__init__(param, k1, k2, k3, k4, alpha1, alpha2,
                                               k_com_pos, k_com_vel, k_com_acc,
                                               yyf_p, yyf_i, yyf_d, dim, dt)
        self.control_out_consensus = np.zeros(self.dim)
    
    def control_update_outer_consensus(self,
                                       b: float,
                                       d: float,
                                       consensus_e: np.ndarray,
                                       consensus_de: np.ndarray,
                                       Lambda_eta: np.ndarray,
                                       ref: np.ndarray,
                                       d_ref: np.ndarray,
                                       e_max: float = 0.2,
                                       dot_e_max: float = 1.5):
        if e_max is not None:  # 增加对位置误差的输入饱和
            consensus_e = np.clip(consensus_e, -e_max, e_max)
        if dot_e_max is not None:  # 增加对速度误差的输入饱和
            consensus_de = np.clip(consensus_de, -dot_e_max, dot_e_max)
        
        s = (consensus_de + self.k_com_vel * d_ref) + self.k1 * (consensus_e + self.k_com_pos * ref) + self.k2 * self.sig(consensus_e + self.k_com_pos * ref, self.alpha1)
        sigma = (self.k1 + self.k2 * self.alpha1 * self.sig(consensus_e + self.k_com_pos * ref, self.alpha1 - 1)) * (consensus_de + self.k_com_vel * d_ref)
        u1 = Lambda_eta + sigma
        u2 = self.k3 * np.tanh(5 * s) + self.k4 * self.sig(s, self.alpha2)
        
        self.yyf_i += self.k_yyf_i * consensus_e
        self.yyf_p = self.k_yyf_p * consensus_e
        self.yyf_d = self.k_yyf_d * consensus_de
        
        self.control_out_consensus = -(u1 + u2 + self.yyf_i + self.yyf_p + self.yyf_d) / (d + b)
