import numpy as np


class fntsmc_param:
    def __init__(self,
                 k1: np.ndarray = np.zeros(3),
                 k2: np.ndarray = np.zeros(3),
                 k3: np.ndarray = np.zeros(3),
                 k4: np.ndarray = np.zeros(3),
                 alpha1: np.ndarray = 1.01 * np.ones(3),
                 alpha2: np.ndarray = 1.01 * np.ones(3),
                 dim: int = 3,
                 dt: float = 0.01
                 ):
        self.k1 = k1
        self.k2 = k2
        self.k3 = k3
        self.k4 = k4
        self.alpha1 = alpha1
        self.alpha2 = alpha2
        self.dim = dim
        self.dt = dt


class fntsmc:
    def __init__(self,
                 param: fntsmc_param = None,
                 k1: np.ndarray = np.array([0.3, 0.3, 1.]),
                 k2: np.ndarray = np.array([0.5, 0.5, 1.]),
                 k3: np.ndarray = np.array([0.05, 0.05, 0.05]),
                 k4: np.ndarray = np.array([6, 6, 6]),
                 alpha1: np.ndarray = np.array([1.01, 1.01, 1.01]),
                 alpha2: np.ndarray = np.array([1.01, 1.01, 1.01]),
                 dim: int = 3,
                 dt: float = 0.01):
        self.k1 = k1 if param is None else param.k1
        self.k2 = k2 if param is None else param.k2
        self.k3 = k3 if param is None else param.k3
        self.k4 = k4 if param is None else param.k4
        self.alpha1 = alpha1 if param is None else param.alpha1
        self.alpha2 = alpha2 if param is None else param.alpha2
        self.dt = dt if param is None else param.dt
        self.dim = dim if param is None else param.dim
        self.s = np.zeros(self.dim)
        self.control_in = np.zeros(self.dim)
        self.control_out = np.zeros(self.dim)
    
    @staticmethod
    def sig(x, a, kt=5):
        return np.fabs(x) ** a * np.tanh(kt * x)
    
    def control_update_outer(self,
                             e_eta: np.ndarray,
                             dot_e_eta: np.ndarray,
                             dot_eta: np.ndarray,
                             kt: float,
                             m: float,
                             dd_ref: np.ndarray,
                             obs: np.ndarray):
        self.s = dot_e_eta + self.k1 * e_eta + self.k2 * self.sig(e_eta, self.alpha1)
        u1 = -kt / m * dot_eta - dd_ref
        u2 = (self.k1 + self.k2 * self.alpha1 * self.sig(e_eta, self.alpha1 - 1)) * dot_e_eta
        u3 = obs + self.k3 * np.tanh(5 * self.s) + self.k4 * self.sig(self.s, self.alpha2)
        self.control_out = -(u1 + u2 + u3)
    
    def get_param_from_actor(self, action_from_actor: np.ndarray, update_z:bool = False):
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
                 dim: int = 3,
                 dt: float = 0.01):
        super(fntsmc_consensus, self).__init__(param, k1, k2, k3, k4, alpha1, alpha2, dim, dt)
        self.control_out_consensus = np.zeros(self.dim)
    
    def control_update_outer_consensus(self,
                                       d: float,
                                       b: float,
                                       a: np.ndarray,
                                       kt: float,
                                       m: float,
                                       consensus_e: np.ndarray,
                                       consensus_de: np.ndarray,
                                       Lambda_eta: np.ndarray,
                                       dot_eta: np.ndarray,
                                       obs: np.ndarray,
                                       g_obs: np.ndarray
                                       ):
        s = consensus_de + self.k1 * consensus_e + self.k2 * self.sig(consensus_e, self.alpha1)
        sigma = (self.k1 + self.k2 * self.alpha1 * self.sig(consensus_e, self.alpha1 - 1)) * consensus_de
        u1 = -(d + b) * kt / m * dot_eta + sigma - Lambda_eta
        u2 = (d + b) * obs + self.k3 * np.tanh(5 * s) + self.k4 * self.sig(s, self.alpha2)
        for _a_ij, _obs in zip(a, g_obs):
            u2 -= _a_ij * _obs
        self.control_out_consensus = -(u1 + u2) / (d + b)
