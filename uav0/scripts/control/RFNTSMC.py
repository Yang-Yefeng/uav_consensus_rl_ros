import numpy as np
import rospy


class rfntsmc_param:
    def __init__(self,
                 k1: np.ndarray = np.array([1.2, 0.8, 1.5]),
                 k2: np.ndarray = np.array([0.2, 0.6, 1.5]),
                 alpha: np.ndarray = np.array([1.2, 1.5, 1.2]),
                 beta: np.ndarray = np.array([0.3, 0.3, 0.3]),
                 gamma: np.ndarray = np.array([0.2, 0.2, 0.2]),
                 lmd: np.ndarray = np.array([2.0, 2.0, 2.0]),
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
        self.alpha = alpha
        self.beta = beta
        self.gamma = gamma
        self.lmd = lmd
        
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
        self.alpha = np.array(_p['alpha']).astype(float)
        self.beta = np.array(_p['beta']).astype(float)
        self.gamma = np.array(_p['gamma']).astype(float)
        self.lmd = np.array(_p['lmd']).astype(float)
        self.k_com_pos = np.array(_p['k_com_pos']).astype(float)
        self.k_com_vel = np.array(_p['k_com_vel']).astype(float)
        self.k_com_acc = np.array(_p['k_com_acc']).astype(float)
        self.k_yyf_p = np.array(_p['k_yyf_p']).astype(float)
        self.k_yyf_i = np.array(_p['k_yyf_i']).astype(float)
        self.k_yyf_d = np.array(_p['k_yyf_d']).astype(float)
        self.dim = np.array(_p['dim']).astype(int)
        self.dt = np.array(_p['dt']).astype(float)


class rfntsmc:
    def __init__(self,
                 param: rfntsmc_param = None,
                 k1: np.ndarray = np.array([1.2, 0.8, 1.5]),
                 k2: np.ndarray = np.array([0.2, 0.6, 1.5]),
                 alpha: np.ndarray = np.array([1.2, 1.5, 1.2]),
                 beta: np.ndarray = np.array([0.3, 0.3, 0.3]),
                 gamma: np.ndarray = np.array([0.2, 0.2, 0.2]),
                 lmd: np.ndarray = np.array([2.0, 2.0, 2.0]),
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
        self.alpha = alpha.copy() if param is None else param.alpha.copy()
        self.beta = beta.copy() if param is None else param.beta.copy()
        self.gamma = gamma.copy() if param is None else param.gamma.copy()
        self.lmd = lmd.copy() if param is None else param.lmd.copy()
        
        self.k_com_pos = k_com_pos if param is None else param.k_com_pos.copy()
        self.k_com_vel = k_com_vel if param is None else param.k_com_vel.copy()
        self.k_com_acc = k_com_acc if param is None else param.k_com_acc.copy()
        self.k_yyf_p = yyf_p if param is None else param.k_yyf_p.copy()
        self.k_yyf_i = yyf_i if param is None else param.k_yyf_i.copy()
        self.k_yyf_d = yyf_d if param is None else param.k_yyf_d.copy()
        
        self.dt = dt if param is None else param.dt
        self.dim = dim if param is None else param.dim
        
        self.sigma_o = np.zeros(self.dim)
        self.dot_sigma_o1 = np.zeros(self.dim)
        self.sigma_o1 = np.zeros(self.dim)
        self.so = self.sigma_o + self.lmd * self.sigma_o1
        
        self.control_out = np.zeros(self.dim)
        
        self.yyf_i = np.zeros(3)
        self.yyf_d = np.zeros(3)
        self.yyf_p = np.zeros(3)
    
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
        
        self.sigma_o = (de + self.k_com_vel * d_ref) + self.k1 * e + self.gamma * self.sig(e, self.alpha)
        self.dot_sigma_o1 = self.sig(self.sigma_o, self.beta)
        self.sigma_o1 += self.dot_sigma_o1 * self.dt
        self.so = self.sigma_o + self.lmd * self.sigma_o1
        
        u1 = -(kt / m * dot_eta
               + dd_ref
               - self.k1 * (de + self.k_com_vel * d_ref)
               - self.gamma * self.alpha * np.fabs(e) ** (self.alpha - 1) * (de + self.k_com_vel * d_ref)
               - self.lmd * self.dot_sigma_o1)
        u2 = self.k2 * self.so + obs
        
        self.yyf_i += self.k_yyf_i * e
        self.yyf_p = self.k_yyf_p * e
        self.yyf_d = self.k_yyf_d * de
        
        self.control_out = -(u1 + u2 + self.yyf_i + self.yyf_p + self.yyf_d + self.k_com_acc * dd_ref)
    
    def reset(self):
        self.s = np.zeros(self.dim)
        self.control_out = np.zeros(self.dim)


class rfntsmc_consensus(rfntsmc):
    def __init__(self,
                 param: rfntsmc_param = None,  # 参数
                 k1: np.ndarray = np.array([1.2, 0.8, 1.5]),
                 k2: np.ndarray = np.array([0.2, 0.6, 1.5]),
                 alpha: np.ndarray = np.array([1.2, 1.5, 1.2]),
                 beta: np.ndarray = np.array([0.3, 0.3, 0.3]),
                 gamma: np.ndarray = np.array([0.2, 0.2, 0.2]),
                 lmd: np.ndarray = np.array([2.0, 2.0, 2.0]),
                 k_com_pos: np.ndarray = np.zeros(3),
                 k_com_vel: np.ndarray = np.zeros(3),
                 k_com_acc: np.ndarray = np.zeros(3),
                 yyf_p: np.ndarray = np.zeros(3),
                 yyf_i: np.ndarray = np.zeros(3),
                 yyf_d: np.ndarray = np.zeros(3),
                 dim: int = 3,
                 dt: float = 0.01):
        super(rfntsmc_consensus, self).__init__(param, k1, k2, alpha, beta, gamma, lmd,
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
                                       dd_ref: np.ndarray,
                                       nu: np.ndarray,
                                       d_nu: np.ndarray,
                                       dd_nu: np.ndarray,
                                       e_max: float = 0.2,
                                       dot_e_max: float = 1.5):
        if e_max is not None:  # 增加对位置误差的输入饱和
            consensus_e = np.clip(consensus_e, -e_max, e_max)
        if dot_e_max is not None:  # 增加对速度误差的输入饱和
            consensus_de = np.clip(consensus_de, -dot_e_max, dot_e_max)

        pos_com = b * self.k_com_pos * ref + self.k_com_pos * nu
        vel_com = b * self.k_com_vel * d_ref + self.k_com_vel * d_nu
        acc_com = b * self.k_com_acc * dd_ref + self.k_com_acc * dd_nu

        self.sigma_o = (consensus_de + vel_com) + self.k1 * (consensus_e + pos_com) + self.gamma * self.sig((consensus_e + pos_com), self.alpha)
        self.dot_sigma_o1 = self.sig(self.sigma_o, self.beta)
        self.sigma_o1 += self.dot_sigma_o1 * self.dt
        self.so = self.sigma_o + self.lmd * self.sigma_o1
        
        u1 = (Lambda_eta + self.k1 * (consensus_de + vel_com) +
              self.gamma * self.alpha * np.fabs((consensus_e + pos_com)) ** (self.alpha - 1) * (consensus_de + vel_com) +
              self.lmd * self.dot_sigma_o1)
        u2 = self.k2 * self.so
        
        self.yyf_i += self.k_yyf_i * consensus_e
        self.yyf_p = self.k_yyf_p * consensus_e
        self.yyf_d = self.k_yyf_d * consensus_de
        
        self.control_out_consensus = -(u1 + u2 + self.yyf_i + self.yyf_p + self.yyf_d + acc_com) / (d + b)
