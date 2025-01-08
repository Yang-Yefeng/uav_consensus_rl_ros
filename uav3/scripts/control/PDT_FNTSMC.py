import numpy as np
from scipy.special import gamma
import rospy


class pdt_fntsmc_param:
    def __init__(self,
                 a_s: np.ndarray = np.array([0.5, 0.5, 0.5]).astype(float),     # 滑模里面的 alpha
                 b1_s: np.ndarray = np.ones(3).astype(float),                    # 滑模里面的 beta1
                 b2_s: np.ndarray = np.ones(3).astype(float),                    # 滑模里面的 beta2
                 b3_s: np.ndarray = np.ones(3).astype(float),                    # 滑模里面的 beta3
                 Ts: np.ndarray = np.array([5., 5., 5.]).astype(float),             # 滑模里面的预设时间
                 k1_s: np.ndarray = np.array([1., 1., 1.]).astype(float),           # 滑模里面的 kappa1
                 a_c: np.ndarray = np.array([0.5, 0.5, 0.5]).astype(float),     # 控制器里面的 alpha
                 b1_c: np.ndarray = np.ones(3).astype(float),                    # 控制器里面的 beta1
                 b2_c: np.ndarray = np.ones(3).astype(float),                    # 控制器里面的 beta2
                 b3_c: np.ndarray = np.ones(3).astype(float),                    # 控制器里面的 beta3
                 Tc: np.ndarray = np.array([10., 10., 10.]).astype(float),             # 控制器里面的预设时间
                 k1_c: np.ndarray = np.array([1., 1., 1.]).astype(float),           # 控制器里面的 kappa1
                 k2:np.ndarray=np.array([0., 0., 0.]).astype(float),        # 补偿
                 k_com_pos: np.ndarray = np.zeros(3),
                 k_com_vel: np.ndarray = np.zeros(3),
                 k_com_acc: np.ndarray = np.zeros(3),
                 k_yyf_p: np.ndarray = np.zeros(3),
                 k_yyf_i: np.ndarray = np.zeros(3),
                 k_yyf_d: np.ndarray = np.zeros(3),
                 dim: int = 3,
                 dt: float = 0.01):
        self.a_s = a_s
        self.b1_s = b1_s
        self.b2_s = b2_s
        self.b3_s = b3_s
        self.k1_s = k1_s
        self.Ts = Ts

        self.a_c = a_c
        self.b1_c = b1_c
        self.b2_c = b2_c
        self.b3_c = b3_c
        self.k1_c = k1_c
        self.Tc = Tc
        self.k2 = k2
        
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
        self.a_s = np.array(_p['alpha_s']).astype(float)
        self.b1_s = np.array(_p['beta1_s']).astype(float)
        self.b2_s = np.array(_p['beta2_s']).astype(float)
        self.b3_s = np.array(_p['beta3_s']).astype(float)
        self.k1_s = np.array(_p['kappa1_s']).astype(float)
        self.Ts = np.array(_p['Ts']).astype(float)
        
        self.a_c = np.array(_p['alpha_c']).astype(float)
        self.b1_c = np.array(_p['beta1_c']).astype(float)
        self.b2_c = np.array(_p['beta2_c']).astype(float)
        self.b3_c = np.array(_p['beta3_c']).astype(float)
        self.k1_c = np.array(_p['kappa1_c']).astype(float)
        self.Tc = np.array(_p['Tc']).astype(float)
        self.k2 = np.array(_p['k2']).astype(float)
        
        self.k_com_pos = np.array(_p['k_com_pos']).astype(float)
        self.k_com_vel = np.array(_p['k_com_vel']).astype(float)
        self.k_com_acc = np.array(_p['k_com_acc']).astype(float)
        self.k_yyf_p = np.array(_p['k_yyf_p']).astype(float)
        self.k_yyf_i = np.array(_p['k_yyf_i']).astype(float)
        self.k_yyf_d = np.array(_p['k_yyf_d']).astype(float)
        
        self.dim = np.array(_p['dim']).astype(int)
        self.dt = np.array(_p['dt']).astype(float)


class pdt_fntsmc:
    def __init__(self,
                 param: pdt_fntsmc_param = None,
                 a_s: np.ndarray = np.array([0.5, 0.5, 0.5]).astype(float),
                 b1_s: np.ndarray = np.ones(3).astype(float),
                 b2_s: np.ndarray = np.ones(3).astype(float),
                 b3_s: np.ndarray = np.ones(3).astype(float),
                 Ts: np.ndarray = np.array([5., 5., 5.]).astype(float),
                 k1_s: np.ndarray = np.array([1., 1., 1.]).astype(float),
                 a_c: np.ndarray = np.array([0.5, 0.5, 0.5]).astype(float),
                 b1_c: np.ndarray = np.ones(3).astype(float),
                 b2_c: np.ndarray = np.ones(3).astype(float),
                 b3_c: np.ndarray = np.ones(3).astype(float),
                 Tc: np.ndarray = np.array([5., 5., 5.]).astype(float),
                 k1_c: np.ndarray = np.array([1., 1., 1.]).astype(float),
                 k2: np.ndarray = np.array([0., 0., 0.]).astype(float),
                 k_com_pos: np.ndarray = np.zeros(3),
                 k_com_vel: np.ndarray = np.zeros(3),
                 k_com_acc: np.ndarray = np.zeros(3),
                 yyf_p: np.ndarray = np.zeros(3),
                 yyf_i: np.ndarray = np.zeros(3),
                 yyf_d: np.ndarray = np.zeros(3),
                 dim: int = 3,
                 dt: float = 0.01
                 ):
        self.a_s = a_s if param is None else param.a_s
        self.b1_s = b1_s if param is None else param.b1_s
        self.b2_s = b2_s if param is None else param.b2_s
        self.b3_s = b3_s if param is None else param.b3_s
        self.Ts = Ts if param is None else param.Ts
        self.k1_s = k1_s if param is None else param.k1_s
        
        a1s = (1 - self.a_s * self.k1_s) / (2 - 2 * self.a_s)
        a2s = self.k1_s - a1s
        self.k0_s = gamma(a1s) * gamma(a2s) / gamma(self.k1_s) / (2 - 2 * self.a_s) / self.b2_s ** self.k1_s * (self.b2_s / self.b3_s) ** a1s / self.Ts
        
        self.a_c = a_c if param is None else param.a_c
        self.b1_c = b1_c if param is None else param.b1_c
        self.b2_c = b2_c if param is None else param.b2_c
        self.b3_c = b3_c if param is None else param.b3_c
        self.Tc = Tc if param is None else param.Tc
        self.k1_c = k1_c if param is None else param.k1_c
        
        a1c = (1 - self.a_s * self.k1_s) / (2 - 2 * self.a_s)
        a2c = self.k1_s - a1c
        self.k0_c = gamma(a1c) * gamma(a2c) / gamma(self.k1_c) / (2 - 2 * self.a_c) / self.b2_c ** self.k1_c * (self.b2_c / self.b3_c) ** a1c / self.Tc
        
        self.k2 = k2 if param is None else param.k2
        
        self.k_com_pos = k_com_pos if param is None else param.k_com_pos.copy()
        self.k_com_vel = k_com_vel if param is None else param.k_com_vel.copy()
        self.k_com_acc = k_com_acc if param is None else param.k_com_acc.copy()
        self.k_yyf_p = yyf_p if param is None else param.k_yyf_p.copy()
        self.k_yyf_i = yyf_i if param is None else param.k_yyf_i.copy()
        self.k_yyf_d = yyf_d if param is None else param.k_yyf_d.copy()
        
        self.yyf_i = np.zeros(3)
        self.yyf_d = np.zeros(3)
        self.yyf_p = np.zeros(3)
        
        self.dt = dt if param is None else param.dt
        self.dim = dim if param is None else param.dim
        self.s = np.zeros(self.dim)
        self.control_in = np.zeros(self.dim)
        self.control_out = np.zeros(self.dim)
    
    def print_param(self):
        print('==== For sliding surface ====')
        print('alpha_s:', self.a_s)
        print('b1_s:', self.b1_s)
        print('b2_s:', self.b2_s)
        print('b3_s:', self.b3_s)
        print('k1_s:', self.k1_s)
        print('Ts:', self.Ts)
        print('k0_s:', self.k0_s)
        print('==== For sliding surface ====')
        
        print('==== For controller ====')
        print('alpha_c:', self.a_c)
        print('b1_c:', self.b1_c)
        print('b2_c:', self.b2_c)
        print('b3_c:', self.b3_c)
        print('k1_c:', self.k1_c)
        print('Tc:', self.Tc)
        print('k0_c:', self.k0_c)
        print('==== For controller ====')
    
    @staticmethod
    def sig(x, a, kt=5):
        return np.fabs(x) ** a * np.tanh(kt * x)
    
    @staticmethod
    def sig_sign(x, a, th=0.01):
        res = []
        for _x, _a in zip(x, a):
            if np.fabs(_x) <= th:  # 这个数奇异
                # res.append(np.sin(np.pi / 2 / th) * np.fabs(_x) ** (-_a))
                res.append(0.)
            else:
                if _x < 0:
                    res.append(np.fabs(_x) ** _a * np.sign(_x))
                else:
                    res.append(np.fabs(_x) ** _a * np.sign(_x))
        return np.array(res)
    
    @staticmethod
    def singu(x, a, th=0.01):
        # 专门处理奇异的函数
        x = np.fabs(x)
        res = []
        for _x, _a in zip(x, a):
            if _x > th:
                # res.append(self.sig(_x, _a, kt))
                res.append(1.0)
            else:
                res.append(np.sin(np.pi / 2 / th) * _x ** np.fabs(_a))
        return np.array(res)
    
    @staticmethod
    def sig_sign_singu(x, a, th=0.01):
        res = []
        for _x, _a in zip(x, a):
            if np.fabs(_x) <= th:  # 这个数奇异
                res.append(np.sin(np.pi / 2 / th) * np.fabs(_x) ** np.fabs(_a))
                # res.append(0.)
            else:
                if _x < 0:
                    res.append(np.fabs(_x) ** _a * np.tanh(5 * _x))
                else:
                    res.append(np.fabs(_x) ** _a * np.tanh(5 * _x))
        return np.array(res)

    def control_update_outer(self,
                             e_eta: np.ndarray,
                             dot_e_eta: np.ndarray,
                             dot_eta: np.ndarray,
                             kt: float,
                             m: float,
                             ref: np.ndarray,
                             d_ref: np.ndarray,
                             dd_ref: np.ndarray,
                             obs: np.ndarray,
                             e_m: np.ndarray = np.array([2, 2, 2]).astype(float),
                             de_m: np.ndarray = np.array([1.5, 1.5, 1.5]).astype(float)):
        e_eta = np.clip(e_eta, -e_m, e_m)
        dot_e_eta = np.clip(dot_e_eta, -de_m, de_m)
        
        '''calculate sliding mode'''
        _s1 = self.b1_s * self.sig_sign_singu(e_eta + self.k_com_pos * ref, 2 - 1 / self.k1_s)
        _s2 = self.b2_s * self.sig_sign_singu(e_eta + self.k_com_pos * ref, 2 * self.a_s - 1 / self.k1_s) / 2.0
        _s3 = self.b3_s * self.sig_sign_singu(e_eta + self.k_com_pos * ref, 4 - 2 * self.a_s - 1 / self.k1_s) / 2.0
        self.s = dot_e_eta + self.k_com_vel * d_ref + self.k0_s * self.sig_sign_singu(_s1 + _s2 + _s3, self.k1_s)
        '''calculate sliding mode'''
        
        _m1 = self.b1_s * self.sig_sign_singu(e_eta + self.k_com_pos * ref, 2 - 1 / self.k1_s)
        _m2 = self.b2_s * self.sig_sign_singu(e_eta + self.k_com_pos * ref, 2 * self.a_s - 1 / self.k1_s) / 2.0
        _m3 = self.b3_s * self.sig_sign_singu(e_eta + self.k_com_pos * ref, 4 - 2 * self.a_s - 1 / self.k1_s) / 2.0
        M_rho1 = self.k0_s * self.k1_s * self.sig_sign_singu(_m1 + _m2 + _m3, self.k1_s - 1)
        M_rho2 = (self.b1_s * (2 - 1 / self.k1_s) * self.sig_sign_singu(e_eta + self.k_com_pos * ref, 1 - 1 / self.k1_s) +
                  self.b2_s * (2 * self.a_s - 1 / self.k1_s) * self.singu(e_eta + self.k_com_pos * ref, 2 * self.a_s - 1 / self.k1_s - 1) / 2.0 +
                  self.b3_s * (4 - 2 * self.a_s - 1 / self.k1_s) *
                  self.sig_sign_singu(e_eta + self.k_com_pos * ref, 3 - 2 * self.a_s - 1 / self.k1_s) / 2.0) * (dot_e_eta + self.k_com_vel * d_ref)
        M_rho = M_rho1 * M_rho2
        
        u1 = kt / m * dot_eta - dd_ref + M_rho
        u2 = obs
        _t1 = self.b1_c * self.sig_sign_singu(self.s, 2 - 1 / self.k1_c)
        _t2 = self.b2_c * self.sig_sign_singu(self.s, 2 * self.a_c - 1 / self.k1_c) / 2.0
        _t3 = self.b3_c * self.sig_sign_singu(self.s, 4 - 2 * self.a_c - 1 / self.k1_c) / 2.0
        u3 = self.k0_c * self.sig_sign_singu(_t1 + _t2 + _t3, self.k1_c) + self.k2 * np.tanh(5 * self.s)
        
        self.yyf_i += self.k_yyf_i * e_eta
        self.yyf_p = self.k_yyf_p * e_eta
        self.yyf_d = self.k_yyf_d * dot_e_eta
        
        self.control_out = -(u1 + u2 + u3 * self.yyf_i + self.yyf_p + self.yyf_d + self.k_com_acc * dd_ref)
    
    def get_param_from_actor(self, action_from_actor: np.ndarray, update_z:bool=False):
        """ TODO """
        pass
    
    def reset_with_new_param(self, param: pdt_fntsmc_param):
        """ TODO """
        self.dim = param.dim
        self.dt = param.dt
        
        self.s = np.zeros(self.dim)
        self.control_in = np.zeros(self.dim)
        self.control_out = np.zeros(self.dim)
    
    def reset(self):
        self.s = np.zeros(self.dim)
        self.control_in = np.zeros(self.dim)
        self.control_out = np.zeros(self.dim)


class pdt_fntsmc_consensus(pdt_fntsmc):
    def __init__(self,
                 param: pdt_fntsmc_param = None,
                 a_s: np.ndarray = np.array([0.5, 0.5, 0.5]).astype(float),
                 b1_s: np.ndarray = np.ones(3).astype(float),
                 b2_s: np.ndarray = np.ones(3).astype(float),
                 b3_s: np.ndarray = np.ones(3).astype(float),
                 Ts: np.ndarray = np.array([5., 5., 5.]).astype(float),
                 k1_s: np.ndarray = np.array([1., 1., 1.]).astype(float),
                 a_c: np.ndarray = np.array([0.5, 0.5, 0.5]).astype(float),
                 b1_c: np.ndarray = np.ones(3).astype(float),
                 b2_c: np.ndarray = np.ones(3).astype(float),
                 b3_c: np.ndarray = np.ones(3).astype(float),
                 Tc: np.ndarray = np.array([5., 5., 5.]).astype(float),
                 k1_c: np.ndarray = np.array([1., 1., 1.]).astype(float),
                 k2: np.ndarray = np.array([0., 0., 0.]).astype(float),
                 k_com_pos: np.ndarray = np.zeros(3),
                 k_com_vel: np.ndarray = np.zeros(3),
                 k_com_acc: np.ndarray = np.zeros(3),
                 yyf_p: np.ndarray = np.zeros(3),
                 yyf_i: np.ndarray = np.zeros(3),
                 yyf_d: np.ndarray = np.zeros(3),
                 dim: int = 3,
                 dt: float = 0.01
                 ):
        super(pdt_fntsmc_consensus, self).__init__(param,
                                                   a_s, b1_s, b2_s, b3_s, Ts, k1_s,
                                                   a_c, b1_c, b2_c, b3_c, Tc, k1_c, k2,
                                                   k_com_pos, k_com_vel, k_com_acc, yyf_p, yyf_i, yyf_d,
                                                   dim, dt)
        self.control_out_consensus = np.zeros(self.dim)
    
    def control_update_outer_consensus(self,
                                       d: float,
                                       b: float,
                                       consensus_e: np.ndarray,
                                       consensus_de: np.ndarray,
                                       Lambda_eta: np.ndarray,
                                       ref: np.ndarray,
                                       d_ref: np.ndarray,
                                       dd_ref: np.ndarray,
                                       nu: np.ndarray,
                                       d_nu: np.ndarray,
                                       dd_nu: np.ndarray,
                                       e_max: np.ndarray = np.array([2, 2, 2]).astype(float),
                                       dot_e_max: np.ndarray = np.array([1.5, 1.5, 1.5]).astype(float)):
        e_eta = np.clip(consensus_e, -e_max, e_max)
        dot_e_eta = np.clip(consensus_de, -dot_e_max, dot_e_max)
        
        pos_com = b * self.k_com_pos * ref + self.k_com_pos * nu
        vel_com = b * self.k_com_vel * d_ref + self.k_com_vel * d_nu
        acc_com = b * self.k_com_acc * dd_ref + self.k_com_acc * dd_nu
        
        '''calculate sliding mode'''
        _s1 = self.b1_s * self.sig_sign_singu(e_eta + pos_com, 2 - 1 / self.k1_s)
        _s2 = self.b2_s * self.sig_sign_singu(e_eta + pos_com, 2 * self.a_s - 1 / self.k1_s) / 2.0
        _s3 = self.b3_s * self.sig_sign_singu(e_eta + pos_com, 4 - 2 * self.a_s - 1 / self.k1_s) / 2.0
        self.s = dot_e_eta + vel_com + self.k0_s * self.sig_sign_singu(_s1 + _s2 + _s3, self.k1_s)
        '''calculate sliding mode'''
        
        _m1 = self.b1_s * self.sig_sign_singu(e_eta + pos_com, 2 - 1 / self.k1_s)
        _m2 = self.b2_s * self.sig_sign_singu(e_eta + pos_com, 2 * self.a_s - 1 / self.k1_s) / 2.0
        _m3 = self.b3_s * self.sig_sign_singu(e_eta + pos_com, 4 - 2 * self.a_s - 1 / self.k1_s) / 2.0
        M_rho1 = self.k0_s * self.k1_s * self.sig_sign_singu(_m1 + _m2 + _m3, self.k1_s - 1)
        M_rho2 = (self.b1_s * (2 - 1 / self.k1_s) * self.sig_sign_singu(e_eta + pos_com, 1 - 1 / self.k1_s) +
                  self.b2_s * (2 * self.a_s - 1 / self.k1_s) * self.singu(e_eta + pos_com, 2 * self.a_s - 1 / self.k1_s - 1) / 2.0 +
                  self.b3_s * (4 - 2 * self.a_s - 1 / self.k1_s) * self.sig_sign_singu(e_eta + pos_com, 3 - 2 * self.a_s - 1 / self.k1_s) / 2.0) * (dot_e_eta + vel_com)
        M_rho = M_rho1 * M_rho2
        
        u1 = -Lambda_eta + M_rho
        _t1 = self.b1_c * self.sig_sign_singu(self.s, 2 - 1 / self.k1_c)
        _t2 = self.b2_c * self.sig_sign_singu(self.s, 2 * self.a_c - 1 / self.k1_c) / 2.0
        _t3 = self.b3_c * self.sig_sign_singu(self.s, 4 - 2 * self.a_c - 1 / self.k1_c) / 2.0
        u2 = self.k0_c * self.sig_sign_singu(_t1 + _t2 + _t3, self.k1_c) + self.k2 * np.tanh(5 * self.s)
        
        self.yyf_i += self.k_yyf_i * consensus_e
        self.yyf_p = self.k_yyf_p * consensus_e
        self.yyf_d = self.k_yyf_d * consensus_de
        
        self.control_out_consensus = -(u1 + u2 + self.yyf_i + self.yyf_p + self.yyf_d + acc_com) / (d + b)
