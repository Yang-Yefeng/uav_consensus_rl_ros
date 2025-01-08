import numpy as np
from typing import Union
import rospy
import pandas as pd
from scipy.special import gamma


class robust_differentiator_3rd:
	def __init__(self,
				 m1: Union[np.ndarray, list] = np.array([0, 0, 0]),
				 m2: Union[np.ndarray, list] = np.array([0, 0, 0]),
				 m3: Union[np.ndarray, list] = np.array([0, 0, 0]),
				 n1: Union[np.ndarray, list] = np.array([0, 0, 0]),
				 n2: Union[np.ndarray, list] = np.array([0, 0, 0]),
				 n3: Union[np.ndarray, list] = np.array([0, 0, 0]),
				 use_freq: bool = False,
				 omega: Union[np.ndarray, list] = np.atleast_2d([0, 0, 0]),
				 dim: int = 3,
				 dt: float = 0.001):
		self.m1 = np.zeros(dim)
		self.m2 = np.zeros(dim)
		self.m3 = np.zeros(dim)
		self.n1 = np.zeros(dim)
		self.n2 = np.zeros(dim)
		self.n3 = np.zeros(dim)
		self.a1 = 3. / 4.
		self.a2 = 2. / 4.
		self.a3 = 1. / 4.
		self.b1 = 5. / 4.
		self.b2 = 6. / 4.
		self.b3 = 7. / 4.
		if use_freq:
			for i in range(dim):
				_omega = omega[i]
				m1n1 = _omega[0] + _omega[1] + _omega[2]
				m2n2 = _omega[0] * _omega[1] + _omega[0] * _omega[2] + _omega[1] * _omega[2]
				m3n3 = _omega[0] * _omega[1] * _omega[2]
				self.m1[i] = m1n1
				self.m2[i] = m2n2
				self.m3[i] = m3n3
				self.n1[i] = m1n1
				self.n2[i] = m2n2
				self.n3[i] = m3n3
		else:
			self.m1 = np.array(m1)
			self.m2 = np.array(m2)
			self.m3 = np.array(m3)
			self.n1 = np.array(n1)
			self.n2 = np.array(n2)
			self.n3 = np.array(n3)

		self.z1 = np.zeros(dim)
		self.z2 = np.zeros(dim)
		self.z3 = np.zeros(dim)
		self.dz1 = np.zeros(dim)
		self.dz2 = np.zeros(dim)
		self.dz3 = np.zeros(dim)
		self.dim = dim
		self.dt = dt

		self.threshold = np.array([0.001, 0.001, 0.001])
	
	def load_param_from_yaml(self, name: str):
		_p = rospy.get_param(name)
		self.dim = int(_p['dim'])
		self.dt = _p['dt']
		self.m1 = np.zeros(self.dim)
		self.m2 = np.zeros(self.dim)
		self.m3 = np.zeros(self.dim)
		self.n1 = np.zeros(self.dim)
		self.n2 = np.zeros(self.dim)
		self.n3 = np.zeros(self.dim)
		
		if _p['use_freq']:
			_w = np.array(_p['omega']).astype(float)
			for i in range(self.dim):
				_omega = _w[i]
				m1n1 = _omega[0] + _omega[1] + _omega[2]
				m2n2 = _omega[0] * _omega[1] + _omega[0] * _omega[2] + _omega[1] * _omega[2]
				m3n3 = _omega[0] * _omega[1] * _omega[2]
				self.m1[i] = m1n1
				self.m2[i] = m2n2
				self.m3[i] = m3n3
				self.n1[i] = m1n1
				self.n2[i] = m2n2
				self.n3[i] = m3n3
		self.z1 = np.zeros(self.dim)
		self.z2 = np.zeros(self.dim)
		self.z3 = np.zeros(self.dim)
		self.dz1 = np.zeros(self.dim)
		self.dz2 = np.zeros(self.dim)
		self.dz3 = np.zeros(self.dim)
	
	def set_init(self, e0: Union[np.ndarray, list], de0: Union[np.ndarray, list], syst_dynamic: Union[np.ndarray, list]):
		self.z1 = np.array(e0)
		self.z2 = np.array(de0)
		self.z3 = np.zeros(self.dim)

		self.dz1 = self.z2.copy()
		self.dz2 = self.z3.copy() + syst_dynamic
		self.dz3 = np.zeros(self.dim)

	@staticmethod
	def sig(x: Union[np.ndarray, list], a):
		return np.fabs(x) ** a * np.sign(x)

	def fal(self, xi: Union[np.ndarray, list], a):
		res = []
		for i in range(self.dim):
			if np.fabs(xi[i]) <= self.threshold[i]:
				res.append(xi[i] / (self.threshold[i] ** np.fabs(1 - a)))
			else:
				res.append(np.fabs(xi[i]) ** a * np.sign(xi[i]))
		return np.array(res)

	def observe(self, syst_dynamic: Union[np.ndarray, list], e: Union[np.ndarray, list]):
		obs_e = e - self.z1
		self.dz1 = self.z2 + self.m1 * self.sig(obs_e, self.a1) + self.n1 * self.sig(obs_e, self.b1)
		self.dz2 = syst_dynamic + self.z3 + self.m2 * self.sig(obs_e, self.a2) + self.n2 * self.sig(obs_e, self.b2)
		self.dz3 = self.m3 * self.sig(obs_e, self.a3) + self.n3 * self.sig(obs_e, self.b3)
		self.z1 = self.z1 + self.dz1 * self.dt
		self.z2 = self.z2 + self.dz2 * self.dt
		self.z3 = self.z3 + self.dz3 * self.dt

		return self.z3


class predefined_time_do:
	def __init__(self,
				 T0: np.ndarray = 2 * np.ones(3),
				 k1: np.ndarray = np.ones(3),
				 alpha: np.ndarray = 0.5 * np.ones(3),
				 beta1: np.ndarray = np.ones(3),
				 beta2: np.ndarray = np.ones(3),
				 beta3: np.ndarray = np.ones(3),
				 dim: int = 3,
				 dt: float = 0.01
				 ):
		self.T0 = T0
		self.k1 = k1
		self.beta1 = beta1
		self.beta2 = beta2
		self.beta3 = beta3
		self.alpha = alpha
		self.dim = dim
		self.dt = dt
		
		haha = 1.
		self.a1 = 0.5 * np.ones(self.dim)
		self.a2 = 2 ** (-self.alpha)
		self.a3 = 0.5 * np.ones(self.dim) * haha ** (1 - self.alpha)
		
		self.b1 = 1 * np.ones(self.dim)
		self.b2 = (1 - self.alpha) / (2 ** (self.alpha - 1))
		self.b3 = (2 - self.alpha) / (3 - self.alpha) * haha ** (1 - self.alpha)
		
		# self.a1 = 0.5 * np.ones(self.dim)
		# self.a2 = 2 ** (-self.alpha)
		# self.a3 = 0.5 * np.ones(self.dim)
		#
		# self.b1 = 1 * np.ones(self.dim)
		# self.b2 = (1 - self.alpha) / (2 ** (self.alpha - 1))
		# self.b3 = (2 - self.alpha) / (3 - self.alpha)
		
		a1s = (1 - self.alpha * self.k1) / (2 - 2 * self.alpha)
		a2s = self.k1 - a1s
		self.k0 = gamma(a1s) * gamma(a2s) / gamma(self.k1) / (2 - 2 * self.alpha) / self.beta2 ** self.k1 * (self.beta2 / self.beta3) ** a1s / self.T0
		
		self.xi = np.zeros(self.dim)  # velicity estimation error
		
		self.dx = np.zeros(self.dim)
		self.x = np.zeros(self.dim)  #
		
		self.d_sigma = np.zeros(self.dim)
		self.sigma = 1 * np.ones(self.dim)
		
		self.delta = np.zeros(self.dim)
		
		'''data record'''
		self.index = 0
		self.rec_t = np.empty(shape=[0, 1], dtype=float)
		self.rec_xi = np.empty(shape=[0, 3], dtype=float)
		self.rec_x = np.empty(shape=[0, 3], dtype=float)
		self.rec_dx = np.empty(shape=[0, 3], dtype=float)
		self.rec_d_sigma = np.empty(shape=[0, 3], dtype=float)
		self.rec_sigma = np.empty(shape=[0, 3], dtype=float)
		self.rec_delta = np.empty(shape=[0, 3], dtype=float)
		'''data record'''
	
	@staticmethod
	def sig(x: Union[np.ndarray, list], a, kt=5):
		return np.fabs(x) ** a * np.tanh(kt * x)
	
	def set_init(self, de0: np.ndarray, dy0: np.ndarray):
		self.xi = de0 - self.x
		self.dx = dy0
	
	def observe(self, dy: Union[np.ndarray, list], de: Union[np.ndarray, list]):
		self.xi = de - self.x  # 速度估计的误差
		
		_dx1 = 2 * self.a1 * self.beta1 * self.sig(self.xi, 2 - 1 / self.k1)
		_dx2 = self.a2 * self.beta2 * self.sig(self.xi, 2 * self.alpha - 1 / self.k1)
		_dx3 = self.a3 * self.beta3 * self.sig(self.xi, 4 - 2 * self.alpha - 1 / self.k1)
		self.dx = self.k0 * (_dx1 + _dx2 + _dx3) ** self.k1 + self.sigma * np.sign(self.xi) + dy
		
		_d_sigma1 = 2 * self.b1 * self.beta1 * self.sig(self.sigma, 2 - 1 / self.k1)
		_d_sigma2 = self.b2 * self.beta2 * self.sig(self.sigma, 2 - 1 / self.k1)
		_d_sigma3 = self.b3 * self.beta3 * self.sig(self.sigma, 4 - 2 * self.alpha - 1 / self.k1)
		self.d_sigma = -self.k0 * (_d_sigma1 + _d_sigma2 + _d_sigma3) ** self.k1 + self.xi * np.sign(self.xi)
		
		self.delta = self.k0 * (_dx1 + _dx2 + _dx3) ** self.k1 + self.sigma * np.sign(self.xi)
		
		self.x = self.x + self.dt * self.dx
		self.sigma = self.sigma + self.dt * self.d_sigma
		
		'''datasave'''
		self.rec_t = np.append(self.rec_t, np.array([[self.index * self.dt]]), axis=0)
		self.rec_xi = np.append(self.rec_xi, [self.xi], axis=0)
		self.rec_dx = np.append(self.rec_dx, [self.dx], axis=0)
		self.rec_x = np.append(self.rec_x, [self.x], axis=0)
		self.rec_d_sigma = np.append(self.rec_d_sigma, [self.d_sigma], axis=0)
		self.rec_sigma = np.append(self.rec_sigma, [self.sigma], axis=0)
		self.rec_delta = np.append(self.rec_delta, [self.delta], axis=0)
		self.index += 1
		'''datasave'''
		
		return self.delta
	
	def save_adap_obs_param(self, path: str, flag: str = 'pos'):
		pd.DataFrame(np.hstack((self.rec_t, self.rec_xi, self.rec_dx, self.rec_x, self.rec_d_sigma, self.rec_sigma, self.rec_delta)),
					 columns=['time',
							  'xi_x', 'xi_y', 'xi_z',
							  'dx_x', 'dx_y', 'dx_z',
							  'x_x', 'x_y', 'x_z',
							  'd_sigma_x', 'd_sigma_y', 'd_sigma_z',
							  'sigma_x', 'sigma_y', 'sigma_z',
							  'delta_x', 'delta_y', 'delta_z']). \
			to_csv(path + flag + '_adap_obs_param.csv', sep=',', index=False)
