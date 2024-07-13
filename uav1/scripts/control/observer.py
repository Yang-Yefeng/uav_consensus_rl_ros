import numpy as np
from typing import Union
import rospy


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
