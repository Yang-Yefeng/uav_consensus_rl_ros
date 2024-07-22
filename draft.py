import numpy as np

set = np.array([[1,1,1], [2,2,2], [3,3,3], [4,4,4]])
t_miemie = 5
tm = 20
_p = set.shape[0]
step = int(tm / _p)
NN = int((tm + t_miemie) / 1)
N = int(t_miemie / 1)
ref = None
for i in range(_p):
    if ref is None:
        ref = np.tile(set[i], (step, 1))
    else:
        ref = np.vstack((ref, np.tile(set[i], (step, 1))))

ref = np.vstack((np.tile(set[0], (N, 1)), ref))

print(ref)