import numpy as np
from scipy.spatial.transform import Rotation

def composeH(R, t, inv=False):
	M = np.eye(4)
	if inv:
		R = Rotation.from_matrix(np.asarray(R)).inv().as_matrix()
		t *= -1
	M[:3, :3] = np.asarray(R)
	M[:3, 3] =  np.asarray(t).ravel()
	return M