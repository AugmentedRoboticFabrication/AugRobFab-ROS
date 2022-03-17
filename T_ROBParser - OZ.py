import glob
import numpy as np
from scipy.spatial.transform import Rotation
from src.util import composeH

class T_ROBParser:
	def __init__(self, fn=None, root=None, cam2tcp=None):
		self.fn = fn
		self.root = root

		if cam2tcp is not None:
			self.cam2tcp = np.load("%s\\%s\\%s" % (self.root, self.fn, cam2tcp))
		else:
			self.cam2tcp = cam2tcp

		# self.cam2tcp = np.eye(4)
		# self.cam2tcp[0][3] = 30
		# self.cam2tcp[1][3] = -1.62
		# self.cam2tcp[2][3] = -1.7

	def tcp2base(self, export=False):
		fn = glob.glob("%s\\%s\\*.mod" % (self.root, self.fn))

		if len(fn) > 1:
			raise RuntimeError("More than 1 .mod files exist!")
		elif len(fn) == 0:
			raise RuntimeError("No .mod files exist!")
		else:
			fn = fn[0]

		file = open(fn, 'r')
		lines = file.readlines()
		
		result = []
		for line in lines:
			tmp = line.split()
			if tmp[0] == "MoveL":
				tmp = tmp[1][1:-1].split(']')
				
				t = tmp[0][1:].split(",")
				t = [float(i) for i in t]
				t = np.asarray(t).reshape(-1,3)

				quat = tmp[1][2:].split(",")
				quat = [float(i) for i in quat]
				quat = [quat[1],quat[2],quat[3],quat[0]]
				
				r = Rotation.from_quat(quat).as_matrix()
				r = np.asarray(r)

				result.append(composeH(r, t))

		if export:
			np.save("%s\\%s\\tcp2base" % (self.root, self.fn), result)

		return result
	
	def cam2base(self, export=False):
		if self.cam2tcp is None:
			raise RuntimeError("No cam2tcp tranformation given!")

		tcp2base = self.tcp2base()
		
		result = []
		for i in range(len(tcp2base)):
			result.append(tcp2base[i]@self.cam2tcp)
		if export:
			np.save("%s\\%s\\cam2base" % (self.root, self.fn), result)
		return result
		




