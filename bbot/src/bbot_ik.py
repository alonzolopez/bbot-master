import numpy as np
import math
from sympy import *
from scipy.spatial.transform import Rotation as R

class bbotAnalysis():
	def __init__(self):
		# Joint Limits [rad]
		self.j1min = math.pi/180.0*(-135.0)
		self.j1max = math.pi/180.0*(135.0)
		self.j2min = math.pi/180.0*(-135.0)
		self.j2max = math.pi/180.0*(135.0)
		self.j3min = math.pi/180.0*(-135.0)
		self.j3max = math.pi/180.0*(135.0)
		self.j4min = math.pi/180.0*(-135.0)
		self.j4max = math.pi/180.0*(135.0)
		self.j5min = math.pi/180.0*(-135.0)
		self.j5max = math.pi/180.0*(135.0)
		self.j6min = math.pi/180.0*(-135.0)
		self.j6max = math.pi/180.0*(135.0)

		# DH Parameters
		self.a2 = 1
		self.a3 = 1
		self.d4 = 1

		# Possible solutions form t1, t5, t3
		self.configs = [
			[1,1,1],
			[1,-1,1],
			[-1,1,1],
			[-1,-1,1],
			[1,1,-1],
			[1,-1,-1],
			[-1,1,-1],
			[-1,-1,-1]
		]
		self.iksol = None

	def IK(self, pose, configoption = 'all', verbose = 'False'):
		if configoption = 'all':
			if verbose = 'False':
				print("Computing IK for all 8 configurations")
			configlist = self.configoption
		r11 = pose[0][0]
		r12 = pose[0][1]
		r13 = pose[0][2]
		r21 = pose[1][0]
		r22 = pose[1][1]
		r23 = pose[1][2]
		r31 = pose[2][0]
		r32 = pose[2][1]
		r33 = pose[2][2]
		px = pose[0][3]
		py = pose[1][3]
		pz = pose[2][3]


		avar1 = -py
		bvar1 = px
		cvar1 = self.d4
		t1 = np.arctan2(np.sqrt(avar1^2+bvar1^2-cvar1^2),cvar1) + np.arctan2(bvar1,avar1) #2 sols for +/- sqrt. no sol if a^2 + b^2 - c^2 < 0

		s5 = np.sin(t1)*r1 - np.cos(t1)*r23

		t5 = np.arctan2(s5, np.sqrt(1 - s5^2)) # 2 sols for +/- sqrt

		s6 = (-np.sin(t1)*r12 + np.cos(t1)*r22)/np.cos(t5)
		c6 = (np.sin(t1)*r11 - np.cos(t1)*r21)/np.cos(t5)

		t6 = np.arctan2(s6,c6)

		s234 = (-np.cos(t1)*r13-np.sin(t1)*r23)/np.cos(t5)
		c234 = r33/np.cos(t5)

		t234 = np.arctan2(s234,c234)

		c3 = (px^2*np.cos(t1)^2 + py^2*np.sin(t1)^2 + pz^2 + 2px*py*np.cos(t1)*np.sin(t1) - self.a2^2 - self.a3^2)/(2*self.a2*self.a3)

		t3 = np.arctan2(np.sqrt(1 - c3^2),c3) # 2 sols for +/- sqrt

		avar6 = pz
		bvar6 = -px*np.cos(t1) - py*np.sin(t1)
		cvar6 = self.a2 + self.a3*np.cos(t3)
		dvar6 = -px*np.cos(t1) - py*np.sin(t1)
		evar6 = -pz
		fvar6 = self.a3*np.sin(t3)

		t3 = np.arctan2(avar6*fvar6 - cvar6*dvar6,cvar6*evar6 - bvar6*fvar6) # A*E - B*D > 0 for sol to exist

		t4 = t234 - t2 - t3
		









