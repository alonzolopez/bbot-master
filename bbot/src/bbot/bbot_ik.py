import numpy as np
import math

def DHToTransMat(ai_1, alphai_1, di, ti):
	mat = np.matrix([
		[np.cos(ti), -np.sin(ti), 0, ai_1],
		[np.sin(ti)*np.cos(alphai_1), np.cos(ti)*np.cos(alphai_1), -np.sin(alphai_1), -np.sin(alphai_1)*di],
		[np.sin(ti)*np.sin(alphai_1), np.cos(ti)*np.sin(alphai_1), np.cos(alphai_1), np.cos(alphai_1)*di],
		[0, 0, 0, 1]
		])
	return mat

def wrapAngle(anglein):
	while anglein > math.pi:
		angleout = anglein - 2*math.pi
	while anglein < -math.pi:
		anglein = anglein + 2*math.pi
	return anglein

def equivalenceCheck(mat1, mat2, eps = 10**(-6)):
	for row in range(mat1.shape[0]):
		for col in range(mat1.shape[1]):
			if np.abs(mat1[row,col]-mat2[row,col]) > eps:
				return False
	return True


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
		self.a2 = 0.105
		self.a3 = 0.134
		self.d4 = 0.0625
		self.a = [0, 0, self.a2, self.a3, 0, 0]
		self.alpha = [0, math.pi/2., 0, 0, math.pi/2., math.pi/2.]
		self.d = [0, 0, 0, self.d4, 0, 0]

		# Possible solutions form t1, t5, t3
		self.possconfigs = [
			[1,1,1,1],
			[1,-1,1,1],
			[-1,1,1,1],
			[-1,-1,1,1],
			[1,1,-1,1],
			[1,-1,-1,1],
			[-1,1,-1,1],
			[-1,-1,-1,1],
			[1,1,1,-1],
			[1,-1,1,-1],
			[-1,1,1,-1],
			[-1,-1,1,-1],
			[1,1,-1,-1],
			[1,-1,-1,-1],
			[-1,1,-1,-1],
			[-1,-1,-1,-1]
		]
		self.targetPose = None
		self.iksols = None
		self.solnconfigs = None

	def IK(self, pose, configoption = 'all', verbose = 'False'):
		if configoption == 'all':
			if verbose == True:
				print("Computing IK for all 16 configurations")
			configlist = self.possconfigs
		else:
			configlist = configoption
		self.targetPose = pose
		solnstemp = []
		configstemp = []

		for i in range(len(configlist)):
			base = configlist[i][0] #theta1
			wrist_yaw = configlist[i][1] #theta5
			elbow = configlist[i][2] #theta3
			shoulder = configlist[i][3] #theta2

			r11 = pose[0,0]
			r12 = pose[0,1]
			r13 = pose[0,2]
			r21 = pose[1,0]
			r22 = pose[1,1]
			r23 = pose[1,2]
			r31 = pose[2,0]
			r32 = pose[2,1]
			r33 = pose[2,2]
			px = pose[0,3]
			py = pose[1,3]
			pz = pose[2,3]


			avar1 = -py
			bvar1 = px
			cvar1 = self.d4
			if (avar1**2 + bvar1**2 < cvar1**2):
				if verbose == True:
					print(str(configlist[i]) + " failed because IK condition for Joint 1 failed")
				continue

			t1 = wrapAngle(np.arctan2(base*np.sqrt(avar1**2+bvar1**2-cvar1**2),cvar1) + np.arctan2(bvar1,avar1)) #2 sols for +/- sqrt. no sol if a^2 + b^2 - c^2 < 0
			if math.isnan(t1):
				if verbose == True:
					print(str(configlist[i]) + " failed because Joint 1 violated with command " + str(t1))
				continue

			s5 = np.sin(t1)*r13 - np.cos(t1)*r23

			t5 = wrapAngle(np.arctan2(s5, wrist_yaw*np.sqrt(1 - s5**2))) # 2 sols for +/- sqrt
			if t5 > self.j5max or t5 < self.j5min or math.isnan(t5):
				if verbose == True:
					print(str(configlist[i]) + " failed because Joint 5 violated with command " + str(t5))
				continue
			if np.cos(t5) == 0:
				if verbose == True:
					print("Theta5 = pi or -pi --> no solution for later angles")
				continue

			s6 = (-np.sin(t1)*r12 + np.cos(t1)*r22)/np.cos(t5)
			c6 = (np.sin(t1)*r11 - np.cos(t1)*r21)/np.cos(t5)

			t6 = wrapAngle(np.arctan2(s6,c6))
			if t6 > self.j6max or t6 < self.j6min or math.isnan(t6):
				if verbose == True:
					print(str(configlist[i]) + " failed because Joint 6 violated with command " + str(t6))
				continue

			s234 = (-np.cos(t1)*r13-np.sin(t1)*r23)/np.cos(t5)
			c234 = r33/np.cos(t5)

			t234 = wrapAngle(np.arctan2(s234,c234))
			if math.isnan(t234):
				if verbose == True:
					print(str(configlist[i]) + " failed because Theta_234 could not be solved with command " + str(t234))
				continue


			c3 = (px**2*np.cos(t1)**2 + py**2*np.sin(t1)**2 + pz**2 + 2*px*py*np.cos(t1)*np.sin(t1) - self.a2**2 - self.a3**2)/(2*self.a2*self.a3)
			t3 = wrapAngle(np.arctan2(elbow*np.sqrt(1 - c3**2),c3)) # 2 sols for +/- sqrt
			if t3 > self.j3max or t3 < self.j3min or math.isnan(t3):
				if verbose == True:
					print(str(configlist[i]) + " failed because Joint 3 violated with command " + str(t3))
				continue

			avar6 = pz
			bvar6 = -px*np.cos(t1) - py*np.sin(t1)
			cvar6 = self.a2 + self.a3*np.cos(t3)
			dvar6 = -px*np.cos(t1) - py*np.sin(t1)
			evar6 = -pz
			fvar6 = self.a3*np.sin(t3)
			t2 = wrapAngle(np.arctan2(shoulder*np.sqrt(avar6**2 + bvar6**2 - cvar6**2),cvar6) + np.arctan2(bvar6,avar6))

			# if (avar6*evar6 - bvar6*dvar6 <= 0):
			# 	if verbose == True:
			# 		print(str(configlist[i]) + " failed the IK condition for Joint 2")
			# 	continue

			# t2 = wrapAngle(np.arctan2(avar6*fvar6 - cvar6*dvar6,cvar6*evar6 - bvar6*fvar6)) # A*E - B*D > 0 for sol to exist
			if t2 > self.j2max or t2 < self.j2min or math.isnan(t2):
				if verbose == True:
					print(str(configlist[i]) + " failed because Joint 6 violated with command " + str(t2))
				continue

			t4 = wrapAngle(t234 - t2 - t3)
			if t4 > self.j4max or t4 < self.j4min or math.isnan(t4):
				if verbose == True:
					print(str(configlist[i]) + " failed because Joint 4 violated with command " + str(t4))
				continue

			if equivalenceCheck(self.FK([t1, t2, t3, t4, t5, t6]),self.targetPose):
				solnstemp.append([t1,t2,t3,t4,t5,t6])
				configstemp.append(configlist[i])
				# print(self.FK([t1,t2,t3,t4,t5,t6]))
			else:
				if verbose == True:
					print(str(configlist[i]) + " failed equivalence test")

		self.iksols = np.matrix(solnstemp)
		self.solnconfigs = np.matrix(configstemp)
		return self.iksols, self.solnconfigs

	def FK(self, jangles):
		T01 = None
		T12 = None
		T23 = None
		T34 = None
		T45 = None
		T56 = None
		T = [T01, T12, T23, T34, T45, T56]
		for i in range(len(T)):
			if i == 1 or i == 4: # Joint angles theta2 and theta5:
				T[i] = DHToTransMat(self.a[i], self.alpha[i], self.d[i], jangles[i]+math.pi/2.)
			else:
				T[i] = DHToTransMat(self.a[i], self.alpha[i], self.d[i], jangles[i])
		T01 = T[0]
		T02 = T01*T[1]
		T03 = T02*T[2]
		T04 = T03*T[3]
		T05 = T04*T[4]
		T06 = T05*T[5]
		return T06

	def Jacobian(self, q):
		T01 = None
		T02 = None
		T03 = None
		T04 = None
		T05 = None
		T06 = None
		Tlink = None
		T = [T01, T02, T03, T04, T05, T06]
		T[0] = DHToTransMat(self.a[0], self.alpha[0], self.d[0],q[0])
		for i in range(1,6):
			Tlink = DHToTransMat(self.a[i], self.alpha[i], self.d[i], q[i])
			T[i] = T[i-1]*Tlink

		J = np.matrix(np.empty((6,6)))
		z0n = None
		p06 = np.squeeze(np.array(T[5][0:3,3].transpose()))
		for i in range(0,6):
			z0n = np.squeeze(np.array(T[i][0:3,2].transpose()))
			p0n = np.squeeze(np.array(T[i][0:3,3].transpose()))
			Jv = np.matrix(np.cross(z0n,p06-p0n)).transpose()
			J[0:3,i] = Jv
			J[3:6,i] = np.matrix(z0n).transpose()
		return J
	

	def invJacobian(self, q):
		J = self.Jacobian(q)
		invJ = J.I
		return invJ



if __name__ == '__main__':
	ikfk = bbotAnalysis()
	# theta3 = -pi/2
	# M = np.matrix([
	# 	[0,0,1,.134],
	# 	[-1,0,0,-.0625],
	# 	[0,-1,0,0.105],
	# 	[0,0,0,1]
	# 	])

	# all zeros
	M = np.matrix([
		[0,1,0,0],
		[-1,0,0,-.0625],
		[0,0,1,0.239],
		[0,0,0,1]
		])

	# t2 = -45, t3 = -90, t4 = 45
	M = np.matrix([
		[0,0,1,(np.sqrt(2)*(.239))/2],
		[-1,0,0,-.0625],
		[0,-1,0,-0.029],
		[0,0,0,1]
		])

	print(ikfk.IK(M,verbose = True))
	print(ikfk.solnconfigs)
	print(ikfk.invJacobian(np.squeeze(np.array(ikfk.iksols))))




