"""
The MixingTaskStateMachine Class holds all the object poses that act as a model of the task space for use in the mixing operation
"""
import numpy as np

class MixingTaskStateMachine():
	def __init__(self):
		# robot poses
		self.TBG = np.matrix([
			[0.0, 0.0, 1.0, 0.08191],
			[0.0, -1.0, 0.0, 0.0],
			[1.0, 0.0, 0.0, 0.026],
			[0.0, 0.0, 0.0, 1.0]
			]) # 4x4 transform from base to base_aruco (aruco described in base frame)
		self.TGB = np.linalg.inv(self.TBG) # 4x4 transform from base_aruco to base (base described in aruco frame)

		# camera poses
		self.TCG = None # 4x4 transform from camera to base_aruco (base_aruco described in camera)
		self.TGC = None # 4x4 transform from base_aruco to camera (camera described in base_aruco)
		self.TBC = None # 4x4 transform from base to camera (camera described in base)

		# cup 1 poses
		self.TCA = None # 4x4 transform from camera to cup1's aruco (cup1'a aruco described in camera)
		self.TAJ = np.matrix([
			[0.0, -1.0, 0.0, -0.01841],
			[1.0, 0.0, 0.0, 0.0],
			[0.0, 0.0, 1.0, -0.037],
			[0.0, 0.0, 0.0, 1.0]
			]) # 4x4 transform from cup 1's aruco to gripping pose (cup1's gripping pose described in cup1's aruco)
		self.TBJ = None # 4x4 transform from base to cup1's gripping pose (cup1's gripping pose described in base frame)
		
		# cup 2 poses
		self.TCD = None # 4x4 transform from camera to cup2's aruco (cup2's aruco described in camera)
		self.TDK = np.matrix([
			[0.0, -1.0, 0.0, -0.01841],
			[1.0, 0.0, 0.0, 0.0],
			[0.0, 0.0, 1.0, -0.037],
			[0.0, 0.0, 0.0, 1.0]
			]) # 4x4 transform from cup2's aruco to gripping pose (cup2's gripping pose described in cup2's aruco)
		self.TBK = None # 4x4 transform from base to cup2's gripping pose (cup2's gripping pose described in base)
		
		# mixing cup poses
		self.TCF = None # 4x4 transform from camera to cupm's aruco (cupm's aruco described in camera)
		self.TFM = np.matrix([
			[0.0, -1.0, 0.0, 0.1875],
			[1.0, 0.0, 0.0, 0.0],
			[0.0, 0.0, 1.0, -0.04533],
			[0.0, 0.0, 0.0, 1.0]
			]) # 4x4 transform from cupm's aruco to dropoff pose (dropoff pose described in cupm's aruco)
		self.TBM = None # 4x4 transform from base to cupm dropoff (dropoff pose described in base)
		
	def update_base(self, transmatrix):
		self.TCG = transmatrix
		self.TGC = np.linalg.inv(transmatrix)
		self.TBC = self.TBG * self.TGC
		# print("updated base")
		# print(self.TCG)

	def update_cup_1(self, transmatrix):
		# print(transmatrix)
		self.TCA = transmatrix
		if self.TBC is not None:
			self.TBJ = self.TBC * self.TCA * self.TAJ
			# print("updated cup_1")
			# print(self.TCA)

	def update_cup_2(self, transmatrix):
		self.TCD = transmatrix
		if self.TBC is not None:
			self.TBK = self.TBC * self.TCD * self.TDK
			# print("updated cup_2")
			# print(self.TCD)


	def update_cup_m(self, transmatrix):
		self.TCF = transmatrix
		if self.TBC is not None:
			self.TBM = self.TBC * self.TCF * self.TFM
			# print("updated cup_m")
			# print(self.TCF)


