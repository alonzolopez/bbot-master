"""
The MixingTaskStateMachine Class holds all the object poses that act as a model of the task space for use in the mixing operation
"""
class MixingTaskStateMachine():
	def __init__(self):
		# robot poses
		self.robot_aruco_in_base = None # 4x4 transform describing bbot's aruco in bbot's base frame
		self.robot_aruco_in_camera = None # 4x4 transform describing bbot's aruco pose in camera frame
		
		# camera pose
		self.camera_pose_in_base = None # use the aruco pose in base frame and aruco pose in camera frame to get camera pose in bbot base frame
		
		# cup 1 poses
		self.cup1_aruco_in_camera = None # pose of cup 1 aruco in the camera's frame
		self.cup1_eetocup1 = None # closed gripping pose in cup1's aruco frame

		# cup 2 poses
		self.cup2_aruco_in_camera = None # pose of cup 2 aruco in the camera's frame
		self.cup2_eetocup2 = None # closed gripping pose in cup2's aruco frame

		# mixing cup poses
		self.cupm_arucopose = None # pose of the mixing cup's aruco in the camera's frame
		self.cupm_droppose = None # pose of ee before dropping