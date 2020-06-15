#!/usr/bin/env python

from dynamixel_sdk import *  
import roslib
import rospy
import actionlib

from numpy import interp
import math

# from bbot.msg import bbotAction, bbotFeedback, bbotResult# imports all generated msgs for goals, feedback, result, etc

from control_msgs.msg import (
	FollowJointTrajectoryAction,
	FollowJointTrajectoryGoal,
	FollowJointTrajectoryActionFeedback,
	FollowJointTrajectoryActionResult,
	JointTolerance,
)

from moveit_msgs.msg import RobotState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def clamp(value, lower, upper):
	return lower if value < lower else upper if value > upper else value

class BBotDynamixel(object):
	def __init__(self):
		# self.feedback = bbotFeedback() # self.feedback.currentpos is a RobotState msg
		self.feedback = FollowJointTrajectoryActionFeedback()
		# self.result = bbotResult() # self.result.complete is a bool msg
		self.result = FollowJointTrajectoryActionResult()
		# self.server = actionlib.SimpleActionServer('bbotaxn', bbotAction, self.execute, False)
		self.server = actionlib.SimpleActionServer('/bbot/PositionJointInterface_trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction, self.execute, False)
		self.server.start()
		self.actionlock = False
		rospy.loginfo('Action server started')
		self.zero = 2047
		self.m1min = 100	# motor min
		self.m1max = 4000	# motor max
		self.m2min = 100	# motor min
		self.m2max = 4000	# motor max
		self.m3min = 100	# motor min
		self.m3max = 4000	# motor max
		self.m4min = 100	# motor min
		self.m4max = 4000	# motor max
		self.m5min = 100	# motor min
		self.m5max = 4000	# motor max
		self.m6min = 100	# motor min
		self.m6max = 4000	# motor max
		self.m7min = 100	# motor min
		self.m7max = 4000	# motor max
		self.j1min = math.pi/180.0*(-170.0) # radians
		self.j1max = math.pi/180.0*(170.0) # radians
		self.j2min = math.pi/180.0*(-120.0) # radians
		self.j2max = math.pi/180.0*(120.0) # radians
		self.j3min = math.pi/180.0*(-170.0) # radians
		self.j3max = math.pi/180.0*(170.0) # radians
		self.j4min = math.pi/180.0*(-120.0) # radians
		self.j4max = math.pi/180.0*(120.0) # radians
		self.j5min = math.pi/180.0*(-170.0) # radians
		self.j5max = math.pi/180.0*(170.0) # radians
		self.j6min = math.pi/180.0*(-120.0) # radians
		self.j6max = math.pi/180.0*(120.0) # radians
		self.j7min = math.pi/180.0*(-175.0) # radians
		self.j7max = math.pi/180.0*(175.0) # radians
		self.minmotorcmds = [self.m1min, self.m2min, self.m3min, self.m4min, self.m5min, self.m6min, self.m7min]
		self.maxmotorcmds = [self.m1max, self.m2max, self.m3max, self.m4max, self.m5max, self.m6max, self.m7max]
		self.velocitylimits = [] # in rpm
		# self.velocityprofile = [30, 30, 30, 30, 30, 30, 60] # units: rev/min. These vals correspond to a velocity-based profile
		self.velocityprofile = [30] * 7 # [1000, 1000, 1000, 1000, 1000, 1000, 1000] # units: ms. These vals correspond to time-based profiles
		self.accelprofile = [1] * 7
		self.drive_modes = [4, 4, 4, 4, 4, 4, 4] # 1 = velocity-based profilew/reverse enabled, 4 = time-based with reverse disabled 5 = time-based profile w/reverse enabled
		self.pid_gains = [
		[850, 0, 0],
		[2000, 500, 0],
		[850, 50, 0],
		[850, 0, 0],
		[850, 0, 0],
		[850, 0, 0]
		]

		self.statepub = rospy.Publisher('/bbot/jointStates', RobotState, queue_size = 1)		# initialize state publisher
		# self.rospy.init_node('bbotdynamixel', anonymous = True)
		rospy.on_shutdown(self.shutdownMotors)

		# Control table address
		self.ADDR_PRO_TORQUE_ENABLE      = 64               # addresses defined in Protocol 2.0 documentation http://emanual.robotis.com/docs/en/dxl/mx/mx-28-2/
		self.ADDR_PRO_GOAL_POSITION      = 116
		self.ADDR_PRO_PRESENT_POSITION   = 132
		self.ADDR_DRIVE_MODE 			= 10
		self.ADDR_OPERATING_MODE 		= 11 
		self.ADDR_GOAL_VELOCITY			= 104
		self.ADDR_VELOCITY_LIMIT		= 44
		self.ADDR_PROFILE_VELOCITY 		= 112
		self.ADDR_PROFILE_ACCELERATION 	= 108
		self.ADDR_P_GAIN = 84
		self.ADDR_I_GAIN = 82
		self.ADDR_D_GAIN = 80
		self.ADDR_POS_TRAJ = 140


		# Data Byte Length
		self.LEN_PRO_GOAL_POSITION       = 4
		self.LEN_PRO_PRESENT_POSITION    = 4
		self.LEN_PROFILE_VELOCITY		= 4
		self.LEN_PROFILE_ACCELERATION		= 4
		self.LEN_POS_TRAJ		= 4

		# Protocol version
		self.PROTOCOL_VERSION            = 2.0				# See which protocol version is used in the Dynamixel

		self.idlist = [1, 2, 3, 4, 5, 6] #, 7]					# list of motor ids. must represent current motors connected for script to work

		self.BAUDRATE                    = 57600			# Dynamixel default baudrate : 57600
		self.DEVICENAME                  = '/dev/ttyUSB0'	# Check which port is being used on your controller
															# ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

		self.TORQUE_ENABLE               = 1				# Value for enabling the torque
		self.TORQUE_DISABLE              = 0				# Value for disabling the torque
		self.DXL_MINIMUM_POSITION_VALUE  = 100				# Dynamixel will rotate between this value
		self.DXL_MAXIMUM_POSITION_VALUE  = 4000				# and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
		self.DXL_MOVING_STATUS_THRESHOLD = 20				# Dynamixel moving status threshold
		self.EXT_POSITION_CONTROL_MODE   = 4                 # Value for extended position control mode (operating mode)


		# Initialize PortHandler instance
		# Set the port path
		# Get methods and members of PortHandlerLinux or PortHandlerWindows
		self.portHandler = PortHandler(self.DEVICENAME)

		# Initialize PacketHandler instance
		# Set the protocol version
		# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
		self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

		# Initialize GroupSyncWrite instance
		self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_PRO_GOAL_POSITION, self.LEN_PRO_GOAL_POSITION)
		self.groupSyncWriteProfileVel = GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_PROFILE_VELOCITY, self.LEN_PROFILE_VELOCITY)
		self.groupSyncWriteAccelVel = GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_PROFILE_ACCELERATION, self.LEN_PROFILE_ACCELERATION)
		# Initialize GroupSyncRead instace for Present Position
		self.groupSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, self.ADDR_PRO_PRESENT_POSITION, self.LEN_PRO_PRESENT_POSITION)
		self.groupSyncReadTraj = GroupSyncRead(self.portHandler, self.packetHandler, self.ADDR_POS_TRAJ, self.LEN_POS_TRAJ)

		# Open port
		if self.portHandler.openPort():
			rospy.loginfo("Succeeded to open the port")
		else:
			rospy.loginfo("Failed to open the port")
			quit()

		# Set port baudrate
		if self.portHandler.setBaudRate(self.BAUDRATE):
			rospy.loginfo("Succeeded to change the baudrate")
		else:
			rospy.loginfo("Failed to change the baudrate")
			quit()
		# self.setExtendedPositionControl(self.idlist[0])	# set motor 1 to extended position control

		# set the Drive Mode to time-based or velocity-based profiles
		self.setDriveMode()
		rospy.loginfo("Drive mode: " + str(self.readDriveMode()))

		# run the setup
		self.setup()

		# set PID gains
		rospy.loginfo("PID gains before setting: " + str(self.readGains()))
		self.setGains(self.pid_gains)
		rospy.loginfo("PID gains after setting: " + str(self.readGains()))

		# set vel limits and read back limits and profile accel/vel
		self.velocitylimits = self.readVelocityLimits() # motors come with max velocity 230 on a scale [0, 1023]
		rospy.loginfo("Velocity Limits: " + str(self.velocitylimits))
		rospy.loginfo("Goal Velocities: " + str(self.readGoalVelocity()))
		rospy.loginfo("Profile Acclerations: " + str(self.readProfileAcceleration()))
		rospy.loginfo("Profile Velocities: " + str(self.readProfileVelocity()))
		# self.setProfileVelocity(self.velocityprofile)
		# self.setProfileAcceleration(self.accelprofile)
		# rospy.loginfo("Profile Velocities: " + str(self.readProfileVelocity()))
		# rospy.loginfo("Profile Acclerations: " + str(self.readProfileAcceleration()))


		self.state = RobotState()	# initialize the state variable as RobotState msg type
		self.updateStateFeedback()	# reads the present motor positions and publishes them
		
		# state pub when action server is idle (not fielding trajs)		
		rospy.Timer(rospy.Duration(0.5), self.timercallback)
		rospy.loginfo("Robot successfully created")

	def setGains(self, gains):
		for motor_id in range(len(self.idlist)):
			dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.idlist[motor_id], self.ADDR_P_GAIN, gains[motor_id][0])
			if dxl_comm_result != COMM_SUCCESS:
				print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
			elif dxl_error != 0:
				print("%s" % self.packetHandler.getRxPacketError(dxl_error))

			dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.idlist[motor_id], self.ADDR_I_GAIN, gains[motor_id][1])
			if dxl_comm_result != COMM_SUCCESS:
				print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
			elif dxl_error != 0:
				print("%s" % self.packetHandler.getRxPacketError(dxl_error))

			dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.idlist[motor_id], self.ADDR_D_GAIN, gains[motor_id][2])
			if dxl_comm_result != COMM_SUCCESS:
				print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
			elif dxl_error != 0:
				print("%s" % self.packetHandler.getRxPacketError(dxl_error))
		print("Gains successfully set to \n" + str(gains))



	def readGains(self):
		gains = []
		for motor_id in range(len(self.idlist)):
			m_gains = []
			dxl_p_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.idlist[motor_id], self.ADDR_P_GAIN)
			if dxl_comm_result != COMM_SUCCESS:
				print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
			elif dxl_error != 0:
				print("%s" % self.packetHandler.getRxPacketError(dxl_error))
			else:
				m_gains.append(dxl_p_gain)

			dxl_i_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.idlist[motor_id], self.ADDR_I_GAIN)
			if dxl_comm_result != COMM_SUCCESS:
				print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
			elif dxl_error != 0:
				print("%s" % self.packetHandler.getRxPacketError(dxl_error))
			else:
				m_gains.append(dxl_i_gain)

			dxl_d_gain, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.idlist[motor_id], self.ADDR_D_GAIN)
			if dxl_comm_result != COMM_SUCCESS:
				print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
			elif dxl_error != 0:
				print("%s" % self.packetHandler.getRxPacketError(dxl_error))
			else:
				m_gains.append(dxl_d_gain)

			gains.append(m_gains)

		return gains

	def readDriveMode(self):
		drive_modes = []
		for motor in self.idlist:
			dxl_drive_mode, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, motor, self.ADDR_DRIVE_MODE)
			if dxl_comm_result != COMM_SUCCESS:
				print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
			elif dxl_error != 0:
				print("%s" % self.packetHandler.getRxPacketError(dxl_error))
			drive_modes.append(dxl_drive_mode)
		return drive_modes

	def setDriveMode(self):
		rospy.loginfo("Setting Drive Mode")
		for i in range(len(self.idlist)):
			motorid = self.idlist[i]
			dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, motorid, self.ADDR_DRIVE_MODE, self.drive_modes[i])
			if dxl_comm_result != COMM_SUCCESS:
				rospy.loginfo("dxl_comm_result != COMM_SUCCESS")
				rospy.loginfo("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
			elif dxl_error != 0:
				rospy.loginfo("%s" % self.packetHandler.getRxPacketError(dxl_error))
			else:
				rospy.loginfo("Drive mode set to " + str(self.drive_modes[i]) + " for motor "+ str(self.idlist[i]))


	def readVelocityLimits(self):
		# Read maximum velocity for each motor in self.idlist
		maxvels = []
		for motor in self.idlist:
			dxl_max_vel, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, motor, self.ADDR_VELOCITY_LIMIT)
			if dxl_comm_result != COMM_SUCCESS:
				print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
			elif dxl_error != 0:
				print("%s" % self.packetHandler.getRxPacketError(dxl_error))
			maxvels.append(dxl_max_vel)
		return maxvels
	def readGoalVelocity(self):
		goalvels = []
		for motor in self.idlist:
			dxl_goal_vel, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, motor, self.ADDR_GOAL_VELOCITY)
			if dxl_comm_result != COMM_SUCCESS:
				print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
			elif dxl_error != 0:
				print("%s" % self.packetHandler.getRxPacketError(dxl_error))
			goalvels.append(dxl_goal_vel)
		return goalvels

	def readProfileVelocity(self):
		profilevels = []
		for motor in self.idlist:
			dxl_profile_vel, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, motor, self.ADDR_PROFILE_VELOCITY)
			if dxl_comm_result != COMM_SUCCESS:
				print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
			elif dxl_error != 0:
				print("%s" % self.packetHandler.getRxPacketError(dxl_error))
			profilevels.append(dxl_profile_vel)
		return profilevels

	def readProfileAcceleration(self):
		profileaccels = []
		for motor in self.idlist:
			dxl_profile_accel, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, motor, self.ADDR_PROFILE_ACCELERATION)
			if dxl_comm_result != COMM_SUCCESS:
				print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
			elif dxl_error != 0:
				print("%s" % self.packetHandler.getRxPacketError(dxl_error))
			profileaccels.append(dxl_profile_accel)
		return profileaccels

	def setProfileVelocity(self, profilevel):
		# DO NOT USE THIS METHOD WHEN SET PROFILE VELOCITY IS SET VIA SYNC WRITE METHOD, AS IT IS WHEN DRIVE_MODE ==1
		rospy.loginfo("Setting profile velocity")
		for index in range(len(self.idlist)):
			dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.idlist[index], self.ADDR_PROFILE_VELOCITY, profilevel[index])
			if dxl_comm_result != COMM_SUCCESS:
				rospy.loginfo("dxl_comm_result != COMM_SUCCESS. Failed to set goal velocity :(")
				rospy.loginfo("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
			elif dxl_error != 0:
				rospy.loginfo("dxl_error != 0")
				rospy.loginfo("%s" % self.packetHandler.getRxPacketError(dxl_error))
			else:
				rospy.loginfo("Profile velocity set for motor ID " + str(self.idlist[index]) + " at profile_velocity = " + str(profilevel[index]) + " in DRIVE MODE " + str(self.drive_modes[index]))

	def setProfileAcceleration(self, profileaccel):
		rospy.loginfo("Setting profile acceleration")
		for index in range(len(self.idlist)):
			dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.idlist[index], self.ADDR_PROFILE_ACCELERATION, profileaccel[index])
			if dxl_comm_result != COMM_SUCCESS:
				rospy.loginfo("dxl_comm_result != COMM_SUCCESS. Failed to set goal accel :(")
				rospy.loginfo("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
			elif dxl_error != 0:
				rospy.loginfo("dxl_error != 0")
				rospy.loginfo("%s" % self.packetHandler.getRxPacketError(dxl_error))
			else:
				rospy.loginfo("Profile accel set for motor ID " + str(self.idlist[index]) + " at profile_accel = " + str(profileaccel))

	def setGoalVelocity(self, goalvel):
		rospy.loginfo("Setting goal velocity")
		for motorid in self.idlist:
			dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, motorid, self.ADDR_GOAL_VELOCITY, goalvel)
			if dxl_comm_result != COMM_SUCCESS:
				rospy.loginfo("dxl_comm_result != COMM_SUCCESS. Failed to set goal velocity :(")
				rospy.loginfo("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
			elif dxl_error != 0:
				rospy.loginfo("dxl_error != 0")
				rospy.loginfo("%s" % self.packetHandler.getRxPacketError(dxl_error))
			else:
				rospy.loginfo("Goal velocity set for motor ID " + str(motorid) + " at goal_velocity = " + str(goalvel))

	def setExtendedPositionControl(self, motorid):
		# Set operating mode to extended position control mode
		rospy.loginfo("Setting extended pos control")
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, motorid, self.ADDR_OPERATING_MODE, self.EXT_POSITION_CONTROL_MODE)
		if dxl_comm_result != COMM_SUCCESS:
			rospy.loginfo("dxl_comm_result != COMM_SUCCESS")
			rospy.loginfo("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			rospy.loginfo("%s" % self.packetHandler.getRxPacketError(dxl_error))
		else:
			rospy.loginfo("Operating mode changed to extended position control mode.")
	def enabletorque(self, motorids):
		# enables torque only for motors specified in motorids
		for motorid in motorids:
			dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, motorid, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_ENABLE)
			if dxl_comm_result != COMM_SUCCESS:
				rospy.loginfo("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
			elif dxl_error != 0:
				rospy.loginfo("%s" % self.packetHandler.getRxPacketError(dxl_error))
			else:
				rospy.loginfo("Dynamixel#%d has been successfully connected" % motorid)

	def disabletorque(self, motorids):
		for motorid in motorids:
			# iterate over motorids and disable torque for each motor
			dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, motorid, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_DISABLE)
			if dxl_comm_result != COMM_SUCCESS:
				rospy.loginfo("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
			elif dxl_error != 0:
				rospy.loginfo("%s" % self.packetHandler.getRxPacketError(dxl_error))

	def readAddParamList(self, motorids):
		# Add parameter storage for each DXL_ID present position value
		for motorid in motorids:
			dxl_addparam_result = self.groupSyncRead.addParam(motorid)
			if dxl_addparam_result != True:
				rospy.loginfo("[ID:%03d] groupSyncRead addparam failed" % motorid)
				quit()

	def readAddParamListTraj(self, motorids):
		# Add parameter storage for each DXL_ID trajectory value
		for motorid in motorids:
			dxl_addparam_result = self.groupSyncReadTraj.addParam(motorid)
			if dxl_addparam_result != True:
				rospy.loginfo("[ID:%03d] groupSyncReadTraj addparam failed" % motorid)
				quit()

	def writeAddParamListAccelVel(self, motorids, param_goal_accel_list):
		for index in range(len(motorids)):
			dxl_addparam_result = self.groupSyncWriteAccelVel.addParam(motorids[index], param_goal_accel_list[index])
			if dxl_addparam_result != True:
				rospy.loginfo("[ID:%03d] groupSyncWriteAccelVel addparam failed" % motorids[index])
				quit()
			if dxl_addparam_result == True:
				rospy.loginfo("[ID:" + str(motorids[index]) + "] groupSyncWriteAccelVel add param successful, wrote value " + str(param_goal_accel_list[index]))


	def writeAddParamListProfileVel(self, motorids, param_goal_vel_list):
		for index in range(len(motorids)):
			dxl_addparam_result = self.groupSyncWriteProfileVel.addParam(motorids[index], param_goal_vel_list[index])
			if dxl_addparam_result != True:
				rospy.loginfo("[ID:%03d] groupSyncWriteProfileVel addparam failed" % motorids[index])
				quit()
			if dxl_addparam_result == True:
				rospy.loginfo("[ID:" + str(motorids[index]) + "] groupSyncWriteProfileVel add param successful, wrote value " + str(param_goal_vel_list[index]))

	def writeAddParamList(self, motorids, param_goal_pos_list):
		# Add Dynamixel#1 goal position value to the Syncwrite parameter storage
		# rospy.loginfo(goal_pos)
		for index in range(len(motorids)):
			dxl_addparam_result = self.groupSyncWrite.addParam(motorids[index], param_goal_pos_list[index])
			if dxl_addparam_result != True:
				rospy.loginfo("[ID:%03d] groupSyncWrite addparam failed" % motorids[index])
				quit()

	def readIsAvailable(self, motorids):
		# Check if groupsyncread data of Dynamixels is available
		for motorid in motorids:
			dxl_getdata_result = self.groupSyncRead.isAvailable(motorid, self.ADDR_PRO_PRESENT_POSITION, self.LEN_PRO_PRESENT_POSITION)
			if dxl_getdata_result != True:
				rospy.loginfo("[ID:%03d] groupSyncRead getdata failed" % motorid)
				quit()

	def readIsAvailableTraj(self, motorids):
		# Check if groupsyncread data of Dynamixels is available
		for motorid in motorids:
			dxl_getdata_result = self.groupSyncReadTraj.isAvailable(motorid, self.ADDR_POS_TRAJ, self.LEN_POS_TRAJ)
			if dxl_getdata_result != True:
				rospy.loginfo("[ID:%03d] groupSyncReadTraj getdata failed" % motorid)
				quit()

	def updateStateFeedback(self):
		# Syncread present position
		dxl_comm_result = self.groupSyncRead.txRxPacket()
		# rospy.loginfo("syncread present pos succeeded")
		# rospy.loginfo("dxl_comm_result: " + str(dxl_comm_result))
		while dxl_comm_result != COMM_SUCCESS:
			rospy.loginfo("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
			dxl_comm_result = self.groupSyncRead.txRxPacket()
			# quit()
		dxl_comm_result_traj = self.groupSyncReadTraj.txRxPacket()
		# rospy.loginfo("syncread present pos succeeded")
		# rospy.loginfo("dxl_comm_result: " + str(dxl_comm_result))
		while dxl_comm_result_traj != COMM_SUCCESS:
			rospy.loginfo("%s" % self.packetHandler.getTxRxResult(dxl_comm_result_traj))
			dxl_comm_result_traj = self.groupSyncReadTraj.txRxPacket()
			# quit()

		# Check if groupsyncread data of Dynamixels is available
		self.readIsAvailable(self.idlist)
		self.readIsAvailableTraj(self.idlist)
		pos_list = []
		traj_list = []
		for motorid in self.idlist:
			motorreading = self.groupSyncRead.getData(motorid, self.ADDR_PRO_PRESENT_POSITION, self.LEN_PRO_PRESENT_POSITION)
			pos_list.append(float(motorreading)/4096.0*360.0-180.0)
			trajreading = self.groupSyncReadTraj.getData(motorid, self.ADDR_POS_TRAJ, self.LEN_POS_TRAJ)
			traj_list.append(float(trajreading)/4096.0*360.0-180.0)
			# print("trajreading: " + str(float(trajreading)/4096.0*360.0-180.0))

		self.state.joint_state.position = pos_list
		self.state.joint_state.header.stamp = rospy.Time.now()
		# self.state.joint_state.name = self.idlist
		self.statepub.publish(self.state)
		rospy.loginfo("motor states updated")
		# rospy.loginfo(pos_list)
		return pos_list, traj_list;

	def listIntToByteArr(self, goal_pos_list):
		param_goal_positions = []
		for pos in goal_pos_list:
			param_goal_positions.append([DXL_LOBYTE(DXL_LOWORD(pos)), DXL_HIBYTE(DXL_LOWORD(pos)), DXL_LOBYTE(DXL_HIWORD(pos)), DXL_HIBYTE(DXL_HIWORD(pos))])
		return param_goal_positions

	def setGoalPos(self, goal_pos_list, profile_vel, accel_time):
		# master function to set and write the goal pos array and profile velocity to each motor
		# goal_pos_list comes in as an array of motor values in range 0-4095
		# profile velocity comes in as an int time arg in ms
		# len(self.idlist) needs to match len(goal_pos_list)
		profile_vel_list = [profile_vel] * len(self.idlist) 
		rospy.loginfo("profile_vel_list: " + str(profile_vel_list))
		accel_time_list = [accel_time] * len(self.idlist)
		cmdlist = []
		for i in range(len(self.idlist)):
			cmdlist.append(int(clamp(goal_pos_list[i], self.minmotorcmds[i], self.maxmotorcmds[i])))
		
		# self.setProfileVelocity(profile_vel_list)
		# self.setProfileAcceleration(accel_time_list)

		
		# Allocate profile acceleration values into array of byte arrays
		param_profile_accels = self.listIntToByteArr(accel_time_list)

		# Add Dynamixels' profile acceleration to the Syncwrite parameter storage
		self.writeAddParamListAccelVel(self.idlist, param_profile_accels)

		# Syncwrite profile accelerations to motors
		dxl_comm_result = self.groupSyncWriteAccelVel.txPacket()
		if dxl_comm_result != COMM_SUCCESS:
			rospy.loginfo("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

		# Clear syncwrite parameter storage
		self.groupSyncWriteAccelVel.clearParam()

		rospy.loginfo("Profile Acclerations: " + str(self.readProfileAcceleration()))



		# Allocate profile velocity values into array of byte arrays
		param_profile_vels = self.listIntToByteArr(profile_vel_list)

		# Add Dynamixels' profile velocities to the Syncwrite parameter storage
		self.writeAddParamListProfileVel(self.idlist, param_profile_vels)

		# Syncwrite profile velocities to motors
		dxl_comm_result = self.groupSyncWriteProfileVel.txPacket()
		if dxl_comm_result != COMM_SUCCESS:
			rospy.loginfo("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

		# Clear syncwrite parameter storage
		self.groupSyncWriteProfileVel.clearParam()

		rospy.loginfo("Profile Velocities: " + str(self.readProfileVelocity()))



		# Allocate goal position values into array of byte arrays
		param_goal_positions = self.listIntToByteArr(cmdlist)

		# Add Dynamixels' goal position value to the Syncwrite parameter storage
		self.writeAddParamList(self.idlist, param_goal_positions)

		# Syncwrite goal position to motors
		dxl_comm_result = self.groupSyncWrite.txPacket()
		if dxl_comm_result != COMM_SUCCESS:
			rospy.loginfo("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

		# Clear syncwrite parameter storage
		self.groupSyncWrite.clearParam()

	def compareGoalCurrPos(self, goal_pos_arr, curr_pos_arr):
		# compares each matched index of goal and current pos arrays
		# returns True if the current pos is within the threshold of the goal pos
		# returns False if the current pos is not within the threshold
		for index in range(len(curr_pos_arr)):
			if (abs(goal_pos_arr[index] - curr_pos_arr[index]) > self.DXL_MOVING_STATUS_THRESHOLD):
				return False
		return True

	def setup(self):
		# enable torque for each motor
		self.enabletorque(self.idlist)
		# Add parameter storage for each DXL_ID present position value
		self.readAddParamList(self.idlist)
		self.readAddParamListTraj(self.idlist)

	def shutdownMotors(self):
		# Clear syncread parameter storage
		self.groupSyncRead.clearParam()
		self.groupSyncReadTraj.clearParam()
		# disable torque for each motor
		self.disabletorque(self.idlist)
		# Close port
		self.portHandler.closePort()

	def angles_to_bits(self, q):
		# takes in a q vec of joint angles in units of radians
		# maps the values to motor command space
		# -180 -> 0
		# 0 radians -> 2047
		# +180 -> 4095
		q_motor = []
		for i in q:
			q_motor.append(int(interp(i, [-math.pi, math.pi], [0, 4095])))
		return q_motor

	def execute(self, goal):
		# goal is a RobotTrajectory
		# execute the traj
		self.actionlock = True
		success = True
		prev_time = 0
		prev_duration = 0.0
		for waypoint in goal.trajectory.points:
			# iterates over the JointTrajectoryPoint objects
			# for now just rospy.loginfo the waypoint, then sleep the duration attached to the waypoint
			if self.server.is_preempt_requested():
				# if preempted, cancel the trajectory
				rospy.loginfo('Action preempted!!!')
				self.server.set_preempted()
				success = False
				break
			# act on the waypoint
			positions_motor_cmd = self.angles_to_bits(waypoint.positions)
			rospy.loginfo(positions_motor_cmd)

			# update time info
			duration = waypoint.time_from_start.to_sec()
			time_allotted = int(duration*1000) - int(prev_duration*1000) # units: ms, type: int
			accel_time = time_allotted/4
			now_split = rospy.get_rostime()
			now = float(now_split.secs) + float(now_split.nsecs)/10.0**9
			start_time = now

			print("duration: " + str(duration))
			print("now: " + str(now))
			print("start_time: " + str(start_time))
			# set the motor positions based on the joint commands and time arg
			self.setGoalPos(positions_motor_cmd, time_allotted, accel_time)
			rospy.loginfo("time alloted = " + str(time_allotted))

			# update state feedback and publish current pos
			# self.feedback.currentpos.joint_state.position = self.updateStateFeedback()
			

			while (now - start_time) < (duration - prev_duration):
				# print("updating")
				# print(now)
				# print(start_time)
				# print(duration)
				# print(prev_duration)
				self.feedback.feedback.header.stamp = rospy.get_rostime()
				self.feedback.feedback.actual.positions, self.feedback.feedback.desired.positions = self.updateStateFeedback()
				# rospy.loginfo(str(self.feedback.currentpos.joint_state.position))
				self.server.publish_feedback(self.feedback.feedback)
				rospy.sleep(0.01)
				now_split = rospy.get_rostime()
				now = float(now_split.secs) + float(now_split.nsecs)/10.0**9
			prev_duration = duration

			# rospy.sleep(waypoint.time_from_start) # sleeps for duration attached to the waypoint
		if success:
			# self.result.complete = True
			self.result.result.error_code = self.result.result.SUCCESSFUL
			self.result.result.error_string = "Success"
			self.actionlock = False
			rospy.loginfo('Succeeded. Completed trajectory')
			print(self.result.result.error_code)
			self.server.set_succeeded(self.result.result)
		self.actionlock = False

	def run(self):
		r = rospy.Rate(10) # Hz
		while not rospy.is_shutdown():
			self.updateStateFeedback()
			r.sleep()
	def timercallback(self, event=None):
		if self.actionlock == False:
			self.updateStateFeedback()
			now_split = rospy.get_rostime()
			now = float(now_split.secs) + float(now_split.nsecs)/10.0**9
			rospy.loginfo("now: " + str(now))


if __name__=='__main__':
	rospy.init_node('bbot_server')
	bbot = BBotDynamixel()
	rospy.spin()
