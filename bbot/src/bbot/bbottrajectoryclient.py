#! /usr/bin/env python

import roslib
import rospy
import actionlib
import random
import sys
import math
import numpy as np

from control_msgs.msg import (
	FollowJointTrajectoryAction,
	FollowJointTrajectoryGoal,
	JointTolerance,
)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class BBOTTrajectoryClient():
	def __init__(self):
		# start the client and wait for the server
		self.client = actionlib.SimpleActionClient('/bbot/PositionJointInterface_trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
		rospy.loginfo("Client created. Waiting for server.")
		server_up = self.client.wait_for_server()
		if not server_up:
			rospy.logerr("Timed out waiting for Joint Trajectory"
				 " Action Server to connect. Start the action server"
				 " before running example.")
			rospy.signal_shutdown("Timed out waiting for Action Server")
			sys.exit(1)
		rospy.loginfo("Action client started")
		rospy.on_shutdown(self.stop)

		# Create the goal for the action server
		self.goal = FollowJointTrajectoryGoal() # initialize the goal
		self.goal.goal_time_tolerance = rospy.Time.from_sec(1) # set goal time tolerance to 1 second
		# self.joint_names = ["bbot_joint_1", "bbot_joint_2", "bbot_joint_3", "bbot_joint_4", "bbot_joint_5", "bbot_joint_6", "bbot_joint_7"]
		self.joint_names = ["bbot_joint_1", "bbot_joint_2", "bbot_joint_3", "bbot_joint_4", "bbot_joint_5", "bbot_joint_6"]
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
		# self.j7min = math.pi/180.0*(-175.0) # radians
		# self.j7max = math.pi/180.0*(175.0) # radians
	def stop(self):
		self.client.cancel_goal()

	def sendRandomGoal(self):
		bbottraj = JointTrajectory() # instantiate a JointTrajectory object
		bbottraj.joint_names = self.joint_names # set joint names consistent with rostopic /bbot/joint_states
		bbottraj.points = [] 
		
		maxcmd = 3.0
		cmdlen = 2
		for i in range(cmdlen):
			jtp = JointTrajectoryPoint()
			jtp.positions = [random.random() * maxcmd, random.random() * maxcmd, random.random() * maxcmd, random.random() * maxcmd, random.random() * maxcmd, random.random() * maxcmd, random.random() * maxcmd]
			jtp.time_from_start = rospy.Duration(1.0*i)
			bbottraj.points.append(jtp)

		self.goal.trajectory = bbottraj 

		# Tolerances applied to the joints as the trajectory is executed.
		# if violated, error_code = PATH_TOLERANCE_VIOLATED = -4
		self.goal.path_tolerance = [] 

		# joint tolerances allowed for the final position
		# if violated, error_code = GOAL_TOLERANCE_VIOLATED = -5
		self.goal.goal_tolerance = [] 


		# the joints are all currently set to a uniform tolerance
		tol = JointTolerance()
		tol.position = 1.0 # rad for revolute, m for prismatic
		for j in bbottraj.joint_names:
			self.goal.path_tolerance.append(tol)
			self.goal.goal_tolerance.append(tol)
			
		self.client.send_goal(self.goal) # send the fully defined goal to the action server
		self.client.wait_for_result(rospy.Duration.from_sec(cmdlen)) # wait for the goal to complete
		return self.client.get_result()

	def sendPositionTrajectory(self, positions, waitforresult = 10.0):
		# takes in a list of waypoint joint positions, positions, len(positions) = 8,
		# where elements at indeces 0-6 inclusive correspond to joint commands
		# and element at index 7 is the time argument for that waypoint

		bbottraj = JointTrajectory() # instantiate a JointTrajectory object
		bbottraj.joint_names = self.joint_names # set joint names consistent with rostopic /bbot/joint_states
		bbottraj.points = [] 
		for i in range(len(positions)):
			jtp = JointTrajectoryPoint()
			jtp.positions = positions[i][0:-1]
			jtp.time_from_start = rospy.Duration(positions[i][-1])
			bbottraj.points.append(jtp)
		rospy.loginfo(str(bbottraj.points))
		self.goal.trajectory = bbottraj 

		# Tolerances applied to the joints as the trajectory is executed.
		# if violated, error_code = PATH_TOLERANCE_VIOLATED = -4
		self.goal.path_tolerance = [] 

		# joint tolerances allowed for the final position
		# if violated, error_code = GOAL_TOLERANCE_VIOLATED = -5
		self.goal.goal_tolerance = [] 

		# goal time tolerance
		self.goal.goal_time_tolerance = rospy.Duration.from_sec(2)


		# the joints are all currently set to a uniform tolerance
		tol = JointTolerance()
		tol.position = 0.02 # rad for revolute, m for prismatic
		for j in bbottraj.joint_names:
			self.goal.path_tolerance.append(tol)
			self.goal.goal_tolerance.append(tol)
			
		self.client.send_goal(self.goal) # send the fully defined goal to the action server
		self.client.wait_for_result(rospy.Duration.from_sec(waitforresult)) # wait for the goal to complete
		return self.client.get_result()
	def sendVelocityTrajectory(self, positions, vels, waitforresult = 10.0):
		# takes in a list of waypoint joint positions, positions, len(positions) = 8,
		# where elements at indeces 0-6 inclusive correspond to position joint commands
		# and element at index 7 in positions is the time argument for that waypoint
		# and list of waypoint joint velocities, vels, len(vels) = 7
		# where elements at indeces 0-6 inclusive correspond to velocity joint commands
		
		bbottraj = JointTrajectory() # instantiate a JointTrajectory object
		bbottraj.joint_names = self.joint_names # set joint names consistent with rostopic /bbot/joint_states
		bbottraj.points = [] 

		for i in range(len(vels)):
			jtp = JointTrajectoryPoint()
			# rospy.loginfo(str(vels[i][0:-1]))
			# rospy.loginfo(str(len(vels[i][0:-1])))
			jtp.positions = positions[i][0:-1]
			jtp.velocities = vels[i]
			jtp.time_from_start = rospy.Duration(positions[i][-1])
			bbottraj.points.append(jtp)
		rospy.loginfo(str(bbottraj.points))
		self.goal.trajectory = bbottraj 

		# Tolerances applied to the joints as the trajectory is executed.
		# if violated, error_code = PATH_TOLERANCE_VIOLATED = -4
		self.goal.path_tolerance = [] 

		# joint tolerances allowed for the final position
		# if violated, error_code = GOAL_TOLERANCE_VIOLATED = -5
		self.goal.goal_tolerance = [] 

		# goal time tolerance
		self.goal.goal_time_tolerance = rospy.Duration.from_sec(2)


		# the joints are all currently set to a uniform tolerance
		tol = JointTolerance()
		tol.position = -1 # rad for revolute, m for prismatic
		for j in bbottraj.joint_names:
			self.goal.path_tolerance.append(tol)
			self.goal.goal_tolerance.append(tol)

			
		self.client.send_goal(self.goal) # send the fully defined goal to the action server
		self.client.wait_for_result(rospy.Duration(waitforresult)) # wait for the goal to complete
		return self.client.get_result()

