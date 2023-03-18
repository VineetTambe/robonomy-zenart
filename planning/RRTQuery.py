import sys
import os

fpath = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, fpath + '/..')

from random import sample, seed
from re import A
import time
import pickle
import numpy as np

# import vrep_interface as vpi
import RobotUtil as rt
import Franka
import time

# Mujoco Imports
import mujoco as mj
from mujoco import viewer

# from controls import panda_controller


class PathPlanner():

	def __init__(self) -> None:
		self.xml_filepath = fpath + "/../assets/franka_emika_panda/panda_with_hand_torque.xml"
		seed(10)

		np.random.seed(0)
		self.deg_to_rad = np.pi/180.

		self.mybot = Franka.FrankArm()
		self.joint_counter = 0
		self.plan = []
		self.interpolated_plan = []
		self.plan_length = len(self.plan)
		self.inc = 1

		self.pointsObs = []
		self.axesObs = []

		# Initialize some data containers for the RRT planner
		self.rrtVertices = []
		self.rrtEdges = []

		self.thresh = 0.1
		self.found_solution = False
		self.solution_interpolated = False


	def set_inital_joint_config(self, joint_angles=None):
		if joint_angles==None:
			self.qInit = [-np.pi/2, -np.pi/2, np.pi/2, -np.pi/2, 0, np.pi - np.pi/6, 0]
		else:
			self.qInit = joint_angles
		self.rrtVertices.append(self.qInit)
		self.rrtEdges.append(0)

	def set_goal_joint_config(self, joint_angles=None):
		if joint_angles==None:
			self.qGoal = [np.pi/2, -np.pi/2, -np.pi/2, -np.pi/2, 0, np.pi - np.pi/6, 0]
		else:
			self.qGoal = joint_angles
	
	def add_obstace(self, pose, dim):
		# envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H([0,0.,0.0],[0.1,0,1.0]),[1.0,1.4,0.1])
		# pointsObs.append(envpoints), axesObs.append(envaxes)
		envpoints, envaxes = rt.BlockDesc2Points(rt.rpyxyz2H(pose[0], pose[1]), dim)
		self.pointsObs.append(envpoints)
		self.axesObs.append(envaxes)

	def find_nearest(self, prev_points, new_point):
		D = np.array([np.linalg.norm(np.array(point)-np.array(new_point)) for point in prev_points])
		return D.argmin()

	
	def naive_interpolation(self):

		angle_resolution = 0.01

		self.interpolated_plan = np.empty((1,7))
		np_plan = np.array(self.plan)
		self.interpolated_plan[0] = np_plan[0]
		
		for i in range(np_plan.shape[0]-1):
			max_joint_val = np.max(np_plan[i+1] - np_plan[i])
			number_of_steps = int(np.ceil(max_joint_val/angle_resolution))
			if number_of_steps==0:
				number_of_steps=1
			inc = (np_plan[i+1] - np_plan[i])/number_of_steps

			for j in range(1,number_of_steps+1):
				step = np_plan[i] + j*inc
				self.interpolated_plan = np.append(self.interpolated_plan, step.reshape(1,7), axis=0)

		self.solution_interpolated = True
		print("Plan has been interpolated successfully!")



	def RRTQuery(self, read_saved_plan=False):

		qGoal = np.array(self.qGoal)
		while len(self.rrtVertices)<3000 and not self.found_solution and not read_saved_plan:

			# Fill in the algorithm here
			qRand = np.array(self.mybot.SampleRobotConfig())

			if np.random.uniform(0,1) < 0.05:
				qRand = qGoal

			idNear = self.find_nearest(self.rrtVertices, qRand)
			qNear = self.rrtVertices[idNear]

			# if np.linalg.norm(qRand - qNear) > thresh:
			# 	qConnect = np.array(qNear) + (thresh * (np.array(qRand) - np.array(qNear)) / np.linalg.norm(qRand - qNear))
			# else:
			# 	qConnect = qRand

			while np.linalg.norm(qRand - qNear) > self.thresh:
				qConnect = np.array(qNear) + (self.thresh * (np.array(qRand) - np.array(qNear)) / np.linalg.norm(qRand - qNear))

				if not self.mybot.DetectCollisionEdge(qConnect, qNear, self.pointsObs, self.axesObs):
					self.rrtVertices.append(qConnect)
					self.rrtEdges.append(idNear)
					qNear = qConnect
				else:
					break
			
			qConnect = qRand
			if not self.mybot.DetectCollisionEdge(qConnect, qNear, self.pointsObs, self.axesObs):
				self.rrtVertices.append(qConnect)
				self.rrtEdges.append(idNear)
			
			idNear = self.find_nearest(self.rrtVertices, qGoal)
			if np.linalg.norm(qGoal - self.rrtVertices[idNear]) < 0.025:
				self.rrtVertices.append(qGoal)
				self.rrtEdges.append(idNear)
				self.found_solution = True
				break

			print(len(self.rrtVertices))
			
		if self.found_solution and not read_saved_plan:
			with open('rrtVertices.pkl', 'wb') as f:
				pickle.dump(self.rrtVertices, f)		
			with open('rrtEdges.pkl', 'wb') as f:
				pickle.dump(self.rrtEdges, f)		

		if read_saved_plan:
			self.found_solution = True
			with open('rrtVertices_lab2.pkl', 'rb') as f:
				self.rrtVertices = pickle.load(f)
			with open('rrtEdges_lab2.pkl', 'rb') as f:
				self.rrtEdges = pickle.load(f)

		### if a solution was found
		if self.found_solution:
			# Extract path
			c=-1 #Assume last added vertex is at goal 
			self.plan.insert(0, self.rrtVertices[c])

			while True:
				c=self.rrtEdges[c]
				self.plan.insert(0, self.rrtVertices[c])
				if c==0:
					break

			# Path shortening
			for i in range(150):
				anchorA = np.random.randint(0, len(self.plan)-2)
				anchorB = np.random.randint(anchorA+1, len(self.plan)-1)

				shiftA = np.random.uniform(0,1)
				shiftB = np.random.uniform(0,1)

				candidateA = (1-shiftA)*np.array(self.plan[anchorA]) + shiftA*np.array(self.plan[anchorA+1])
				candidateB = (1-shiftB)*np.array(self.plan[anchorB]) + shiftB*np.array(self.plan[anchorB+1])

				if not self.mybot.DetectCollisionEdge(candidateA, candidateB, self.pointsObs, self.axesObs):
					while anchorB > anchorA:
						self.plan.pop(anchorB)
						anchorB = anchorB-1
					self.plan.insert(anchorA+1, candidateB)
					self.plan.insert(anchorA+1, candidateA)

			
			for (i, q) in enumerate(self.plan):
				print("Plan step: ", i, "and joint: ", q)
			plan_length = len(self.plan)
			
			self.naive_interpolation()

			return

		else:
			print("No solution found")


	def dummy_position_control(self, model, data):

		# global joint_counter
		# global inc
		# global plan
		# global plan_length
		# global interpolated_plan

		# Instantite a handle to the desired body on the robot
		body = data.body("hand")

		# Check if plan is available, if not go to the home position
		if (self.found_solution==False or self.solution_interpolated==False):
			desired_joint_positions = np.array(self.qInit)
		
		else:
			# If a plan is available, cycle through poses
			self.plan_length = self.interpolated_plan.shape[0]

			if np.linalg.norm(self.interpolated_plan[self.joint_counter] - data.qpos[:7]) < 0.01 and self.joint_counter < self.plan_length:
				self.joint_counter+=self.inc

			desired_joint_positions = self.interpolated_plan[self.joint_counter]

			if self.joint_counter==self.plan_length-1:
				self.inc = -1*abs(self.inc)
				self.joint_counter-=1
			if self.joint_counter==0:
				self.inc = 1*abs(self.inc)
		

		# Set the desired joint velocities
		desired_joint_velocities = np.array([0,0,0,0,0,0,0])

		# Desired gain on position error (K_p)
		Kp = np.eye(7,7)*300

		# Desired gain on velocity error (K_d)
		Kd = 50

		# Set the actuator control torques
		data.ctrl[:7] = data.qfrc_bias[:7] + Kp@(desired_joint_positions-data.qpos[:7]) + Kd*(desired_joint_velocities-data.qvel[:7])


if __name__ == "__main__":

	# Load the xml file here
	planner = PathPlanner()

	obstacles = [
		# ([rpy, xyz], dims)
		([[0.,0.,0.], [0.1, 0.0, 1.0]], [1.0, 1.4, 0.1]), # top
		([[0.,0.,0.], [0.1, -0.65, 0.475]], [1.3, 0.1, 0.95]), # left
		([[0.,0.,0.], [0.1, 0.65, 0.475]], [1.3, 0.1, 0.95]), # right
		([[0.,0.,0.], [-0.5, 0.0, 0.475]], [0.1, 1.2, 0.95]), # bottom
		([[0.,0.,0.], [0.45, 0.0, 0.15]], [0.3, 0.3, 0.3]), # box
	]

	for obst in obstacles:
		planner.add_obstace(obst[0], obst[1])
	
	planner.set_inital_joint_config()
	planner.set_goal_joint_config()

	model = mj.MjModel.from_xml_path(planner.xml_filepath)
	data = mj.MjData(model)

	# Set the simulation scene to the home configuration
	mj.mj_resetDataKeyframe(model, data, 0)

	# Set the position controller callback
	mj.set_mjcb_control(planner.dummy_position_control)

	# Compute the RRT solution
	planner.RRTQuery(read_saved_plan=False)

	# Launch the simulate viewer
	viewer.launch(model, data)