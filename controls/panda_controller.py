import numpy as np

class FrankaController:
	def __init__(self):
		self.desired_joint_positions = np.array([0,0,0,0,0,0,0])
		self.desired_joint_velocities = np.array([0,0,0,0,0,0,0])

	def setGoalStates(self, desired_joint_positions = np.array([0,0,0,0,0,0,0]), desired_joint_velocities = np.array([0,0,0,0,0,0,0])):
		self.desired_joint_positions = desired_joint_positions
		self.desired_joint_velocities = desired_joint_velocities
		return

	def positionControlSim(self, model, data):

		# Instantite a handle to the desired body on the robot
		body = data.body("hand")

		# Desired gain on position error (K_p)
		Kp = 1000

		# Desired gain on velocity error (K_d)
		Kd = 1000

		# Set the actuator control torques
		data.ctrl[:7] = data.qfrc_bias[:7] + Kp*(self.desired_joint_positions-data.qpos[:7]) + Kd*((self.desired_joint_velocities)-data.qvel[:7])

	def jointPosControl(self,desired_joint_positions):
		# TODO directly set the desired joint positions using the franka py control library here.
		pass

	
	def cartecianPosControl(self,desired_positions):
		# TODO directly set the desired cartecian positions using the franka py control library here.
		pass