import numpy as np
import mujoco as mj

class FrankaController:
	def __init__(self):
		self.desired_cartesian_positions = np.array([0,0,0])
		self.desired_cartesian_velocities = np.array([0,0,0])
		self.desired_joint_positions = np.array([0,0,0,0,0,0,0])
		self.desired_joint_velocities = np.array([0,0,0,0,0,0,0])

	def setGoalStates(self, 
			desired_cartesian_pos = np.array([0,0,0]),
			desired_cartesian_vel = np.array([0,0,0]),
			desired_joint_pos = np.array([0,0,0,0,0,0,0]), 
			desired_joint_vel = np.array([0,0,0,0,0,0,0])):
		
		self.desired_cartesian_positions = desired_cartesian_pos
		self.desired_cartesian_velocities = desired_cartesian_vel
		self.desired_joint_positions = desired_joint_pos
		self.desired_joint_velocities = desired_joint_vel
		return

	def jointPositionControlSim(self, model, data):

		# Instantite a handle to the desired body on the robot
		body = data.body("hand")

		# Desired gain on position error (K_p)
		Kp = 1000

		# Desired gain on velocity error (K_d)
		Kd = 1000

		# Set the actuator control torques
		data.ctrl[:7] = data.qfrc_bias[:7] + Kp*(self.desired_joint_positions-data.qpos[:7]) + Kd*((self.desired_joint_velocities)-data.qvel[:7])

	# Control callback for an impedance controller
	def cartesianPositionControlSim(self, model, data): #TODO:
		try :
			# Get the Jacobian at the desired location on the robot
			jacp = np.zeros((3, model.nv))
			jacr = np.zeros((3, model.nv))
			bodyid = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, 'hand')
			mj.mj_jacBody(model, data, jacp, jacr, bodyid)
			full_jacobian = np.concatenate((jacp,jacr))

			# Instantite a handle to the desired body on the robot
			Kd = 0.750
			Kp = 8.0

			curr_pos = data.xpos[bodyid]
			# curr_vel = (curr_pos - prev_pos) / (curr_time - prev_time)
			handVel = np.zeros((6,))

			curr_vel = np.zeros((3,))
			mj.mj_objectVelocity(model,data, mj.mjtObj.mjOBJ_BODY, bodyid, handVel[:6], True)
			curr_vel = handVel[:3]
			# Get the position error
			error_pos = self.desired_cartesian_positions - curr_pos
			error_vel = self.desired_cartesian_velocities - curr_vel

			# This function works by taking in return parameters!!! Make sure you supply it with placeholder
			# variables
			des_force = np.zeros((6,1))
			des_force[:3] = (Kp * error_pos + Kd * error_vel).reshape(3,-1) 
			des_force[1] = 0
			des_force[2] = 0

			# Compute the impedance control input torques
			calculated_torques = np.squeeze((full_jacobian.T @ des_force))

			# Set the control inputs
			data.ctrl[:7] = data.qfrc_bias[:7] + calculated_torques
		except Exception as e:
			print(e)

	def directControlSim(self, desired_pose):
		# TODO directly set the desired_pose to the sim pose -> this is just for simulation purpose.
		pass

	def jointPosControl(self,desired_joint_positions):
		# TODO directly set the desired joint positions using the franka py control library here.
		pass

	
	def cartesianPosControl(self,desired_positions):
		# TODO directly set the desired cartesian positions using the franka py control library here.
		pass