import mujoco as mj
from mujoco import viewer
import numpy as np
# import math

class FrankaController:
	def __init__(self):
		self.desired_joint_positions = np.array([0,0,0,0,0,0,0])
		self.desired_joint_velocities = np.array([0,0,0,0,0,0,0])

	def setGoalStates(self, desired_joint_positions = np.array([0,0,0,0,0,0,0]), desired_joint_velocities = np.array([0,0,0,0,0,0,0])):
		self.desired_joint_positions = desired_joint_positions
		self.desired_joint_velocities = desired_joint_velocities
		return

	def positionControl(self, model, data):

		# Instantite a handle to the desired body on the robot
		body = data.body("hand")

		# Desired gain on position error (K_p)
		Kp = 1000

		# Desired gain on velocity error (K_d)
		Kd = 1000

		# Set the actuator control torques
		data.ctrl[:7] = data.qfrc_bias[:7] + Kp*(self.desired_joint_positions-data.qpos[:7]) + Kd*((self.desired_joint_velocities)-data.qvel[:7])


if __name__ == "__main__":
	
	# TODO move this test code to a function Test code:

	# TODO remove absolute path with an os.dir or os.get absolute path 
	xml_filepath = "../assets/franka_emika_panda/panda_nohand_torque.xml"

	controlObj = FrankaController()

	des_joint_positions = np.array([0.5740863127850836, 0.31159654210717075, -0.679618955563617, -2.078791138726525, -0.11481848580697593, 3.914113692658701, -2.864637256298371])
	controlObj.setGoalStates(desired_joint_positions = des_joint_positions)
	# Load the xml file here
	model = mj.MjModel.from_xml_path(xml_filepath)
	data = mj.MjData(model)

	# Set the simulation scene to the home configuration
	mj.mj_resetDataKeyframe(model, data, 0)
	mj.set_mjcb_control(controlObj.positionControl)

	# Launch the simulate viewer
	viewer.launch(model, data)
