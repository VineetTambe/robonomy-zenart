import sys
import os 

fpath = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, fpath + '/..')
from controls import panda_controller
import mujoco as mj
from mujoco import viewer

import numpy as np

if __name__ == "__main__":

	
	xml_filepath = fpath + '/../assets/franka_emika_panda/panda_nohand_torque.xml'

	# Load the xml file here
	model = mj.MjModel.from_xml_path(xml_filepath)
	data = mj.MjData(model)

	# Set the simulation scene to the home configuration
	mj.mj_resetDataKeyframe(model, data, 0)

	controlObj = panda_controller.FrankaController()
	# test for joint position control ----------------------------------------------
	
	# des_joint_positions = np.array([0.5740863127850836, 0.31159654210717075, -0.679618955563617, -2.078791138726525, -0.11481848580697593, 3.914113692658701, -2.864637256298371])
	# controlObj.setGoalStates(desired_joint_positions = des_joint_positions)
	# mj.set_mjcb_control(controlObj.jointPositionControlSim)

	#----------------------------------------------------------------------

	# test for cartesian position control ----------------------------------------------
	des_pos = np.array([0.7 ,0.7 ,00.0])
	controlObj.setGoalStates(desired_cartesian_pos = des_pos)
	mj.set_mjcb_control(controlObj.cartesianPositionControlSim)
	# Launch the simulate viewer
	viewer.launch(model, data)