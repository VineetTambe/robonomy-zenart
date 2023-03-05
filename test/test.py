import sys
sys.path.insert(0, '..')

from controls import panda_controller
import mujoco as mj
from mujoco import viewer
import os 
import numpy as np

if __name__ == "__main__":

	fpath = os.path.dirname(os.path.abspath(__file__))
	xml_filepath = fpath + '/../assets/franka_emika_panda/panda_nohand_torque.xml'
	controlObj = panda_controller.FrankaController()

	des_joint_positions = np.array([0.5740863127850836, 0.31159654210717075, -0.679618955563617, -2.078791138726525, -0.11481848580697593, 3.914113692658701, -2.864637256298371])
	controlObj.setGoalStates(desired_joint_positions = des_joint_positions)
	# Load the xml file here
	model = mj.MjModel.from_xml_path(xml_filepath)
	data = mj.MjData(model)

	# Set the simulation scene to the home configuration
	mj.mj_resetDataKeyframe(model, data, 0)
	mj.set_mjcb_control(controlObj.positionControlSim)

	# Launch the simulate viewer
	viewer.launch(model, data)
