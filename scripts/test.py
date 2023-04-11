import numpy as np
from autolab_core import RigidTransform

from frankapy import FrankaArm
import time 
from scipy.interpolate import interp1d
import pickle
import rospy
import os
from frankapy import FrankaConstants as FC
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import PosePositionSensorMessage, ShouldTerminateSensorMessage
from franka_interface_msgs.msg import SensorDataGroup
from autolab_core import RigidTransform
from frankapy import FrankaArm, SensorDataMessageType
from frankapy import FrankaConstants as FC
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import PosePositionSensorMessage, ShouldTerminateSensorMessage
from franka_interface_msgs.msg import SensorDataGroup

from frankapy.utils import convert_array_to_rigid_transform

data_dir = '../data/processed'

import matplotlib.pyplot as plt
if __name__ == "__main__":
    

    fa = FrankaArm()
    fa.reset_joints()
    xrot = 15
    yrot = -30
    curpose = fa.get_pose()
    rotation = curpose.rotation   
    # rotation = rotation @ RigidTransform.z_axis_rotation( 180 * np.pi/180)
    rotation = rotation @ RigidTransform.x_axis_rotation(xrot * np.pi / 180)
    rotation = rotation @ RigidTransform.y_axis_rotation(yrot * np.pi / 180)
    
    topose =RigidTransform(
        translation = curpose.translation,
    #     from_frame='franka_tool',
        rotation = rotation,
        from_frame='franka_tool',
        to_frame= 'world'
    )
    print("GOING TO POSE")
    fa.goto_pose(topose, duration = 5)
    # curpose = fa.get_pose()
    # topose =RigidTransform(
    #     translation = curpose.translation,
    #     rotation = curpose.rotation  ,
    #     from_frame='franka_tool',
    #     to_frame= 'world'
    # )
    # fa.goto_pose(topose, duration = 5)