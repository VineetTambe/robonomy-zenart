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
import json
import copy

import matplotlib.pyplot as plt

def plan_trajectory():
    with open(os.path.join('/home/student/Prog/.zenart/data/processed/', 'apple.json'), 'rb') as f:
        path = json.load(f)

    max_height = 0.15
    dt = 0.02
    ts = np.arange(0, 30, dt)
    alpha = np.linspace(0, 1, len(ts))
    xForwardOffset = 0.7
    drawingPlaneZ = 0.1
    
    for stroke in path:
        stroke = np.array(stroke)
        stroke *= max_height
        stroke -= max_height//2
        newpath = []
        for point in stroke:
            y = point[1]
            x = point[0] + xForwardOffset 
            z = drawingPlaneZ
            newpath.append((x, y, z))
        newpath =  np.array(newpath)
        

        distance = np.cumsum(np.sqrt(np.sum(np.diff(newpath, axis=0)**2, axis=1 )) )
        # distance = np.insert(distance, 0, 0)/distance[-1]
        print(distance)

        # be careful with angle interpolation!
        interpolator =  interp1d(distance, newpath, kind='cubic', axis=0)
        interpPoints = interpolator(alpha)
        newpath = []
        for i, pt in enumerate(interpPoints):
            if i == len(interpPoints) - 1:
                break

            
            newpath.append(
                RigidTransform(
                    translation = pt,
                    rotation = RigidTransform.z_axis_rotation()
                )
            )

        newpath.append((x, y, z + 0.03, 0))

        print(newpath)


    # pub = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=10)
    
    # init_time = rospy.Time.now().to_time()
    # pose_traj = np.array(pose_traj)
    # dt = 0.02
    # ts = np.arange(0, 30, dt)
    # pts = []
    # xs = []
    # ys = []
    # zs = []

    # for pt in (pose_traj):
    #     xs.append(pt.translation[0])        
    #     ys.append(pt.translation[1])       
    #     zs.append(pt.translation[2])       
    #     pts.append(pt.translation[:2])  
    # pts = np.array(pts)
    # points = pts

    # distance = np.cumsum( np.sqrt(np.sum( np.diff(points, axis=0)**2, axis=1 )) )
    # distance = np.insert(distance, 0, 0)/distance[-1]

    # # Interpolation for different methods:
    # interpolations_methods = ['slinear', 'quadratic', 'cubic']
    
    
    # alpha = np.linspace(0, 1, len(ts))

    # interpolated_points = {}
    # for method in interpolations_methods:
    #     interpolator =  interp1d(distance, points, kind=method, axis=0)
    #     interpolated_points[method] = interpolator(alpha)

    # print(interpolated_points['cubic'])


    # plt.figure(figsize=(7,7))
    # for method_name, curve in interpolated_points.items():
    #     plt.plot(*curve.T, '-', label=method_name);

    # plt.plot(*points.T, 'ok', label='original points');
    # plt.axis('equal'); plt.legend(); plt.xlabel('x'); plt.ylabel('y');
    # # plt.show()
    # print(len(interpolated_points['cubic']), len(pts))

    # for i, t in enumerate(ts):
    #     translation =  list(interpolated_points['cubic'][i])
    #     translation.append(pose_traj[0].translation[2] - 0.0125)
    #     print(translation[-1], pose_traj[0].translation[2])
    #     pose_tf = RigidTransform(
    #         rotation = pose_traj[0].rotation,
    #         translation = translation
    #     )

    #     timestamp = rospy.Time.now().to_time() - init_time
    #     traj_gen_proto_msg = PosePositionSensorMessage(
    #         id=i, 
    #         timestamp=timestamp,
    #         position=pose_tf.translation, 
    #         quaternion=pose_tf.quaternion
	# 	)
    #     ros_msg = make_sensor_group_msg(
    #         trajectory_generator_sensor_msg=sensor_proto2ros_msg(
    #             traj_gen_proto_msg, 
    #             SensorDataMessageType.POSE_POSITION),
    #     )

    #     print(traj_gen_proto_msg.position, i)
    #     # Sleep the same amount as the trajectory was recorded in
    #     # rospy.loginfo('Publishing: ID {}, dt: {:.4f}'.format(traj_gen_proto_msg.id, dt))
    #     # pub.publish(ros_msg)
    #     time.sleep(dt)

    # # fa.reset_joints()
    # # pts = np.array(pts)
    # # plt.scatter(pts[:, 0], pts[:, 1])
    # # plt.show()
        
plan_trajectory()