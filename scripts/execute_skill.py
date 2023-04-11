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


def getDeltas(file_name):

    with open(os.path.join(data_dir, file_name), 'rb') as f:
        path = pickle.load(f)

    sf = 0.1
    print(len(path))
    deltas = []
    # deltas.append(np.array([0, 0, 0]))
    for p in path:
        print(p.shape)
        p = p/8
        mask = (p[:,0]<1) & (p[:,1]<1)
        p = p[mask][::2]
        print(p)
        if (len(p)>0):
            deltas.append(p)
            # deltas.append(p[0])
            # deltas.append(p[-1])
    # deltas.append(np.array([0, 0, 0]))
    p = np.vstack(deltas)
    p[:,2]=0
    print(p)
    return p


if __name__ == "__main__":
    
    # reset franka to its home joints
    print("starting script")

    
    # record traj this way ->

    # guide_duration =  1000
    # fa.run_guide_mode(guide_duration, block=False) #hitting joint limit will end early
    # while True:
    #     print('-'*20)
    #     print(fa.get_pose())
    #     print(fa.get_joints())
    #     time.sleep(5)

    # ------------------------------


    p = getDeltas('apple.pkl')

    pose_traj = []
   
    x_ini = 0.15
    y_ini = -0.05
    # z_ini = 0.375
    z_ini = 0.375
    x_factor = 0.1
    y_factor = 0.1
    

    fa = FrankaArm()
    # Step1. reset arm to fixed position. 
    fa.reset_joints()

    p0 = fa.get_pose()
    p1 = p0.copy()
  

    init = np.array([x_ini, y_ini, z_ini])
    # plt.scatter(p[:,0], p[:,1])
    # plt.show()

    for d in p:
        print(d)
        print(d.shape)
        T_delta = RigidTransform(
        translation=init + d,
        rotation=RigidTransform.z_axis_rotation(np.deg2rad(0)), 
                            from_frame=p1.from_frame, to_frame=p1.from_frame)
        p1 = p0*T_delta
        pose_traj.append(p1)


    pub = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=10)
    
    init_time = rospy.Time.now().to_time()
    pose_traj = np.array(pose_traj)
    dt = 0.02
    # dt = 0.02
    ts = np.arange(0, 30, dt)
    fa.goto_pose(pose_traj[0], duration = 5)
    fa.goto_pose((pose_traj[1]), 
                 duration=int(ts[-1]), 
                 dynamic=True, 
                 buffer_time=10, 
                 cartesian_impedances=[600.0, 600.0, 600.0, 50.0, 50.0, 50.0]
    )

    pts = []
    xs = []
    ys = []
    zs = []

    for pt in (pose_traj):
        xs.append(pt.translation[0])        
        ys.append(pt.translation[1])       
        zs.append(pt.translation[2])       
        pts.append(pt.translation[:2])  
    pts = np.array(pts)
    points = pts

    distance = np.cumsum( np.sqrt(np.sum( np.diff(points, axis=0)**2, axis=1 )) )
    distance = np.insert(distance, 0, 0)/distance[-1]

    # Interpolation for different methods:
    interpolations_methods = ['slinear', 'quadratic', 'cubic']
    
    alpha = np.linspace(0, 1, len(ts))

    interpolated_points = {}
    for method in interpolations_methods:
        interpolator =  interp1d(distance, points, kind=method, axis=0)
        interpolated_points[method] = interpolator(alpha)

    print(interpolated_points['cubic'])


    plt.figure(figsize=(7,7))
    for method_name, curve in interpolated_points.items():
        plt.plot(*curve.T, '-', label=method_name)

    plt.plot(*points.T, 'ok', label='original points')
    plt.axis('equal'); plt.legend(); plt.xlabel('x'); plt.ylabel('y');
    # plt.show()
    print(len(interpolated_points['cubic']), len(pts))

    for i, t in enumerate(ts):
        translation =  list(interpolated_points['cubic'][i])
        translation.append(pose_traj[0].translation[2] - 0.0125)
        # print(fa.get_pose().translation)
        pose_tf = RigidTransform(
            rotation = pose_traj[0].rotation,
            translation = translation
        )

        timestamp = rospy.Time.now().to_time() - init_time
        traj_gen_proto_msg = PosePositionSensorMessage(
            id=i, 
            timestamp=timestamp,
            position=pose_tf.translation, 
            quaternion=pose_tf.quaternion
		)
        print(np.array(fa.get_pose().translation), translation)
        # print(np.array(fa.get_pose().translation - traj_gen_proto_msg.position))
        ros_msg = make_sensor_group_msg(
            trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                traj_gen_proto_msg, 
                SensorDataMessageType.POSE_POSITION),
        )

        # print(traj_gen_proto_msg.position, i)
        # Sleep the same amount as the trajectory was recorded in
        # rospy.loginfo('Publishing: ID {}, dt: {:.4f}'.format(traj_gen_proto_msg.id, dt))
        pub.publish(ros_msg)
        time.sleep(dt)

        # TODO:
        # 1. Z position - not constant - Ronit
        # 2. Add Tilt as per brush stroke direction - Praveen 
        # 3. Brush dipping - Vineet
        # 4. Code clean up - Vineet
        # 5. Debug API - possibility of issue with API