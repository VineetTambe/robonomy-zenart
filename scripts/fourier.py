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
import datetime
from frankapy.utils import convert_array_to_rigid_transform
import time
data_dir = '../data/processed'

import matplotlib.pyplot as plt


def getClockXY(r, index):
    deltas = []
    # deltas.append(np.array([0, 0, 0]))
    theta = np.linspace(0, np.pi * 2, 100)
    
    x = r * np.sin(theta) 
    y = r * np.cos(theta) 
    z = np.zeros_like(x)
    path = list(zip(x, y, z))
    for p in path:
        deltas.append(p)

    # following lines for the clock arms

    xi, yi, zi = np.array(deltas[-1]) - 0.001
    xf, yf, zf = 0.0, 0.0, zi - 0.05

    moveUpx = np.linspace(xi, xf, 10)
    moveUpy = np.linspace(yi, yf, 10)
    moveUpz = np.linspace(zi, zf, 10)
    print(np.vstack([moveUpx, moveUpy, moveUpz]).T.shape)
    deltas += list(np.vstack([moveUpx, moveUpy, moveUpz]).T)
    shape = moveUpx.shape
    deltas += list(np.vstack([np.random.randn(*shape) * 1e-10, np.random.randn(*shape) * 1e-10, moveUpz[::-1] + 0.001]).T)

    now = datetime.datetime.now() + datetime.timedelta(minutes = index * 15)  
    hour = now.hour % 12 
    minute = now.minute % 60
    hourAngle = (2 * np.pi * hour / 12  + (minute / 60) * (2 * np.pi/12)) - np.pi
    for i in np.linspace(0, r * 0.5, 10): 
        arm1x = i * np.cos(hourAngle)
        arm1y = i * np.sin(hourAngle)
        arm1z = 0.0
        deltas.append([arm1x, arm1y, arm1z])


    # # trajectory that lifts and goes to center
    xi, yi, zi = np.array(deltas[-1]) - 0.001
    xf, yf, zf = 0.0, 0.0, zi - 0.05



    moveUpx = np.linspace(xi, xf, 10)
    moveUpy = np.linspace(yi, yf, 10)
    moveUpz = np.linspace(zi, zf, 10)
    print(np.vstack([moveUpx, moveUpy, moveUpz]).T.shape)
    deltas += list(np.vstack([moveUpx, moveUpy, moveUpz]).T)
    shape = moveUpx.shape
    deltas += list(np.vstack([np.random.randn(*shape) * 1e-10, np.random.randn(*shape) * 1e-10, moveUpz[::-1] + 0.001]).T)



    # print(hour, minute)


    minuteAngle = (minute * 2 * np.pi / 60) - np.pi
    # minuteAngle = 0

    for i in np.linspace(0, r*0.85, 10): 
        arm1x = i * np.cos(minuteAngle)
        arm1y = i * np.sin(minuteAngle)
        arm1z = 0.0
        deltas.append([arm1x, arm1y, arm1z])


    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    p = np.vstack(deltas)
    ax.scatter(p[:, 0], p[:, 1], p[:, 2])
    # plt.axis('equal')
    # plt.show()
    return p


def getCircleXY(r = 0.05):
    deltas = []
    # deltas.append(np.array([0, 0, 0]))
    theta = np.linspace(0, np.pi * 2, 100)
    
    x = r * np.sin(theta) 
    y = r * np.cos(theta) 
    z = np.zeros_like(x)
    path = list(zip(x, y, z))
    for p in path:
        deltas.append(p)
    p = np.vstack(deltas)
    p[:,2]=0
    print(p)
    plt.scatter(p[:, 0], p[:, 1])
    plt.show()
    return p


def getPixelXY(file_name):

    with open(os.path.join(data_dir, file_name), 'rb') as f:
        path = pickle.load(f)

    # sf = 0.1
    path = [path]
    # print(path.shape)
    # print(len(path))
    deltas = []
    # deltas.append(np.array([0, 0, 0]))
    for p in path:
        print(p.shape)
        # p = p/8 # for apple
        p = p/4
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
    p2 = np.zeros_like(p)
    p2[:,0] = p[:,1]
    p2[:,1] = p[:,0]
    print(p2)
    return p2


# TODO
# 1. add this function inside a timer callback
# 2. Add a state machine and move it to BRUSH_DIPPING state when this is happening

def dipBrushIntoWater(fa : FrankaArm, ts):
    
    # Certain notes about this manuver - 
    # It has to be fast and intermediate motions should be minimal otherwise drawing will evaporate.

    # This is point is the center of the water tray at the top surface
    # It is assumed that the x and y axis of the box and the robot base are alligned

    dip_height = 0.025 # 1.5cm
    # TODO find out this coordinate
    water_tray_location = ([np.asarray([0.7, # x
                                        0.445, # y 
                                        0.14]), # z
    
                        # Zrot = rotation about (reset) initial pose z axis
                        # TODO test this rotation without any translation
                        np.asarray([[ 1,  0,  0 ],
                                    [ 0, -1,  0 ],
                                    [ 0,  0, -1]])
                            ])
    
    water_tray_dims = np.asarray([0.134, # length 134mm
                           0.09241, # width 92.41 mm
                           0.0353]) # height 35.3
    
    # 0. Stop whatever was happening before 
    fa.stop_skill()

    poses = []

    rotation = RigidTransform.x_axis_rotation( 20 * np.pi / 180)
    rotation = rotation @ RigidTransform.y_axis_rotation( -20 * np.pi / 180)

    # 1. Move franka arm to the water tray
    poses.append(RigidTransform(rotation = water_tray_location[1], 
                                translation = water_tray_location[0] + np.asarray([0, 0, dip_height]),
                                from_frame='franka_tool', 
                                to_frame='world'))
    

    poses.append(RigidTransform(rotation = water_tray_location[1], 
                            translation = water_tray_location[0] + np.asarray([0, 0, -dip_height]),
                            from_frame='franka_tool', 
                            to_frame='world'))

    # 2. Dip the brush into the water at a tilt and translate in the x direction
    poses.append(RigidTransform(rotation = water_tray_location[1] @ RigidTransform.z_axis_rotation(np.pi), 
                                translation = water_tray_location[0] + np.asarray([- water_tray_dims[0], - water_tray_dims[1], 0.00]),
                                from_frame='franka_tool', 
                                to_frame='world'))

    for pose in poses:
        fa.goto_pose(pose, duration = 5)

    # 3. Rotate 360 degrees about z axis 

    # 4. Move out of the water tray with brush touching the wall - (stretch goal)
    
    # TODO enable the publisher API by making last goto pose the following command
    fa.reset_joints()
    # fa.goto_pose(prev_pose, 
    #              duration=5, 
    #              dynamic=True, 
    #              buffer_time=10, 
    #              cartesian_impedances=[600.0, 600.0, 600.0, 50.0, 50.0, 50.0]
    # )

    # Move the state machine state to whatever was happening before this

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

    
   
    x_ini = 0.2
    y_ini = -0.05
    z_ini = 0.3825
    # z_ini = 0.215
    x_factor = 0.1
    y_factor = 0.1
    

    # Create FrankaArm object
    fa = FrankaArm()
    
    
    # Step1. reset arm to fixed position. 
    fa.reset_joints()
    p0 = fa.get_pose()
    p1 = p0.copy()

    # Get initial pose
    
  

    # print(p1)
    dt = 0.02
    trajectory_duration = 20
    # This param determines the speed with which the robot will complete the trajectory
    ts = np.arange(0, trajectory_duration, dt)
    
    # #---------------------------------------------------

    # # Store the initial pose
    init = np.array([x_ini, y_ini, z_ini])

    pub = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=10)
    # for i in range(300):
    index = 0
    # while True:

# fa.reset_joints()
    dipBrushIntoWater(fa, ts)
    
    pose_traj = []
    # #---------------------------------------------------

    # From a local file load the timestamps and the offsets
    p = getPixelXY('data2.pkl')
    # p = getClockXY(r = 0.075, index = index)
    index = index +1
    # p = getCircleXY(r = 0.05)

    # for every pixel point convert it to a set of translations and rotations to generate a list of poses
    for d in p:
        print(d)
        print(d.shape)
        T_delta = RigidTransform(
        translation=init + d,
        rotation=RigidTransform.z_axis_rotation(np.deg2rad(0)), 
                            from_frame=p1.from_frame, to_frame=p1.from_frame)
        p1 = p0*T_delta
        pose_traj.append(p1)

    # Initialize ros publisher to publish pose
    
    
    init_time = rospy.Time.now().to_time()
    pose_traj = np.array(pose_traj)

    
    # Go to an initial pose - this is required to initialiize the set topic by pose API
    print(pose_traj[0])
    print(pose_traj[1])

    fa.goto_pose(pose_traj[0], duration = 5)
    fa.goto_pose(pose_traj[1], duration = 5, use_impedance=False, cartesian_impedances=[600.0, 600.0, 3000.0, 100.0, 100.0, 100.0])
    fa.goto_pose((pose_traj[1]), 
                duration=int(ts[-1]), 
                dynamic=True, 
                buffer_time=40, 
                use_impedance=True,
                #  cartesian_impedances=[600.0, 600.0, 600.0, 50.0, 50.0, 50.0]
                cartesian_impedances=[1500.0, 1500.0, 3000.0, 100.0, 100.0, 100.0]
    )

    pts = []
    xs = []
    ys = []
    zs = []

    # For every Pose 
    for pt in (pose_traj):
        xs.append(pt.translation[0])        
        ys.append(pt.translation[1])       
        zs.append(pt.translation[2])       
        pts.append(pt.translation[:2]) # X and Y coordinates to compute the distance 
    points = np.array(pts)

    distance = np.cumsum( np.sqrt(np.sum( np.diff(points, axis=0)**2, axis=1 )))
    distance = np.insert(distance, 0, 0)/distance[-1]
    print(distance)

    # Interpolation for different methods:
    interpolations_methods = ['slinear', 'quadratic', 'cubic']
    
    alpha = np.linspace(0, 1, len(ts))
    
    # Interpolate the points to get a smooth trajectory
    interpolated_points = {}
    for method in interpolations_methods:
        interpolator =  interp1d(distance, points, kind=method, axis=0)
        interpolated_points[method] = interpolator(alpha)

    print(interpolated_points['cubic'])

    # Plot the results
    plt.figure(figsize=(7,7))
    for method_name, curve in interpolated_points.items():
        plt.plot(*curve.T, '-', label=method_name)

    plt.plot(*points.T, 'ok', label='original points')
    plt.axis('equal'); plt.legend(); plt.xlabel('x'); plt.ylabel('y');
    plt.show()
    print(len(interpolated_points['cubic']), len(pts))

    # Publish the trajectory
    for i, t in enumerate(ts):
        translation =  list(interpolated_points['cubic'][i])

        # TODO figure out a better way to compute Z - rather than trial and error to find correct Z offset
        # Append the Z offset to so that the brush touches the board.

        translation.append(pose_traj[1].translation[2])
        
        # Publish the pose on the ros topic
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

    fa.stop_skill()
    fa.reset_joints() 
        # time.sleep(10)
