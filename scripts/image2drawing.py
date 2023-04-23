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
data_dir = '../data'

import matplotlib.pyplot as plt


def euclidianDist(x1,y1,x2,y2):
    return np.sqrt((x2-x1)**2 + (y2-y1)**2)

def getTraj(file_name, x_scale = 0.5, y_scale = 0.5):
    coordgraph = open(os.path.join(data_dir , "XYfiles", file_name), 'r')

    dist_threshold = 3.0

    z_pick_height = 0.02
    z_safety_offset = 0.0
    z_height = 0.0

    xarr = []
    yarr = []
    x_final = []
    y_final = []
    z_final = []

    for line in coordgraph:
        x,y = line.split(",")
        xarr.append(int(x))
        yarr.append(-int(y))

    # flag = []

    x_diff = []
    y_diff = []
    z_diff = []

    for i in range(0,len(xarr)-1, 3):
        dist = euclidianDist(xarr[i],yarr[i],xarr[i+1],yarr[i+1])
        if dist > dist_threshold:
            x_interpolate = np.linspace(xarr[i],xarr[i+1],50)
            y_interpolate = np.linspace(yarr[i],yarr[i+1],50)

            x = np.linspace(-dist/2, dist/2, 50)
            y = -x**2 
            z_interpolate = -(y-np.min(y))/(np.max(y)-np.min(y)) * z_pick_height - z_safety_offset
            x_final += list(x_interpolate)
            y_final += list(y_interpolate)
            z_final += list(z_interpolate)
        else:
            x_final.append(xarr[i])
            y_final.append(yarr[i])
            z_final.append(z_height)
    deltas = []
    
    x_final = [((x - np.min(x_final)) / (np.max(x_final) - np.min(x_final))) * x_scale for x in x_final]
    y_final = [((y - np.min(y_final)) / (np.max(y_final) - np.min(y_final))) * y_scale for y in y_final]

    path = list(zip(x_final, y_final, z_final))
    for p in path:
        deltas.append(p)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    p = np.vstack(deltas)

    ax.scatter(p[:, 0], p[:, 1], p[:, 2])
    
    print(p[:-5,:])
    
    # ax.scatter(x_diff, y_diff, z_diff, c='r', marker='o')
    # print(x_diff)
    # plt.axis('equal')
    plt.show()
    return p

# def getClockXY(r):
#     deltas = []
#     # deltas.append(np.array([0, 0, 0]))
#     theta = np.linspace(0, np.pi * 2, 100)
    
#     x = r * np.sin(theta) 
#     y = r * np.cos(theta) 
#     z = np.zeros_like(x)
#     path = list(zip(x, y, z))
#     for p in path:
#         deltas.append(p)

#     # following lines for the clock arms

#     xi, yi, zi = np.array(deltas[-1]) - 0.001
#     xf, yf, zf = 0.0, 0.0, zi - 0.05

#     moveUpx = np.linspace(xi, xf, 10)    z_pick_height = 0.05
#     z_safety_offset = 0.01
#     z_height = 0.0pe)
#     deltas += list(np.vstack([moveUpx, moveUpy, moveUpz]).T)
#     shape = moveUpx.shape
#     deltas += list(np.vstack([np.random.randn(*shape) * 1e-10, np.random.randn(*shape) * 1e-10, moveUpz[::-1] + 0.001]).T)

#     now = datetime.datetime.now()

#     hour = now.hour % 12
#     minute = now.minute % 60
#     hourAngle = (2 * np.pi * hour / 12  + (minute / 60) * (2 * np.pi/12)) - np.pi
#     for i in np.linspace(0, r * 0.5, 10): 
#         arm1x = i * np.cos(hourAngle)
#         arm1y = i * np.sin(hourAngle)
#         arm1z = 0.0
#         deltas.append([arm1x, arm1y, arm1z])


#     # # trajectory that lifts and goes to center
#     xi, yi, zi = np.array(deltas[-1]) - 0.001
#     xf, yf, zf = 0.0, 0.0, zi - 0.05



#     moveUpx = np.linspace(xi, xf, 10)
#     moveUpy = np.linspace(yi, yf, 10)
#     moveUpz = np.linspace(zi, zf, 10)
#     print(np.vstack([moveUpx, moveUpy, moveUpz]).T.shape)
#     deltas += list(np.vstack([moveUpx, moveUpy, moveUpz]).T)
#     shape = moveUpx.shape
#     deltas += list(np.vstack([np.random.randn(*shape) * 1e-10, np.random.randn(*shape) * 1e-10, moveUpz[::-1] + 0.001]).T)



#     # print(hour, minute)


#     minuteAngle = (minute * 2 * np.pi / 60) - np.pi
#     # minuteAngle = 0

#     for i in np.linspace(0, r*0.85, 10): 
#         arm1x = i * np.cos(minuteAngle)
#         arm1y = i * np.sin(minuteAngle)
#         arm1z = 0.0
#         deltas.append([arm1x, arm1y, arm1z])


#     fig = plt.figure()
#     ax = fig.add_subplot(111, projection='3d')

#     p = np.vstack(deltas)
#     ax.scatter(p[:, 0], p[:, 1], p[:, 2])
#     # plt.axis('equal')
#     # plt.show()
#     return p


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

    with open(os.path.join(data_dir , "/processed", file_name), 'rb') as f:
        path = pickle.load(f)

    # sf = 0.1
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

    
   
    x_ini = 0.0
    y_ini = -0.05
    # z_ini = 0.3825
    z_ini = 0.215 # this is the debug z
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

    # fa.reset_joints()
    # dipBrushIntoWater(fa, ts)
        
    pose_traj = []
    # #---------------------------------------------------

    # From a local file load the timestamps and the offsets
    # p = getPixelXY('apple.pkl')
    # p = getClockXY(r = 0.075)
    # p = getCircleXY(r = 0.05)
    p = getTraj("hut.dot", x_scale = 0.5, y_scale = 0.5)

    # ts = np.linspace(0,trajectory_duration,p.shape[0])
    # Debug operatio REMOVE this
    

    # for every pixel point convert it to a set of translations and rotations to generate a list of poses
    for d in p:
        # print(d)
        # print(d.shape)
        T_delta = RigidTransform(
        translation=init + d,
        rotation=RigidTransform.z_axis_rotation(np.deg2rad(0)), 
                            from_frame=p1.from_frame, to_frame=p1.from_frame)
        p1 = p0*T_delta
        pose_traj.append(p1)
    # exit()
    # Initialize ros publisher to publish pose
    
    # print(pose_traj)

    init_time = rospy.Time.now().to_time()
    pose_traj = np.array(pose_traj)
    print("-" * 20)
    for pt in (pose_traj):
        print(pt.translation)
    print("-" * 20)
    # Go to an initial pose - this is required to initialiize the set topic by pose API
    print(pose_traj[0])
    print(pose_traj[1])

    fa.goto_pose(pose_traj[0], duration = 5)
    fa.goto_pose(pose_traj[1], duration = 5, use_impedance=False, cartesian_impedances=[600.0, 600.0, 3000.0, 100.0, 100.0, 100.0])
    fa.goto_pose((pose_traj[1]), 
                duration=int(ts[-1]), 
                dynamic=True, 
                buffer_time=10, 
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
    print(points)
    distance = np.cumsum( np.sqrt(np.sum( np.diff(points, axis=0)**2, axis=1 )))
    distance = np.insert(distance, 0, 0)/distance[-1]

    print("distance: ",distance)

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
    print(*points.T)
    plt.plot(*points.T, 'ok', label='original points')
    plt.axis('equal'); plt.legend(); plt.xlabel('x'); plt.ylabel('y')
    plt.show()
    print(len(interpolated_points['cubic']), len(pts))

    # Publish the trajectory
    for i, t in enumerate(ts):
        # translation =  list(interpolated_points['cubic'][i])
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
        # pub.publish(ros_msg)
        time.sleep(dt)

    fa.stop_skill()
    fa.reset_joints() 
    # time.sleep(10)
