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


class zenart:
    def __init__(self, fa):

        self.fa = fa
    
        self.x_ini = 0.15
        self.y_ini = -0.05
        # self.z_ini = 0.375
        self.z_ini = 0.375
        self.x_factor = 0.1
        self.y_factor = 0.1
        # Store the initial pose
        self.init = np.array([self.x_ini, self.y_ini, self.z_ini])

        # Initialize ros publisher to publish pose
        self.pub = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=10)
        
    def getPixelXY(self, file_name: str):

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
    
    def interpolatePointsFromTrajectory(self, pose_traj):
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

        # Interpolation for different methods:
        interpolations_methods = ['slinear', 'quadratic', 'cubic']
        
        alpha = np.linspace(0, 1, len(self.ts))
        
        # Interpolate the points to get a smooth trajectory
        interpolated_points = {}
        for method in interpolations_methods:
            interpolator =  interp1d(distance, points, kind=method, axis=0)
            interpolated_points[method] = interpolator(alpha)

        print(interpolated_points['cubic'])

    def executeTraj(self, pose_traj, trajectory_duration, dt):
        '''
            This is the general trajectory executor function
            args:
                pose_traj: List of poses (numpy arrays with 1st element translation and 2nd element rotation)
                trajectory_duration: duration of the trajectory execution
                dt: Time interval between each pose 
        '''
        init_time = rospy.Time.now().to_time()
        # This param determines the speed with which the robot will complete the trajectory
        ts = np.arange(0, trajectory_duration, self.dt)

        # Go to an initial pose - this is required to initialiize the set topic by pose API
        self.fa.goto_pose(pose_traj[0], duration = 5)
        self.fa.goto_pose((pose_traj[1]), 
                    duration=int(ts[-1]), 
                    dynamic=True, 
                    buffer_time=10, 
                    cartesian_impedances=[600.0, 600.0, 600.0, 50.0, 50.0, 50.0]
        )


        pose_traj = np.array(pose_traj)

        # Interpolate the points to get a smooth trajectory
        interpolated_points = self.interpolatePointsFromTrajectory(pose_traj)

        # ----------------- Plot the results of trajectory interpolation -----------------
        plt.figure(figsize=(7,7))
        for method_name, curve in interpolated_points.items():
            plt.plot(*curve.T, '-', label=method_name)

        plt.plot(interpolated_points.T, 'ok', label='original points')
        plt.axis('equal'); plt.legend(); plt.xlabel('x'); plt.ylabel('y')
        # plt.show()
        print(len(interpolated_points['cubic']), len(pts))
        #--------------------------------------------------------------------------------
        
        # Publish the trajectory
        for i, t in enumerate(ts):
            translation =  list(interpolated_points['cubic'][i])

            # TODO figure out a better way to compute Z - rather than trial and error to find correct Z offset
            # Append the Z offset to so that the brush touches the board.
            translation.append(pose_traj[0].translation[2] - 0.0125)
            
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
            self.pub.publish(ros_msg)
            time.sleep(dt)

    # TODO
    # 1. add this function inside a timer callback
    # 2. Add a state machine and move it to BRUSH_DIPPING state when this is happening
    def dipBrushIntoWater(self):
        
        # Certain notes about this manuver - 
        # It has to be fast and intermediate motions should be minimal otherwise drawing will evaporate.

        # This is point is the center of the water tray at the top surface
        # It is assumed that the x and y axis of the box and the robot base are alligned
        
        prev_pose = fa.get_pose()

        dip_height = 0.01 # 1cm
        water_tray_location = ([np.asarray([0.52413815, # x
                                            -0.05815484, # y 
                                            0.10330608]), # z
        
                            # Zrot = rotation about (reset) initial pose z axis
                            # TODO test this rotation without any translation
                            np.asarray([[ 1,  0,  0 ],
                                        [ 0, -1,  0 ],
                                        [ 0,  0, -1]])
                                ])

        water_tray_dims = np.asarray([0.09241, # width 92.41 mm
                            0.134, # length 134mm
                            0.0353]) # height 35.3
        
        # 0. Stop whatever was happening before 
        fa.stop_skill()

        # 1. Move franka arm to the water tray
        fa.goto_pose(RigidTransform(rotation = water_tray_location[1], 
                                    translation = water_tray_location[0] + np.asarray([0, 0, -dip_height]),
                                    from_frame='franka_tool', 
                                    to_frame='world'), 
                    duration = 5)

        # 2. Dip the brush into the water at a tilt and translate in the x direction
        fa.goto_pose(RigidTransform(rotation = water_tray_location[1], 
                                    translation = water_tray_location[0] + np.asarray([water_tray_dims[0], water_tray_dims[1], dip_height]),
                                    from_frame='franka_tool', 
                                    to_frame='world'), 
                    duration = 5)

        # 3. Rotate 360 degrees about z axis 

        # 4. Move out of the water tray with brush touching the wall - (stretch goal)
        

        # TODO enable the publisher API by making last goto pose the following command

        
        fa.goto_pose(prev_pose, 
                    duration=int(ts[-1]), 
                    dynamic=True, 
                    buffer_time=10, 
                    cartesian_impedances=[600.0, 600.0, 600.0, 50.0, 50.0, 50.0]
        )

        # Move the state machine state to whatever was happening before this

    def run(self):
        # Step1. reset arm to fixed position. 
        self.fa.reset_joints()
        
        # Get initial pose
        p0 = self.fa.get_pose()
        p1 = p0.copy()
        pose_traj = []

        self.dipBrushIntoWater()
        pass

        # From a local file load the timestamps and the offsets
        p = self.getPixelXY('apple.pkl')

        # for every pixel point convert it to a set of translations and rotations to generate a list of poses
        for d in p:
            print(d)
            print(d.shape)
            T_delta = RigidTransform(
            translation= self.init + d,
            rotation=RigidTransform.z_axis_rotation(np.deg2rad(0)), 
                                from_frame=p1.from_frame, to_frame=p1.from_frame)
            p1 = p0*T_delta
            pose_traj.append(p1)
        
        
        # Trajectory parameters
        dt = 0.02
        trajectory_duration = 30
        
        self.executeTraj(pose_traj, trajectory_duration, dt)

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

    # Create FrankaArm object
    fa = FrankaArm()
    zenart_obj = zenart(fa)
