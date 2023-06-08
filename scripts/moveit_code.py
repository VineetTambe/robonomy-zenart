#!/usr/bin/env python2.7

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from mpl_toolkits.mplot3d import Axes3D 
import time
import matplotlib.pyplot as plt
import datetime

## END_SUB_TUTORIAL
import numpy as np
import os
import json

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this tutorial the group is the primary
    ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
    ## If you are using a different robot, change this value to the name of your robot
    ## arm planning group.
    ## This interface can be used to plan and execute motions:
    group_name = "panda_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print ("============ Planning frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print ("============ End effector link: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print( "============ Available Planning Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print ("============ Printing robot state")
    print (robot.get_current_state())
    print ("")
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names


  def go_to_joint_state(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_to_joint_state
    ##
    ## Planning to a Joint Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^^
    ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
    ## thing we want to do is move it to a slightly better configuration.
    # We can get the joint values from the group and adjust some of the values:
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -pi/4
    joint_goal[2] = 0
    joint_goal[3] = -pi/2
    joint_goal[4] = 0
    joint_goal[5] = pi/3
    joint_goal[6] = 0

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()

    ## END_SUB_TUTORIAL

    # For testing:
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def go_to_pose_goal(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4

    move_group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  def getClockXY(self, r):
    deltas = []
    # deltas.append(np.array([0, 0, 0]))
    theta = np.linspace(0, np.pi * 2, 12)
    
    x = r * np.sin(theta) 
    y = r * np.cos(theta) 
    z = np.zeros_like(x)
    path = list(zip(x, y, z))
    for p in path:
        deltas.append(p)

    # following lines for the clock arms

    xi, yi, zi = np.array(deltas[-1]) - 0.001
    xf, yf, zf = 0.0, 0.0, zi + 0.05

    moveUpx = np.linspace(xi, xf, 10)
    moveUpy = np.linspace(yi, yf, 10)
    moveUpz = np.linspace(zi, zf, 10)
    print(np.vstack([moveUpx, moveUpy, moveUpz]).T.shape)
    deltas += list(np.vstack([moveUpx, moveUpy, moveUpz]).T)
    shape = moveUpx.shape
    deltas += list(np.vstack([np.random.randn(*shape) * 1e-10, np.random.randn(*shape) * 1e-10, moveUpz[::-1] + 0.001]).T)

    now = datetime.datetime.now()

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
    # ax = fig.add_subplot(111, projection='3d')
    # ax = fig.gca(projection='3d')
    p = np.vstack(deltas)
    # ax.scatter(p[:, 0], p[:, 1], p[:, 2])
    plt.scatter(p[:, 0], p[:, 1], c ="blue")

    plt.axis('equal')
    plt.show()
    return p
  
  def euclidianDist(self, x1,y1,x2,y2):
    return np.sqrt((x2-x1)**2 + (y2-y1)**2)
  
  def getTraj(self, file_name, x_scale = 0.5, y_scale = 0.5):
    data_dir = '../data'
    coordgraph = open(os.path.join(data_dir , "XYfiles", file_name), 'r')

    dist_threshold = 3.0

    z_pick_height = -0.02
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

    for i in range(0,len(xarr)-1, 2):
        dist = self.euclidianDist(xarr[i],yarr[i],xarr[i+1],yarr[i+1])
        if dist > dist_threshold:
            x_interpolate = np.linspace(xarr[i] + 1e-3,xarr[i+1] - 1e-3,50)
            y_interpolate = np.linspace(yarr[i] + 1e-3,yarr[i+1] - 1e-3,50)

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
    # ax = fig.add_subplot(111, projection='3d')

    p = np.vstack(deltas)

    # ax.scatter(p[:, 0], p[:, 1], p[:, 2])
    plt.scatter(p[:, 0], -p[:, 1], c ="blue")
    # print(p[:-5,:])

    plt.show()
    return p
  
  def plan_cartesian_path(self, scale=1):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    # with open(os.path.join('/home/student/Prog/.zenart/data/processed/', 'apple.json'), 'rb') as f:
    #     path = json.load(f)
    # path = self.getClockXY(r = 0.20)
    path = self.getTraj("RI_logo2.dot", x_scale = 0.2, y_scale = 0.2)



    sf = 0.1
    print(len(path))
    deltas = []
    # deltas.append(np.array([0, 0, 0]))
    # for p in path:
    #     p = np.array(p)
    #     print(p.shape)
    #     p = p/8
    #     mask = (p[:,0]<1) & (p[:,1]<1)
    #     p = p[mask][::4]
    #     print(p)
    #     if (len(p)>0):
    #         deltas.append(p)
    #         # deltas.append(p[0])
    #         # deltas.append(p[-1])
    # deltas = np.array(deltas)
    # # deltas.append(np.array([0, 0, 0]))
    # p = np.vstack(deltas)
    # p[:,2]=0
    # print(p)
    print(path)

    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_cartesian_path
    ##
    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through. If executing  interactively in a
    ## Python shell, set scale = 1.0.
    ##
    waypoints = []

    print("*"*20)

    wpose = move_group.get_current_pose().pose
    for point in path:
        # stroke = np.array(stroke)
        # for point in stroke:
        print(point)


        tpose = copy.deepcopy(wpose)
        tpose.position.x = point[0] + 0.5 
        tpose.position.y = point[1]
        tpose.position.z = point[2] + 0.5
        waypoints.append(tpose)

    print("*"*20)

    # wpose.position.z -= scale * 0.1  # First move up (z)
    # wpose.position.y += scale * 0.2  # and sideways (y)
    # waypoints.append(copy.deepcopy(wpose))

    # wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    # waypoints.append(copy.deepcopy(wpose))

    # wpose.position.y -= scale * 0.1  # Third move sideways (y)
    # waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

    ## END_SUB_TUTORIAL


  def display_trajectory(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    ## BEGIN_SUB_TUTORIAL display_trajectory
    ##
    ## Displaying a Trajectory
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory)

    ## END_SUB_TUTORIAL


  def execute_plan(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL execute_plan
    ##
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    move_group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
    ## END_SUB_TUTORIAL


  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL wait_for_scene_update
    ##
    ## Ensuring Collision Updates Are Received
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## If the Python node dies before publishing a collision object update message, the message
    ## could get lost and the box will not appear. To ensure that the updates are
    ## made, we wait until we see the changes reflected in the
    ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
    ## For the purpose of this tutorial, we call this function after adding,
    ## removing, attaching or detaching an object in the planning scene. We then wait
    ## until the updates have been made or ``timeout`` seconds have passed
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False
    ## END_SUB_TUTORIAL



def main():
  try:
    
    tutorial = MoveGroupPythonIntefaceTutorial()
    print("Planning path")
    cartesian_plan, fraction = tutorial.plan_cartesian_path()
    print("Displaying trajecotyr")
    tutorial.display_trajectory(cartesian_plan)
    
    print(type(cartesian_plan))
    
    print("Executing path")
    tutorial.execute_plan(cartesian_plan)
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

