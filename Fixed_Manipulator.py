import TSID as tsid

import pinocchio as se3
from os.path import join
import os
import numpy as np
import copy

USE_VIEWER = True

# get Robot model
filename = str(os.path.dirname(os.path.abspath(__file__))) + '/models'
os.environ['ROS_PACKAGE_PATH'] = filename
path = filename + '/talos_data/'
urdf = path + 'robots/talos_left_arm_nogripper.urdf'
robot = tsid.RobotWrapper(urdf, path)

if USE_VIEWER:
    import gepetto.corbaserver
    from pinocchio.utils import *
    import time
    import commands
    l = commands.getstatusoutput("ps aux |grep 'gepetto-gui'| grep -v 'grep' | wc -l")

    if int(l[1]) == 0:
        os.system('gepetto-gui &')

    time. sleep(1)
    cl =gepetto.corbaserver.Client()
    gui = cl.gui
    robot.initDisplay(loadModel=True)
    robot.display(robot.q0)

invdyn = tsid.Invdyn("invdyn", robot)

q = np.matrix([-0.5, 1.517, -1.8, 0, 0, -0.5]).transpose()
if USE_VIEWER:
    robot.display(q)

# Task for joint
task_joint = tsid.Tasks.TaskJointPosture("task-joint", robot)
Kp = 400 * np.matrix(np.ones(len(robot.q0))).transpose()
Kv = 40 * np.matrix(np.ones(len(robot.q0))).transpose()
task_joint.setKp(Kp)
task_joint.setKv(Kv)
q_des = np.matrix(np.random.randn(len(robot.q0))).transpose()
q_des = robot.q0
traj_joint = tsid.Trajectories.TrajectoryEuclidianConst("traj_joint", q_des)
sample_joint = tsid.Trajectories.TrajectorySample(0)
task_joint.setMask(m = np.matrix([1.0, 0, 0, 0, 0, 0], np.int).transpose())
invdyn.addMotionTask(task_joint, 1.0, 1)

task_joint2 = tsid.Tasks.TaskJointPosture("task-joint", robot)
Kp = 400 * np.matrix(np.ones(len(robot.q0))).transpose()
Kv = 40 * np.matrix(np.ones(len(robot.q0))).transpose()
task_joint2.setKp(Kp)
task_joint2.setKv(Kv)
task_joint2.setMask(m = np.matrix([0, 1, 1, 1, 1, 1], np.int).transpose())
invdyn.addMotionTask(task_joint2, 1.0, 2)

# Task for End-Effector in SE3
task_EE = tsid.Tasks.TaskSE3Equality("task-se3", robot, "arm_left_7_link")
Kp = 400 * np.matrix(np.ones(6)).transpose()
Kv = 40 * np.matrix(np.ones(6)).transpose()
task_EE.setKp(Kp)
task_EE.setKv(Kv)
task_EE.setPositionControl(True)
M_des = robot.framePosition(q, robot.model.getFrameId("arm_left_7_link"))

M_des.translation = copy.deepcopy(np.matrix([0.1, 0.0, 0.0]).transpose() + M_des.translation)
traj_EE = tsid.Trajectories.TrajectorySE3Constant("traj_se3", M_des)
sample_EE = tsid.Trajectories.TrajectorySample(0)
invdyn.addMotionTask(task_EE, 1.0, 0)

max_it = 1000
max_tol = 1e-5
t = 0.0

v = robot.v0
dt = 0.001

solver = tsid.HQPSolver("solver")
error = 100

for i in range(0, max_it+1):
    sample_joint = traj_joint.computeNext()
    task_joint.setReference(sample_joint)
    task_joint2.setReference(sample_joint)
    sample_EE = traj_EE.computeNext()
    task_EE.setReference(sample_EE)
    HQPData = invdyn.computeProblemData(t, q, v)
    if i == 0:
        print ""
        tsid.Solvers.utils.printHQPData(HQPData)
        print ""
    x_sol = solver.solve(HQPData)

    v += dt * invdyn.getAccelerations(x_sol)
    q = se3.integrate(robot.model, q, dt*v)

    error_EE = np.linalg.norm(task_EE.position_error()[0:3],2)
    if i % 100 == 0:
        if USE_VIEWER:
            robot.display(q)
        print "Time", t, "Pos error", error_EE
    #print q.transpose(), "q"
    t += dt

#if error_EE < max_tol:
print ""
print "Success Convergence"
print "Final Destination", robot.framePosition(q, robot.model.getFrameId("arm_left_7_link"))
print "Norm", np.linalg.norm(q-q_des, 2)
print ""

