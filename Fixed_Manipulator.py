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
urdf = path + 'robots/talos_left_arm.urdf'
robot = tsid.RobotWrapper(urdf, path)

if USE_VIEWER:
    import gepetto.corbaserver
    from pinocchio.utils import *
    import time

    #os.system('gepetto-gui &')
    time. sleep(1)
    cl =gepetto.corbaserver.Client()
    gui = cl.gui
    robot.initDisplay(loadModel=True)
    robot.display(robot.q0)

invdyn = tsid.Invdyn("invdyn", robot)
task_joint = tsid.Tasks.TaskJointPosture("task-joint", robot)

Kp = 400 * np.matrix(np.ones(len(robot.q0))).transpose()
Kv = 40 * np.matrix(np.ones(len(robot.q0))).transpose()

task_joint.setKp(Kp)
task_joint.setKv(Kv)

q_des = np.matrix(np.random.randn(len(robot.q0))).transpose()
traj = tsid.Trajectories.TrajectoryEuclidianConst("traj_joint", q_des)
sample = tsid.Trajectories.TrajectorySample(0)

invdyn.addMotionTask(task_joint, 1.0, 0)

max_it = 1000
t = 0.0
q = robot.q0
v = robot.v0
dt = 0.001

solver = tsid.HQPSolver("solver")

for i in range(0, max_it):

    sample = traj.computeNext()
    task_joint.setReference(sample)
    HQPData = invdyn.computeProblemData(t, q, v)
    if i == 0:
        tsid.Solvers.utils.printHQPData(HQPData)

    x_sol = solver.solve(HQPData)

    v += dt * np.matrix(x_sol[0:len(robot.q0)]).transpose()
    q = se3.integrate(robot.model, q, dt*v)
    t += dt

    if i % 100 == 0 and USE_VIEWER:
        robot.display(q)


print q_des
print q
