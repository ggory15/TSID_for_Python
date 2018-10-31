import TSID as tsid

import pinocchio as se3
from os.path import join
import os
import numpy as np
import copy
# get Robot model
filename = str(os.path.dirname(os.path.abspath(__file__)))
os.environ['ROS_PACKAGE_PATH'] = filename
path = filename + '/../models/'
urdf = path + 'ur_description/urdf/ur5_gripper.urdf'

robot = tsid.RobotWrapper(urdf, [path, ])

print "Test for SE3 Task"
task = tsid.Tasks.TaskSE3Equality("task-se3", robot, "ee_link")
Kp = 25 * np.matrix(np.ones(6)).transpose()
Kv = 10 * np.matrix(np.ones(6)).transpose()

task.setKp(Kp)
task.setKv(Kv)

M_ref = se3.SE3()
M_ref.translation = copy.deepcopy(np.matrix([0.81725,   0.19145, -0.005491]).transpose())
rot_tmp = np.matrix(np.zeros((3,3)))
rot_tmp[0:3, 0] = np.matrix([0, 1, 0]).transpose()
rot_tmp[0:3, 1] = np.matrix([1, 0, 0]).transpose()
rot_tmp[0:3, 2] = np.matrix([0, 0, -1]).transpose()
M_ref.rotation = rot_tmp

print M_ref, "M_ref"

traj = tsid.Trajectories.TrajectorySE3Constant("traj_se3", M_ref)
sample = tsid.Trajectories.TrajectorySample(0)

t = 0.0
dt = 0.001
error = 0.0
error_past = 1e100
max_tol = 0.00001
q = np.matrix(np.ones(len(robot.q0))).transpose()
v = robot.v0
max_it = 5000

for i in range(0, max_it):
    se3.computeAllTerms(robot.model, robot.data, q, v)
    sample = traj.computeNext()

    task.setReference(sample)
    constraint = task.compute(t, q, v)
    dv = np.linalg.pinv(constraint.matrix(), 1e-4) *constraint.vector()


    v += dt * dv
    q = se3.integrate(robot.model, q, dt*v)
    #print (q)
    t += dt
    error = np.linalg.norm(task.position_error(), 2)

    if (i%100 == 0):
        print "Time", t, "Pos error", error, "Vel error", np.linalg.norm(task.velocity_error(), 2)

    if i == max_it-1:
        print "Final q", q.transpose()
    if error < max_tol:
        print "Success : Max tol is less than threshold"
        print "Final q", q.transpose()
        break;
    assert (error_past > error)

print ""
print ""
print ""


print "Test for Joint Task"
task = tsid.Tasks.TaskJointPosture("task-joint", robot)
task.setKp(Kp)
task.setKv(Kv)

q_des = np.matrix(np.random.randn(len(robot.q0))).transpose()
traj = tsid.Trajectories.TrajectoryEuclidianConst("traj_joint", q_des)
sample = tsid.Trajectories.TrajectorySample(0)

t = 0.0
error = 0.0
error_past = 1e100

for i in range(0, max_it):
    se3.computeAllTerms(robot.model, robot.data, q, v)
    sample = traj.computeNext()
    task.setReference(sample)

    constraint = task.compute(t, q, v)
    dv = np.linalg.pinv(constraint.matrix(), 1e-4) *constraint.vector()

    v += dt * dv
    q = se3.integrate(robot.model, q, dt*v)
    #print (q)
    t += dt
    error = np.linalg.norm(task.position_error(), 2)

    if (i%100 == 0):
        print "Time", t, "Pos error", error, "Vel error", np.linalg.norm(task.velocity_error(), 2)

    if i == max_it-1:
        print "Final q", q.transpose()

    if error < max_tol:
        print "Success : Max tol is less than threshold"
        print "Desried q", q_des.transpose()
        print "Final q", q.transpose()
        break;
    assert (error_past > error)

