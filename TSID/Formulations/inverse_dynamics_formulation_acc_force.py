import pinocchio as se3
import numpy as np
from inverse_dynamics_formulation_base import *

class TaskLevel(object):
    def __init__(self, task, priority):
        self.m_priority = priority
        self.m_task = task


class ContactLevel(object):
    def __init__(self, contact):
        self.m_contact = contact

class Invdyn(InvdynBase):
    def __init__(self, name, robot, verbose=False):
        InvdynBase.__init__(self, name, robot, verbose)


        self.m_t = 0.0
        self.m_v = robot.nv
        self.m_k = 1
        if robot.nv == robot.nq :
            self.m_eq = 0
        elif robot.nv - 6 == robot.nq:
            self.m_eq = 6
        else:
            print ("Root Type error")

        self.m_in = 0
        self.robot = robot
        if self.m_k == 0:
            self.Jc = []
        else:
            self.Jc = np.matrix(np.zeros((self.m_k, self.m_v)))

    def data(self):
        return 0

    def nVar(self):
        return 0

    def nEq(self):
        return 0

    def nIn(self):
        return 0

    def addMotionTask(self, task, weight, priority, transition_time = 0.0):
        return 0

    def addForceTask(self, task, weight, priority, transition_time = 0.0):
        return 0

    def addTorqueTask(self, task, weight, priority, transition_time = 0.0):
        return 0

    def updateTaskWeight(self, task_name, weight):
        return 0

    def addRigidContact(self, contact):
        return 0

    def removeTask(self, taskName, transition_time=0.0):
        return 0

    def removeRigidContact(self, contactName, transition_time= 0.0):
        return 0

    def computeProblemData(self, time, q, v):
        return 0

    def getActuatorForces(self, sol):
        return 0

    def getActuatorForces(self, sol):
        return 0

    def getAccelerations(self, sol):
        return 0

    def getContactForces(self, sol):
        return 0

    def getContactForce(self, name, sol, f):
        return 0

