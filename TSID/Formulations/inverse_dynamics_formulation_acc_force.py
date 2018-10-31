import pinocchio as se3
import numpy as np
from inverse_dynamics_formulation_base import *
from ..Math import *
from ..Solvers import ConstraintLevel, HQPData

class TaskLevel(object):
    def __init__(self, task, priority):
        self.m_priority = priority
        self.m_task = task
        self.m_constraint = []

    def setConstraint(self, const):
        self.m_constraint = const

    def getConstraint(self):
        return self.m_constraint

    def getTask(self):
        return self.m_task


class ContactLevel(object):
    def __init__(self, contact):
        self.m_contact = contact

class Invdyn(InvdynBase):
    def __init__(self, name, robot, verbose=False):
        InvdynBase.__init__(self, name, robot, verbose)
        self.m_data = self.m_robot.model
        if self.m_robot.isFloatingBase():
            self.m_baseDynamics = ConstraintEquality("base-dynamcis", 6, self.m_robot.nv)

        self.m_solutionDecoded = False

        self.m_t = 0.0
        self.m_v = robot.nv
        self.m_k = 0
        if self.m_robot.isFloatingBase():
            self.m_eq = 6
        else:
            self.m_eq = 0
        self.m_in = 0
        self.m_Jc = np.matrix(np.zeros((self.m_k, self.m_v)))
        self.m_HqpData = HQPData()
        self.m_HqpData.resize(3)

        if self.m_robot.isFloatingBase():
            self.m_HqpData.setData(0, 1.0, self.m_baseDynamics)

        self.m_taskMotions = []

    def data(self):
        return self.m_data

    def nVar(self):
        return self.m_v + self.m_k

    def nEq(self):
        return self.m_eq

    def nIn(self):
        return self.m_in

    def addMotionTask(self, task, weight, priority, transition_time = 0.0):
        assert weight > 0.0
        assert transition_time >= 0.0

        tl = TaskLevel(task, priority)
        self.m_taskMotions.append(tl)
        self.addTask(tl, weight, priority)

        return True

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
        self.m_t = time
        se3.computeAllTerms(self.m_robot.model, self.m_robot.data, q, v)

        if self.m_robot.isFloatingBase():
            M_a = self.m_robot.mass(q)
        else:
            M_a = self.m_robot.mass(q)
            h_a = self.m_robot.bias(q, v)
            J_a = self.m_Jc

            for i in range(0, len(self.m_taskMotions)):
                c = self.m_taskMotions[i].getTask().compute(time, q, v)
                if c.isEquality():
                    self.m_taskMotions[i].getConstraint().setMatrix(c.matrix())
                    self.m_taskMotions[i].getConstraint().setVector(c.vector())
                elif c.isInequality():
                    self.m_taskMotions[i].getConstraint().setMatrix(c.matrix())
                    self.m_taskMotions[i].getConstraint().setLowerBound(c.lowerBound())
                    self.m_taskMotions[i].getConstraint().setUpperBound(c.upperBound())
                else:
                    self.m_taskMotions[i].getConstraint().setLowerBound(c.lowerBound())
                    self.m_taskMotions[i].getConstraint().setLowerBound(c.lowerBound())

        self.m_solutionDecoded = False

        return self.m_HqpData

    def getActuatorForces(self, sol):
        return 0

    def getAccelerations(self, sol):
        return 0

    def getContactForces(self, sol):
        return 0

    def addTask(self, tl, weight, priority):
        if priority > self.m_HqpData.size():
            self.m_HqpData.resize(priority-self.m_HqpData.size())
        c = tl.getTask().getConstraint()

        if c.isEquality():
            tl.setConstraint(ConstraintEquality(c.name, c.rows(), self.m_v + self.m_k))
            if priority == 0 and self.m_robot.isFloatingBase():
                self.m_eq += c.rows()
        elif c.isInequality():
            tl.setConstraint(ConstraintInequality(c.name, c.rows(), self.m_v + self.m_k))
            if priority == 0 and self.m_robot.isFloatingBase():
                self.m_in += c.rows()
        else:
            tl.setConstraint(ConstraintBound(c.name, self.m_v + self.m_k))

        self.m_HqpData.setData(priority, weight, tl.getConstraint())

        return 0

    def resizeHqpData(self):
        self.Jc = np.matrix(np.zeros((self.m_k, self.m_v)))
        if self.m_robot.isFloatingBase():
            self.m_baseDynamics = ConstraintEquality("base-dynamcis", 6, self.m_v + self.m_k)

        return 0

    def removeFromHqpData(self, name):
        return 0

    def decodeSolution(self, sol):
        return 0


