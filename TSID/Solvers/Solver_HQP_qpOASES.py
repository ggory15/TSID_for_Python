import qpoases
import numpy as np
import copy
#import HQPData
from fwd import HQPData

class HQPSolver(object):
    def __init__(self, name):
        self.m_name = name
        self.m_hessian_regularization = 1e-8
        self.m_n = 0
        self.m_neq = 0
        self.m_nin = 0
        self.m_bound = 0

    def solve(self, HQPdata):
        assert HQPdata.data() > 0
        problemData = self.SqueezeData(HQPdata)
        cl0 = problemData[0]
        n = cl0[0][1].cols()

        neq = np.matrix(np.zeros(len(problemData))).transpose()
        nin = np.matrix(np.zeros(len(problemData))).transpose()
        nbound = np.matrix(np.zeros(len(problemData))).transpose()

        for i in range(0, len(problemData)):
            cl = problemData[i]
            for j in range(0, len(cl)):
                assert n == cl[j][1].cols()
                if cl[j][1].isEquality():
                    neq[j] += cl[j][1].rows()
                elif cl[j][1].isInequality():
                    nin[j] += cl[j][1].rows()
                else:
                    nbound[j] += cl[j][1].rows()

        self.resize(n, neq, nin, nbound)

        i_eq = 0
        i_in = 0
        i_bound = 0
        for i in range(0, len(problemData)):
            cl = problemData[i]
            for j in range(0, len(cl)):
                if cl[j][1].isEquality():
                    self.A[i][i_eq: i_eq + cl[j][1].rows(), 0:n] = copy.deepcopy(cl[j][1].matrix())
                    self.Alb[i][i_eq: i_eq + cl[j][1].rows()] = copy.deepcopy(cl[j][1].vector())
                    self.Aub[i][i_eq: i_eq + cl[j][1].rows()] = copy.deepcopy(cl[j][1].vector())
                    i_eq += cl[j][1].rows()

        for i in range(1, len(problemData)):
            if not len(self.A[i]) == len(self.A[i-1]):
                self.A[i][0:len(self.A[i-1]), ] = copy.deepcopy(self.A[i-1])
                self.Alb[i][0:len(self.Alb[i-1])] = copy.deepcopy(self.Alb[i-1])
                self.Aub[i][0:len(self.Aub[i-1])] = copy.deepcopy(self.Aub[i-1])


        H_tmp = np.matrix(np.eye(n + len(self.A[0]), n + len(self.A[0])))
        #H[len(self.A[0]):, len(self.A[0]):] *= self.m_hessian_regularization / problemData[0][0][0]
        g_tmp = np.array(np.zeros(n + len(self.A[0])))

        n_size = n + len(self.A[0])
        c_size = len(self.A[0])
        lb2 = np.array(100000 * np.ones(n_size))
        nWSR = np.array([100])

        A_tmp = np.matrix(np.zeros((n_size, c_size)))

        solver = qpoases.PySQProblem(n_size, c_size)
        option = qpoases.PyOptions()
        option.printLevel = qpoases.PyPrintLevel.NONE
        solver.setOptions(option)
        solver.init(H_tmp, g_tmp, self.A[0].transpose(), -lb2, lb2, np.squeeze(np.asarray(self.Alb[0])), np.squeeze(np.asarray(self.Alb[0])), nWSR)

        xOpt = np.zeros(n_size)
        solver.getPrimalSolution(xOpt)
        return xOpt

    def resize(self, n, neq, nin, nbound):
        self.A = []
        self.Alb = []
        self.Aub = []
        self.lb = []
        self.ub = []

        self.slack_num = []

        for i in range(0, len(neq)):
            if i == 0:
                slack = 0
            else:
                slack = self.slack_num[i-1]

            if neq[i] + nin[i] > 0:
                self.A.append(np.matrix(np.zeros((int(neq[i] + nin[i] + slack), n + int(neq[i] + nin[i])))))
                self.Alb.append(np.matrix(np.zeros(int(neq[i] + nin[i] + slack))).transpose())
                self.Aub.append(np.matrix(np.zeros(int(neq[i] + nin[i] + slack))).transpose())
            else:
                self.lb.append(np.matrix(np.zeros(int(nbound[i] + slack))).transpose())
                self.ub.append(np.matrix(np.zeros(int(nbound[i] + slack))).transpose())
            self.slack_num.append(int(neq[i] + nin[i] + nbound[i]))

        return True

    def SqueezeData(self, HQPdata):
        j = 0
        for i in range(0, HQPdata.size()):
            if HQPdata.data()[i]:
                j += 1

        ProblemData = HQPData()
        ProblemData.resize(j)

        j = 0
        for i in range(0, HQPdata.size()):
            if HQPdata.data()[i]:
                ProblemData.data()[j] = copy.deepcopy(HQPdata.data()[i])
                j += 1

        return ProblemData.data()