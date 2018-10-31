import numpy as np
import copy
from constraint_base import ConstraintBase

class ConstraintInequality(ConstraintBase):
    def __init__(self, *args):
        if len(args) == 1:
            ConstraintBase.__init__(self, args[0])
        elif len(args) == 2:
            assert len(args) == 2
        elif len(args) == 3:
            if isinstance(args[1], int) and isinstance(args[2], int):
                ConstraintBase.__init__(self, args[0], args[1], args[2])
                self.m_lb = np.matrix(np.zeros(args[1])).transpose()
                self.m_ub = np.matrix(np.zeros(args[1])).transpose()
            else:
                assert False
        elif len(args) == 4:
            if isinstance(args[1], np.ndarray) and isinstance(args[2], np.ndarray) and isinstance(args[3], np.ndarray):
                ConstraintBase.__init__(self, args[0], args[1])
                self.m_lb = args[2]
                self.m_lb = args[3]
                assert len(self.m_A) == len(self.m_lb)
                assert len(self.m_A) == len(self.m_ub)

        assert len(args) < 5

    def rows(self):
        assert len(self.m_A) == len(self.m_lb)
        assert len(self.m_A) == len(self.m_ub)
        return len(self.m_A)

    def cols(self):
        assert self.m_A.size

    def resize(self, r, c):
        self.m_A = np.matrix(np.zeros((r, c)))
        self.m_lb = np.matrix(np.zeros(r)).transpose()
        self.m_ub = np.matrix(np.zeros(r)).transpose()

    def isEquality(self):
        return False

    def isInequality(self):
        return True

    def isBound(self):
        return False

    def vector(self):
        assert False

    def lowerBound(self):
        return self.m_lb

    def upperBound(self):
        return self.m_ub

    def setVector(self, b):
        assert False

    def setLowerBound(self, lb):
        self.m_lb = copy.deepcopy(lb)

    def setUpperBound(self, ub):
        self.m_ub = copy.deepcopy(ub)
