import numpy as np
import copy
from constraint_base import ConstraintBase

class ConstraintEquality(ConstraintBase):
    def __init__(self, *args):
        if len(args) == 1:
            ConstraintBase.__init__(self, args[0])
        elif len(args) == 2:
            assert len(args) == 2
        elif len(args) == 3:
            if isinstance(args[1], np.ndarray) and isinstance(args[2], np.ndarray):
                ConstraintBase.__init__(self, args[0], args[1])
                self.m_b = args[2]
                assert len(self.m_A) == len(self.m_b)
            elif isinstance(args[1], int) and isinstance(args[2], int):
                ConstraintBase.__init__(self, args[0], args[1], args[2])
                self.m_b = np.matrix(np.zeros(args[1])).transpose()
            else:
                assert False
        assert len(args) < 4

    def rows(self):
        assert len(self.m_A) == len(self.m_b)
        return len(self.m_A)

    def cols(self):
        assert self.m_A.size

    def resize(self, r, c):
        self.m_A = np.matrix(np.zeros((r, c)))
        self.m_b = np.matrix(np.zeros(r)).transpose()

    def isEquality(self):
        return True

    def isInequality(self):
        return False

    def isBound(self):
        return False

    def vector(self):
        return self.m_b

    def lowerBound(self):
        assert False

    def upperBound(self):
        assert False

    def setVector(self, b):
        self.m_b = copy.deepcopy(b)

    def setLowerBound(self, lb):
        assert False

    def setUpperBound(self, ub):
        assert False
