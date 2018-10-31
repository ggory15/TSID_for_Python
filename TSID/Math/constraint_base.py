import numpy as np
import copy

class ConstraintBase(object):
    def __init__(self, *args):
        if len(args) == 1:
            self.name = args[0]
        elif len(args) == 2:
            self.name = args[0]
            self.m_A = args[1]
        elif len(args) == 3:
            self.name = args[0]
            self.m_A = np.matrix(np.zeros((args[1], args[2])))

        assert len(args) < 4

    def name(self):
        return self.name

    def matrix(self):
        return self.m_A

    def setMatrix(self, A):
        self.m_A = A
        return True
