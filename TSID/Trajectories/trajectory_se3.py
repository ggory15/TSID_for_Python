import numpy as np
from pinocchio import SE3
from trajectory_base import *
import copy
class TrajectorySE3Constant(TrajectoryBase):
    def __init__(self, name, M):
        self.name = name
        self.m_sample = TrajectorySample(12, 6)

        pos_tmp = np.matrix(np.zeros(12)).transpose()
        pos_tmp[0:3] = copy.deepcopy(M.translation)
        for i in range(1, 4):
            pos_tmp[3*i : 3+3*i] = copy.deepcopy(M.rotation[0:3, i-1])

        self.m_sample.setPos(pos_tmp)

    def size(self):
        return 6

    def computeNext(self):
        return self.m_sample

    def getLastSample(self):
        return self.m_sample

    def has_trajectory_ended(self):
        return True