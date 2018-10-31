from pinocchio import SE3
import copy
import numpy as np

def vectorToSE3(vec):
    M = SE3()

    rot_tmp = np.matrix(np.ones((3, 3)))
    rot_tmp[0:3, 0] = vec[3:6]
    rot_tmp[0:3, 1] = vec[6:9]
    rot_tmp[0:3, 2] = vec[9:12]

    M.translation = copy.deepcopy(vec[0:3])
    M.rotation= rot_tmp
    return M

def SE3toVector(M):
    ref = np.matrix(np.zeros(12)).transpose()
    ref[0:3] = copy.deepcopy(M.translation)
    ref[3:6] = copy.deepcopy(M.rotation[0:3, 0])
    ref[6:9] = copy.deepcopy(M.rotation[0:3, 1])
    ref[9:12] = copy.deepcopy(M.rotation[0:3, 2])
