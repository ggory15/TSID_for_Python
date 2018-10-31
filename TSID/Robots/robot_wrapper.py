from pinocchio.robot_wrapper import RobotWrapper as PinocchioRobotWrapper
import pinocchio as se3
import numpy as np

class RobotWrapper(PinocchioRobotWrapper):
    def com_all(self):
        com_tmp = [];
        com_tmp.append(self.data.com[0])
        com_tmp.append(self.data.vcom[0])
        com_tmp.append(self.data.acom[0])
        return com_tmp

    def isFloatingBase(self):
        if self.nv == self.nq:
            return False
        else:
            return True

    def frameClassicAcceleration(self, index):
        f = self.model.frames[index]
        a = f.placement.actInv(self.data.a[f.parent])
        v = f.placement.actInv(self.data.v[f.parent])
        a.linear += np.cross(v.angular.T, v.linear.T).T
        return a;
# if needed, we can put some def which is related on Robot Wrapper
