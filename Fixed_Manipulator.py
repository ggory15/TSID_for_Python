import TSID as tsid

import pinocchio as se3
from os.path import join
import os
import numpy as np

# get Robot model
filename = str(os.path.dirname(os.path.abspath(__file__)))
os.environ['ROS_PACKAGE_PATH'] = filename
path = filename + '/models/'
urdf = path + 'ur_description/urdf/ur5_gripper.urdf'
robot = tsid.RobotWrapper(urdf, [path, ])

invdyn = tsid.Invdyn("invdyn", robot)
