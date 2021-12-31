'''!@example UseKineReachability.py
@brief 
@date 2021-08-19
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''

import sys

import numpy as np
from IPython import embed
from RVBUST.RPS import *

logger = Logger.GetConsoleLogger("RVS_CL")


def InitRobot():
    robot = RobotModel()
    robot.InitFromRVDF(
        GetDataPath() + "Multibody/RobotModels/ABB/IRB1200_5_90/IRB1200_5_90.rvdf")
    # robot.InitFromRVDF(GetDataPath() + "Multibody/RobotModels/Motoman/GP12/GP12.rvdf")
    # robot.InitFromRVDF(GetDataPath() + "Multibody/RobotModels/Motoman/MotoMini/MotoMini.rvdf")
    # robot.InitFromRVDF(GetDataPath() + "Multibody/RobotModels/Epson/LS6_502S/LS6_502S.rvdf")

    manip = robot.GetActiveManipulator()
    kin_solver = CreateKinSolver(manip)
    # kin_solver = ScaraKinematics(manip)
    # kin_solver = GenericKinematics(manip)
    return robot, kin_solver


if __name__ == "__main__":
    if len(sys.argv) != 2:
        logger.Warn("need arguments, 0 - CalcWorkspace, 1 - ShowWorkspace")
        sys.exit(0)

    robot, kin_solver = InitRobot()
    reachobj = Reachability()
    reachobj.Init(robot, kin_solver)

    # reachobj.SetFileName(GetDataPath() + "Database/RobotWorksapce/ABBworkspace.h5")
    logger.Info("The name of data file is: {}".format(reachobj.m_file_name))

    method = int(sys.argv[1])
    if method == 0:
        reachobj.SetSampleDelta(0.04, 0.5)

        reachobj.GenerateReachData(sample_type=0)  # sample posture in Rotation
        # reachobj.GenerateReachData(sample_type=1,axis=[0,0,-1])    ## sample posture in Rotation
        # reachobj.GenerateReachData(sample_type=2,posture=[0,0,0,1])    ## sample posture in fixed posture
        reachobj.SaveHDF5()
    else:

        reachobj.LoadHDF5()
        # show reachability in plane with point p(0,0,0) and normal vector n(1,0,0)
        reachobj.ShowReachData(show_type=0, plane=[0, 0, 0, 1, 0, 0])
        # show reachability in plane with point p(0,0,0) and normal vector n(1,1,1)
        reachobj.ShowReachData(show_type=0, plane=[0, 0, 0, 1, 1, 1])
        # reachobj.ShowReachData(show_type=0)    ## show the whole workspace
        # reachobj.ShowReachData(show_type=1)    ## show by Mayavi tool
        embed()
        reachobj.HideReachViewer()
        embed()
        reachobj.ReshowReachViewer()
        embed()
        reachobj.DeleteReachViewer()
        embed()
