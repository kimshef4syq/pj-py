'''!@example UseKineManipulability.py
@brief 
@date 2021-08-19
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''

import sys

import numpy as np
from IPython import embed
from RVBUST.RPS import *

logger = Logger.GetConsoleLogger("RVS_CL")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        logger.Warn(
            "need arguments, 0 - CalcManipulability, 1 - ShowManipulability")
        sys.exit(0)

    robot = RobotModel()
    robot.InitFromRVDF(
        GetDataPath() + "Multibody/RobotModels/ABB/IRB1200_5_90/IRB1200_5_90.rvdf")
    robot.SetActiveManipulator("Arm")
    manip = robot.GetManipulator("Arm")
    kin_solver = CreateKinSolver(manip)

    manipulability = Manipulability()
    manipulability.Init(manip, kin_solver)
    manipulability.SetSampleDelta(0.04, 0.5)

    logger.Info("The name of data file is: {}".format(
        manipulability.m_file_name))

    method = int(sys.argv[1])
    if method == 0:
        manipulability.GenerateManipulabilityData(
            sample_type=0)  # sample posture in Rotation
        # manipulability.GenerateManipulabilityData(sample_type=1,axis=[0,0,-1])    ## sample posture in Rotation
        # manipulability.GenerateManipulabilityData(sample_type=2,posture=[0,0,0,1])    ## sample posture in fixed posture
        manipulability.SaveHDF5()
    else:
        manipulability.LoadHDF5()
        # show reachability in plane with point p(0,0,0) and normal vector n(1,0,0)
        manipulability.ShowManipulabilityData(
            show_type=0, plane=[0, 0, 0, 1, 0, 0])
        # show reachability in plane with point p(0,0,0) and normal vector n(1,1,1)
        manipulability.ShowManipulabilityData(
            show_type=0, plane=[0, 0, 0, 1, 1, 1])
        # manipulability.ShowManipulabilityData(show_type=0)    ## show the whole workspace
        # manipulability.ShowManipulabilityData(show_type=1)    ## show by Mayavi tool
        embed()
        manipulability.HideManipulabilityViewer()
        embed()
        manipulability.ReshowManipulabilityViewer()
        embed()
        manipulability.DeleteManipulabilityViewer()
        embed()
