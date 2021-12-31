'''!@example UseTrajWithDO.py
@brief Demo of setting digital output while still executing trajectory.
@date 2021-08-19
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''

import sys
import time
import numpy as np
from RVBUST.RPS import *

print(sys.argv)

rvis = RobotVis()
r: GenericRobotController = SimController.Create("Motoman_GP12")
rvis.AddBody(r.GetRobotModel())
kin_solver: GenericKinematics = r.GetKinSolver()
jls = ConvertJointLimits2Arr(kin_solver.GetJointLimits())
r.Connect("192.168.10.120")
r.EnableRobot()

points = [
    [0., 0., 0., 0., 0., 0.],
    [np.pi/2, -0.5, -0.5, 0., 0., 0.],
    [-np.pi/2, 0.5, -0.5, 1., 0., 0.],
    [0., 0., 0., 0., 0., 0.],
]

path = CreatePath(points, 0.1, PathType_Bezier5thBlend)
traj = CreateTrajectory(path, *jls, TrajType_DoubleS)

# Assume DO 1,2,3,4 is set True between some waypoints
t1 = traj.GetNearestTimeStamp(Rx(points[1]))
s1 = traj.GetPathParameter()(t1)
traj.AddDOInfo(s1, 1, True)
traj.AddDOInfo(s1, 2, True)
traj.AddDOInfo(s1, 3, True)
traj.AddDOInfo(s1, 4, True)
t2 = traj.GetNearestTimeStamp(Rx(points[-2]))
traj.AddDOInfoWithTimeFlag(t2, 1, False)
traj.AddDOInfoWithTimeFlag(t2, 2, False)
traj.AddDOInfoWithTimeFlag(t2, 3, False)
traj.AddDOInfoWithTimeFlag(t2, 4, False)
print(f"Set DO [1,2,3,4] ON during {t1} and {t2}")

hs = DrawTraj(traj, kin_solver=kin_solver, show_in_plt=False,
              show_in_vis=True, view=rvis.GetView())
r.MoveJoints(traj.GetPosition(0))
r.ExecuteTrajectory(traj, False)

do_vals = []
t0 = time.time()
while r.GetControllerState().IsMoving():
    do_val = [time.time() - t0]
    for i in range(1, 5):
        do_val.append(r.GetDigitalOutput(i)[1])
    do_vals.append(do_val)
    time.sleep(0.01)
do_vals = np.asarray(do_vals)
plt.plot(do_vals[:, 0], do_vals[:, 1:])
plt.show()

r.Disconnect()
