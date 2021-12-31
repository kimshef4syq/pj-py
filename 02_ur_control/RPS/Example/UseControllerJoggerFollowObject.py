'''!@example UseControllerJoggerFollowObject.py
@brief An example about using controller jogger to follow a box.
@date 2021-08-19
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''

import os
import time
import numpy as np
from threading import Thread

from IPython import embed
from RVBUST.RPS import *
Logger.SetLevelForAll(3)


def UpdateTarget(r, jogger, view, h):
    global following_object
    kin_solver: GenericKinematics = r.GetKinSolver()
    h_box_axes = None
    while following_object:
        ret, position = view.GetPosition(h)
        ret, rotation = view.GetRotation(h)
        pose = Pose(R3(position), Rotation(rotation))
        if h_box_axes:
            view.Delete(h_box_axes)
        h_box_axes = view.Axes(position, rotation, axis_len=0.1)
        q_seed = r.GetJointPosition()[1]
        ret, ik_ret, dist = kin_solver.GetNearestIK(pose, q_seed)
        if ik_ret.success:
            jogger.UpdateTarget(ik_ret[0])
        time.sleep(0.1)
    print(f"exit following object thread")


r: GenericRobotController = SimController.Create("ABB_IRB1200_5_90")
r.Connect("192.168.10.122")
r.EnableRobot()

rvis = RobotVis()
rvis.AddBody(r)

jogger = ControllerJogger(r, 1.0)
jogger.UpdateSpeedRatio(0.1)
jogger.StartJog()

# 跟踪Vis中的一个物体，手动拖动物体，机器人末端会自动跟踪
r.SetTCP(Pose([0, 0, 0.1, 0, 0, 0, 1]))
position = r.GetPose()[1].Coeffs()[:3]
view = rvis.GetView()
h = view.Box(position, [0.05, 0.05, 0.05])
view.SetRotation(h, [0, 1, 0, 0])

following_object = True

thrd = Thread(target=UpdateTarget, args=(r, jogger, view, h), daemon=False)
thrd.start()
view.EnableGizmo(h, 4)
view.SetGizmoDisplayScale(0.2)
# now you can drag the box in viewer
embed()
following_object = False
view.DisableGizmo()
jogger.StopJog()
