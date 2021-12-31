'''!@example UseRemoteCenter.py
@brief 
@date 2021-08-19
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''

from RVBUST.RPS import *
from IPython import embed

robot_model = RobotModel()
robot_model.InitFromRVDF(
    GetDataPath() + "Multibody/RobotModels/ABB/IRB1200_5_90/IRB1200_5_90.rvdf"
)
rvis = RobotVis()
rvis.AddBody(robot_model)
rvis.GetView().Home()
rvis.GetView().SetCameraPose(
    [1.2939268350601196, -1.922868013381958, 2.11846661567688],
    [0.9186282157897949, -1.2116361856460571, 1.524075984954834],
    [-0.5171252489089966, 0.3715256452560425, 0.7710707783699036])
eef = EndEffector()

embed()

eef.InitFromCylinder(0.004, 0.3)
eef.SetTCP(Pose(0, 0, 0.15, 0, 0, 0, 1))
eef.SetAttachingPose(Pose(0.03, 0, 0, 0, 0, 0, 1))

embed()

manipulator = robot_model.GetActiveManipulator()
rvis.AddBody(eef)
manipulator.SetActiveEndEffector(eef)

embed()

controller = SimController.Create(manipulator)
controller.Connect()
controller.EnableRobot()
kin_solver = controller.GetKinSolver()

target_pose = Pose(0.451, 0, 0.5071, 0, 0.995004, 0, 0.0998334)

h = rvis.PlotFrame(target_pose)

ik_constraint = SE3Tangent(1e-5, 1e-5, 0.1, 0, 1, np.pi)

kin_solver.SetPoseConstraint(ik_constraint)

embed()

for i in range(100):
    q_seed = kin_solver.GetRandomStateNearBy(manipulator.GetDoFPositions(),
                                             0.1)
    res, ik_report = kin_solver.GetPositionIK(target_pose, q_seed)
    if (res == RVSReturn_Success):
        for q in ik_report.ik_vector:
            manipulator.SetDoFPositions(q)
            time.sleep(0.01)

embed()
