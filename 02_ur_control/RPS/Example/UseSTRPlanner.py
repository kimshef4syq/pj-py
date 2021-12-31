'''!@example UseSTRPlanner.py
@brief 
@date 2021-08-19
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''

from IPython import embed
from RVBUST.RPS import *

robot_model = RobotModel()

robot_model.InitFromRVDF(
    GetDataPath() + "Multibody/RobotModels/UniversalRobots/UR5/UR5.rvdf"
)

Twb = Pose([0, -0.15, 0, 0, 0, 0, 1])
robot_model.SetBaseTransformation(Twb)
rvis = RobotVis()
view = rvis.GetView()
view.Home()
view.SetCameraPose(
    [-3.2326595783233643, -2.957855701446533, 2.434558391571045],
    [-2.4848639965057373, -2.4226741790771484, 2.0416433811187744],
    [0.4349738359451294, 0.052190396934747696, 0.8989292979240417])

Twc = Pose(
    np.load(
        GetDataPath() + "Projects/PolishingProject/Cam2robot.npy"))
wash = Multibody()
wash.InitFromMeshFile(
    GetDataPath() + "Projects/PolishingProject/Wash.stl")
wash.SetBaseTransformation(Twc)
rvis.AddBody(wash)

rvis.PlotFrame(Twc)
rvis.AddBody(robot_model)

home = Rx([-1.48524, -1.80172, 2.00258, -1.48595, 4.64519, -4.79932])
robot_model.SetJointPositions(home)

manip = robot_model.GetActiveManipulator()
controller = SimController.Create(manip)

controller.Connect()
controller.EnableRobot()

tool = EndEffector()
tool.InitFromRVDF(
    GetDataPath() + "Multibody/EndEffectors/MillingTool/MillingTool.rvdf"
)
rvis.AddBody(tool)
manip.SetActiveEndEffector(tool)
att = Pose([-1.11022e-16, 0, 0, 0, 0.707107, -1.38778e-17, 0.707107])
tool.SetAttachingPose(att)
ett = Pose([0.228, 0.157, 0, 0, 0, 0, 1]) + SE3Tangent(
    [0, 0, 0, 0, -np.pi / 2, 0])

manip.SetTCP(ett)

kin_solver = controller.GetKinSolver()
joint_limits = ConvertJointLimits2Arr(kin_solver.GetJointLimits())
joint_limits[1] = [20] * 6
joint_limits[2] = [100] * 6

wlist = np.loadtxt(
    GetDataPath() + "Projects/PolishingProject/Waypoints2.txt")
wlist_base = [Twb.Inverse() * Pose(w) for w in wlist]

actions = np.linspace(-0.2, 0.2, 9) * np.pi
actions = np.append(actions, 0)

embed()

print("planning using DWAPlanner")
planner = STRPlanner(rvis.m_env, manip)
planner.Initialize(wlist_base, home, list(actions), 100,
                   False, 40, InitStateType_Distance)

t1 = time.time()
planner.Plan()
print("planning cost time ", time.time() - t1)
qlist = planner.Getqlist()

embed()
limits = manip.GetDoFLimits()
max_vels, max_accs, max_jerks = ConvertJointLimits2Arr(limits)
traj = CreateTrajectory(CreatePath(qlist, 0.1, PathType_Bezier5thBlend),
                        0.5,
                        acc_max=np.linalg.norm(max_accs),
                        jerk_max=np.linalg.norm(max_jerks))

controller.MoveJoints(traj.GetPosition(0))
time.sleep(0.5)
controller.ExecuteTrajectory(traj)
time.sleep(0.5)
controller.MoveJoints(home)

embed()
