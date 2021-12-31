'''!@example UseDragging.py
@brief Dragging objects in RobotVis.
@date 2021-08-19
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''

from IPython import embed
from RVBUST import RPS

robot_model = RPS.RobotModel()
robot_model.InitFromRVDF(
    RPS.GetDataPath() + "Multibody/RobotModels/ABB/IRB1200_5_90/IRB1200_5_90.rvdf")
eef = RPS.EndEffector()
eef.InitFromRVDF(RPS.GetDataPath(
) + "Multibody/EndEffectors/Grippers/TwoFingerGripper/TwoFingerGripper.rvdf")
box = RPS.Multibody()
box.InitFromBox(0.3, 0.2, 0.3)

rvis = RPS.RobotVis()
rvis.AddBody(robot_model)
rvis.AddBody(eef)
rvis.AddBody(box)

manipulator = robot_model.GetActiveManipulator()

rvis.GetView().Home()

embed()

rvis.StopDragging()
rvis.StartDragging(robot_model)

embed()

rvis.StopDragging()
rvis.StartDragging(manipulator)

embed()

rvis.StopDragging()
rvis.StartDragging(box)

embed()

rvis.StopDragging()
rvis.StartDragging(eef)

embed()

rvis.StopDragging()
eef.SetAttachingPose(RPS.Pose.IdentityStatic())
manipulator.SetActiveEndEffector(eef)
rvis.StartDragging(manipulator)

embed()
