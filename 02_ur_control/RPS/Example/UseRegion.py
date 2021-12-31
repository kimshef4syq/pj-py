'''!@example UseRegion.py
@brief 
@date 2021-08-19
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''

from IPython import embed
from RVBUST.RPS import *
logger = Logger.GetConsoleLogger('logger')

region_se3 = RegionSE3(origin=Pose(), lower_bound=SE3Tangent(
    [0, 0, 0, 0, -1.5, -1.5]), upper_bound=SE3Tangent([0, 0, 0, 0, 1, 1]))


logger.Info(f"region_se3, origin: {region_se3.GetOrigin()}")
for pose in region_se3.SampleUniformInRegion([0, 0, 0, 0, 2, 4]):
    logger.Info(f"Sampled pose: {pose}")
    if region_se3.IsInRegion(pose):
        logger.Info("is in region")

pose1 = Pose() + SE3Tangent([0, 0, 0, 0, 2, 1])
logger.Info(f"pose: {pose1}, is in region: {region_se3.IsInRegion(pose1)}")

t = region_se3.GetDistance(pose1)
logger.Info(f"distance to the region: {t}")

pose2 = pose1 + t
logger.Info(
    f"moved the distance toward the region, now is in region: {region_se3.IsInRegion(pose2)}")


""" Region Rx """
region_rx = RegionRx(Rx([1, 1]), RxTangent([1, 1]), RxTangent([2, 2]))
logger.Info(f"region_rx, origin: {region_rx.GetOrigin()}")
for position in region_rx.SampleUniformInRegion([3, 3]):
    logger.Info(f"Sampled position: {position}")
    if region_rx.IsInRegion(position):
        logger.Info("is in region")

position1 = Rx([-4, 2])
logger.Info(
    f"position: {position1}, is in region: {region_rx.IsInRegion(position1)}")

t = region_rx.GetDistance(position1)
logger.Info(f"distance to the region: {t}")

position2 = position1 + t
logger.Info(
    f"moved the distance toward the region, now is in region: {region_rx.IsInRegion(position2)}")


cart_region = CartesianRegion(origin=Pose([0.5, 0, 0.5, 0, 0, 0, 1]), lower_bound=SE3Tangent(
    [0, 0, 0, 0, 0, 0]), upper_bound=SE3Tangent([0, 0, 0, np.pi, 0, 0]), num_samples=5)


logger.Info(f"Cartesian Region, origin: {cart_region.GetOrigin()}")
cart_region_discrete = cart_region.Discretize()
poses = cart_region_discrete.GetCartesianPoses()
for pose in poses:
    logger.Info(f"discretized pose: {pose}")

robot_model = CreateRobotModel("Motoman_GP7")
kin_solver = CreateKinSolver(robot_model.GetActiveManipulator())

joint_positions = cart_region.GetJointPositions(kin_solver=kin_solver)

for q in joint_positions:
    logger.Info(f"discretized joint positions: {q}")


embed()
