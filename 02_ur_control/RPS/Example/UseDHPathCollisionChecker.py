'''!@example UseDHPathCollisionChecker.py
@brief 
@date 2021-08-19
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''

import time
from IPython import embed
from RVBUST.RPS import *

robot_model = RobotModel()
robot_model.InitFromRVDF(
    GetDataPath() + "Multibody/RobotModels/UniversalRobots/UR5/UR5.rvdf")
robot_model.SetBaseTransformation(Pose([0, 0, 0, 0, 0, 0, 1]))
home_position = Rx(
    [-1.48524, -1.80172, 2.00258, -1.48595, 4.64519, -4.79932])
robot_model.SetJointPositions(home_position)

obstacle1 = Multibody()
obstacle1.InitFromBox(0.2, 0.2, 0.2)
obstacle1.SetName("obstacle1")
# orientation_obs = Rotation().RandomStatic()
obstacle1.SetBaseTransformation(Pose(0.43356, -0.144609, 0.419158, 0, 0, 0, 1))

obstacle2 = Multibody()
obstacle2.InitFromBox(0.1, 0.2, 0.25)
obstacle2.SetName("obstacle2")
obstacle2.SetBaseTransformation(Pose(0.2, -0.3, 0.1, 0, 0, 0, 1))

obstacle3 = Multibody()
obstacle3.InitFromBox(0.25, 0.1, 0.2)
obstacle3.SetName("obstacle3")
obstacle3.SetBaseTransformation(Pose(0.4, 0.2, 0.3, 0, 0, 0, 1))

obstacles = [obstacle1, obstacle2, obstacle3]

robot_vis = RobotVis()
env = Environment()
env.AddBody(robot_model)
env.AddBodies(obstacles)
robot_vis.LoadEnvironment(env)


manipulator = robot_model.GetActiveManipulator()
robot_controller = SimController.Create(manipulator)
robot_controller.Connect()
robot_controller.EnableRobot()

start_point = robot_controller.GetJointPosition()[1]

#start_point = manipulator.GetDoFPositions()
end_point = Rx([0.5, -0.5, 0.3, 0.1, 0.1, 0.1])

raw_path = CreatePath([start_point, end_point], 0.01, PathType_NoBlend)
temp_max_vels = np.ones(6)*1
temp_max_accs = np.ones(6)*2
temp_max_jerks = np.ones(6)*2
final_trajectory = CreateTrajectory(
    raw_path, temp_max_vels, temp_max_accs, temp_max_jerks)
robot_controller.ExecuteTrajectory(final_trajectory, True)

embed()
robot_model.SetJointPositions(start_point)
links = robot_model.GetLinks()[:-1]
box_geoms = [link.GetVisualGeometry() for link in links]
bboxes = [box.GetOOBBBoundingBox() for box in box_geoms]
bposes = [link.GetPose() for link in links]
init_poses = [Pose(box.pose) for box in bboxes]
print("start: init", init_poses)
for box, init_p, pose in zip(bboxes, init_poses, bposes):
    box.pose = pose * init_p
mboxes = []
for box in bboxes:
    temp_mul = Multibody()
    temp_mul.InitFromBox(box.length, box.width, box.height)
    temp_mul.SetBaseTransformation(box.pose)
    mboxes.append(temp_mul)
env.Clear()
# env.AddBodies(mboxes)
# env.AddBody(obstacle3, True)
# embed()
# fcl_checker = FCLCollisionChecker()
# fcl_checker.InitFromEnv(env)
# path_length = raw_path.GetLength()
# t_ins = 0.0
# is_collided = False
# resolution = 0.05
# new_cm = env.GetCollisionMatrix()
# new_cm.DisablePair(mboxes[0], obstacle3.GetLinks()[0])
# temp_st = time.time()
# while t_ins < path_length:
#     curr_pos = raw_path.GetConfig(t_ins)
#     robot_model.SetJointPositions(Rx(curr_pos.Coeffs()))
#     for tl, init_p, mbox in zip(links[1:], init_poses[1:], mboxes[1:]):
#         mbox.SetBaseTransformation(tl.GetPose()*init_p)
#     is_collided, report = fcl_checker.CheckCollision(new_cm)
#     if is_collided == True:
#         break
#     else:
#         t_ins = t_ins + resolution
# temp_et = time.time()
# print("Total time: {} seconds.".format(temp_et - temp_st))
# if is_collided is True:
#     print("There exists collision between objects.")
#     print(report)
# else:
#     print("No collision.")
# embed()

env.AddBody(robot_model)
env.AddBody(obstacle3, True)
fcl_checker = FCLCollisionChecker()
fcl_checker.InitFromEnv(env)
path_length = raw_path.GetLength()
count = 0
while count < 100:
    t_ins = 0.0
    is_collided = False
    resolution = 0.05
    new_cm = env.GetCollisionMatrix()
    new_cm.DisablePair(links[2], links[0])
    new_cm.DisablePair(links[3], links[6])
    new_cm.DisablePair(links[3], links[5])
    temp_st = time.time()
    while t_ins < path_length:
        curr_pos = raw_path.GetConfig(t_ins)
        robot_model.SetJointPositions(Rx(curr_pos.Coeffs()))
        is_collided, report = fcl_checker.CheckCollision(new_cm)
        if is_collided == True:
            break
        else:
            t_ins = t_ins + resolution
    temp_et = time.time()
    print("Total time: {} seconds.".format(temp_et - temp_st))
    # if is_collided is True:
    #     print("There exists collision between objects.")
    #     print(report)
    # else:
    #     print("No collision.")
    count = count + 1


# fcl_collision_detection = FCLCollisionChecker()
# path_length = raw_path.GetLength()
# t_ins = 0.0
# # start time
# start_time = time.time()
# is_collided = False
# resolution = 0.05
# obstacle_links = [l.GetLink("BaseLink") for l in obstacles]
# while t_ins < path_length:
#     curr_pos = raw_path.GetConfig(t_ins)
#     robot_model.SetJointPositions(Rx(curr_pos.Coeffs()))
#     temp_links = robot_model.GetLinks()
#     links = []
#     for l in temp_links:
#         vis_geo = l.GetVisualGeometry()
#         if vis_geo.type != GeometryType_None:
#             links.append(l)
#     for link in links:
#         for obs_link in obstacle_links:
#             fcl_collision_detection.Clear()
#             fcl_collision_detection.AddCollisionObject(link)
#             fcl_collision_detection.AddCollisionObject(obs_link)
#             is_collided, _ = fcl_collision_detection.CheckCollision(
#                 link, obs_link)
#             if is_collided is True:
#                 break
#         if is_collided is True:
#             break
#     if is_collided is True:
#         break
#     else:
#         t_ins = t_ins + resolution
# end_time = time.time()
# print("Check collision based on Full Traverse:")
# print("Runtime: {} seconds.".format(end_time - start_time))
# if is_collided:
#     print("There exists collision between the manipulator and obstacle.\n")
# else:
#     print("No Collision!\n")

# embed()

# print("Check collision based on Dynamic Hierarchy:")
# dh_path_collision_checker = DHPathCollisionChecker(fcl_collision_detection)
# dh_path_collision_checker.InitFromEnv(env, manipulator)
# dh_path_collision_checker.UpdateContDistThreshold(0.0)
# dh_path_collision_checker.UpdatePathResolution(resolution)
# start_time = time.time()
# res = dh_path_collision_checker.CheckPathCollision(raw_path)
# end_time = time.time()
# print("Runtime: {} seconds.".format(end_time - start_time))
# if res:
#     print("There exists collision between the manipulator and obstacle.\n")
# else:
#     print("No Collision!\n")
