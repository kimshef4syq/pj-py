'''!@example UsePropDistanceField.py
@brief 
@date 2021-08-19
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''

from RVBUST.RPS import *


field_box = Multibody()
field_box.InitFromBox(1, 1, 1)
field_box.SetBaseTransformation(Pose(0.5, 0.5, 0.5, 0, 0, 0, 1))
obs_box_1 = Multibody()
obs_box_1.InitFromBox(0.2, 0.3, 0.1)
obs_box_1.SetBaseTransformation(
    Pose(0.8, 0.2, 0.2, np.sin(np.pi/8), 0, 0, np.cos(np.pi/8)))
obs_box_2 = Multibody()
obs_box_2.InitFromBox(0.1, 0.2, 0.2)
obs_box_2.SetBaseTransformation(
    Pose(0.5, 0.5, 0.5, np.sin(np.pi/8), 0, 0, np.cos(np.pi/8)))
robot_vis = RobotVis()
robot_vis.GetView().Home()
robot_vis.GetView().SetCameraPose(
    [1.64, -1.93, 1.7], [1.22, - 1.14, 1.26], [-0.24, 0.37, 0.90])
# robot_vis.AddBody(field_box)
robot_vis.AddBody(obs_box_1)
robot_vis.AddBody(obs_box_2)

robot_model = RobotModel()
robot_model.InitFromRVDF(
    GetDataPath() + "Multibody/RobotModels/UniversalRobots/UR5/UR5.rvdf")
robot_vis.AddBody(robot_model)
manipulator = robot_model.GetActiveManipulator()
# no collision
# manipulator.SetDoFPositions(Rx([1.2, -0.5, -0.1, 0.2, 0.2, 0.2]))

# collision
# manipulator.SetDoFPositions(Rx([0.96,-0.5,-0.1,0.2,0.2,0.2]))

# no collision
# manipulator.SetDoFPositions(Rx([0.2,-0.5,-0.1,0.2,0.2,0.2]))

# collision
manipulator.SetDoFPositions(Rx([0.1, -0.4, -0.1, 0.2, 0.2, 0.2]))

print("Initialize the propagation distance field...")
# create a propagation distance field
field_bottom_left = [0, 0, 0]
field_top_right = [1, 1, 1]
resolution = 0.03
chomp_distance_field = PropagateDistanceField(
    field_bottom_left, field_top_right, resolution, "chomp distance field", 0.3, False)
chomp_distance_field.InitDistField()

# obstacle1 voxels
link_obs1 = obs_box_1.GetLink("BaseLink")
link_obs1_decomp = LinkDecomposition(
    link_obs1, chomp_distance_field.GetFieldResolution(), False)
link_obs1_voxels = link_obs1_decomp.GetLinkVoxel()
obs1_voxels = link_obs1_voxels.GetVoxels(False)
obs1_voxels_arr = np.array([obs1_voxels])
h_obs1 = robot_vis.GetView().Point(obs1_voxels_arr.flatten(), 5)

# obstacle2 voxels
link_obs2 = obs_box_2.GetLink("BaseLink")
link_obs2_decomp = LinkDecomposition(
    link_obs2, chomp_distance_field.GetFieldResolution(), False)
link_obs2_voxels = link_obs2_decomp.GetLinkVoxel()
obs2_voxels = link_obs2_voxels.GetVoxels(False)
obs2_voxels_arr = np.array([obs2_voxels])
h_obs2 = robot_vis.GetView().Point(obs2_voxels_arr.flatten(), 5)


# add two obstacles into the propagation distance field
chomp_distance_field.AddObjectsIntoField(obs1_voxels)
chomp_distance_field.AddObjectsIntoField(obs2_voxels)

# display the gradient info in the updated distance field
sampling_interval = chomp_distance_field.GetFieldResolution()*2
field_points_x = np.arange(0+sampling_interval/2.0,
                           1+sampling_interval/2.0, sampling_interval)
field_points_y = np.arange(0+sampling_interval/2.0,
                           1+sampling_interval/2.0, sampling_interval)
field_points_z = np.arange(0+sampling_interval/2.0,
                           1+sampling_interval/2.0, sampling_interval)
field_points = []
for x in field_points_x:
    for y in field_points_y:
        for z in field_points_z:
            temp = [x, y, z]
            field_points.append(temp)
line_sets = np.array([[]])
point_sets = np.array([[]])
for xyz in field_points:
    temp_pos = xyz
    temp_dist = chomp_distance_field.GetDistanceAtPosition(temp_pos)
    if temp_dist == -1:
        continue
    temp_grad = chomp_distance_field.GetGradientAtPosition(xyz)
    temp = temp_pos + 0.1 * np.array(temp_grad)
    temp_line = np.append(temp_pos, temp)
    if line_sets.size == 0:
        line_sets = np.array([temp_line])
    else:
        line_sets = np.append(line_sets, [temp_line], axis=0)
    if point_sets.size == 0:
        point_sets = np.array([temp_pos])
    else:
        point_sets = np.append(point_sets, [temp_pos], axis=0)
print("Displaying the gradient of distance field...")
h_field_points = robot_vis.GetView().Point(point_sets.flatten(), 4)
h_field_grad = robot_vis.GetView().Line(line_sets.flatten(), 1)

print("Finished (PropagateDistanceField).")
print("==================================")

links_with_rm = robot_model.GetLinks()
valid_links_with_rm = []
for link in links_with_rm:
    vis_geom = link.GetVisualGeometry()
    if vis_geom.type != GeometryType_None:
        valid_links_with_rm.append(link)
rm_link_decomp = []
print("robot's links sphere decomposition...")
for link in valid_links_with_rm:
    temp_link_decomp = LinkDecomposition(
        link, chomp_distance_field.GetFieldResolution(), False)
    rm_link_decomp.append(temp_link_decomp)
print("Finished (robot's links sphere decomposition).")
print("==================================")

rm_link_spheres = []
for link_d in rm_link_decomp:
    temp_spheres = link_d.GetLinkSpheres(False)
    rm_link_spheres = rm_link_spheres + temp_spheres

is_collided = False
print("Checking collision...")
for sphere in rm_link_spheres:
    temp_pos = list(sphere.pose.Translation())
    dist = chomp_distance_field.GetDistanceAtPosition(temp_pos)
    if dist == -1:
        continue
    if dist <= sphere.radius:
        is_collided = True
        print("The pose of sphere: ", sphere.pose)
        break
print("Finished.")
print("==================================")

print("The result is: ")
if is_collided is True:
    print("There exists collision between robot arm and obstacle.")
else:
    print("No collision!")

embed()
