/**
 * @example UsePropDistanceField.cpp
 * @brief Demo of using propagation distance filed.
 * @date 2021-08-19
 *
 * @copyright Copyright (c) RVBUST, Inc - All rights reserved.
 */
#include <RVS/CollisionChecker/PropagateDistanceField/PropagateDistanceField.h>
#include <RVS/CollisionChecker/PropagateDistanceField/LinkDecomposition.h>
#include <RVS/Environment/Manipulator.h>

using namespace RVS;

int main()
{
    using std::cos;
    using std::sin;

    // import robot model from rvdf
    auto robot_model = std::make_shared<RobotModel>();
    auto file_path =
        GetDataPath() + "Multibody/RobotModels/UniversalRobots/UR5/UR5.rvdf";
    robot_model->InitFromRVDF(file_path.c_str());
    auto manipulator = robot_model->GetActiveManipulator();
    // TEST
    // no collision
    // manipulator->SetDoFPositions(Rxd({1.2, -0.5, -0.1, 0.2, 0.2, 0.2}));

    // collision
    // manipulator->SetDoFPositions(Rxd({0.96, -0.5, -0.1, 0.2, 0.2, 0.2}));

    // no collision
    // manipulator->SetDoFPositions(Rxd({0.2, -0.5, -0.1, 0.2, 0.2, 0.2}));

    // collision
    manipulator->SetDoFPositions(Rxd({0.1, -0.4, -0.1, 0.2, 0.2, 0.2}));

    // obstacle a in the distance field
    auto obstacle_box_a = std::make_shared<Multibody>();
    obstacle_box_a->InitFromBox(0.2, 0.3, 0.1);
    SE3d temp_pose = SE3d(0.8, 0.2, 0.2, sin(Constants<double>::Pi() / 8.0), 0,
                          0, cos(Constants<double>::Pi() / 8.0));
    obstacle_box_a->SetBaseTransformation(temp_pose);

    // obstacle b in the distance field
    auto obstacle_box_b = std::make_shared<Multibody>();
    obstacle_box_b->InitFromBox(0.1, 0.2, 0.2);
    temp_pose = SE3d(0.5, 0.5, 0.5, sin(Constants<double>::Pi() / 8.0), 0, 0,
                     cos(Constants<double>::Pi() / 8.0));
    obstacle_box_b->SetBaseTransformation(temp_pose);

    RVS_INFO("Initialize the propagation distance field...");
    // create a propagation distance field with the range of [0,0,0] to [1,1,1]
    std::array<double, 3> field_bottom_left({0.0, 0.0, 0.0});
    std::array<double, 3> field_top_right({1.0, 1.0, 1.0});
    double field_resolution = 0.03;
    auto prop_distance_field = std::make_shared<PropagateDistanceField>(
        field_bottom_left, field_top_right, field_resolution,
        "chomp distance field", 0.3, false);
    prop_distance_field->InitDistField();

    // Yield the voxels with obstacle a
    auto link_obs_a = obstacle_box_a->GetLink("BaseLink");
    auto link_obs_a_decomp = std::make_shared<LinkDecomposition>(
        link_obs_a, prop_distance_field->GetFieldResolution(), false);
    auto link_obs_a_voxels = link_obs_a_decomp->GetLinkVoxel();

    // Yield the voxels with obstacle b
    auto link_obs_b = obstacle_box_b->GetLink("BaseLink");
    auto link_obs_b_decomp = std::make_shared<LinkDecomposition>(
        link_obs_b, prop_distance_field->GetFieldResolution(), false);
    auto link_obs_b_voxels = link_obs_b_decomp->GetLinkVoxel();

    // add obstacle a and b into the propagation distance field
    prop_distance_field->AddObjectsIntoField(
        link_obs_a_voxels->GetVoxels(false));
    prop_distance_field->AddObjectsIntoField(
        link_obs_b_voxels->GetVoxels(false));
    RVS_INFO("Finished (PropagateDistanceField).");

    auto links_from_robot_model = robot_model->GetLinks();
    std::vector<std::shared_ptr<Link>> valid_links;
    valid_links.clear();
    for (const auto &link : links_from_robot_model) {
        auto vis_geom = link->GetVisualGeometry();
        if (vis_geom->type != GeometryType_None) {
            valid_links.emplace_back(link);
        }
    }
    std::vector<std::shared_ptr<LinkDecomposition>>
        link_decomp_with_robot_model;
    link_decomp_with_robot_model.clear();
    RVS_INFO("Robot's links sphere or voxel decomposition...");
    // To decompose the links of robot model into collision spheres or collision
    // voxels in order to perform collision check
    // Note: More often than not, it will take a long time (several seconds or
    // much more) to perform sphere or voxel decomposition with robot model
    for (auto &v_l : valid_links) {
        auto temp_link_decomp = std::make_shared<LinkDecomposition>(
            v_l, prop_distance_field->GetFieldResolution(), false);
        link_decomp_with_robot_model.emplace_back(temp_link_decomp);
    }
    RVS_INFO("Finished (Robot's links sphere decomposition).");

    std::vector<std::shared_ptr<Sphere>> link_spheres_with_rm;
    link_spheres_with_rm.clear();
    for (const auto &l_d : link_decomp_with_robot_model) {
        auto temp = l_d->GetLinkSpheres(false);
        link_spheres_with_rm.insert(link_spheres_with_rm.end(), temp.begin(),
                                    temp.end());
    }
    RVS_INFO("Checking collision between robot model and obstacles...");
    bool is_collided = false;
    for (const auto &sphere : link_spheres_with_rm) {
        auto temp_pos = sphere->pose.Translation();
        double dist_to_obstacle = prop_distance_field->GetDistanceAtPosition(
            {temp_pos.x(), temp_pos.y(), temp_pos.z()});
        // check whether the given position is valid in distance field
        if (dist_to_obstacle == -1) {
            continue;
        }
        if (dist_to_obstacle <= sphere->radius) {
            is_collided = true;
            RVS_INFO(
                "The sphere with radius {} and center [{}] is collided with "
                "some obstacle.",
                sphere->radius, temp_pos.transpose());
            break;
        }
    }
    RVS_INFO("Finished (checking collision).");

    RVS_INFO("The result: ");
    if (is_collided) {
        RVS_INFO("There exists collision between robot arm and obstacles.");
    }
    else {
        RVS_INFO("No collision!");
    }
    return 0;
}
