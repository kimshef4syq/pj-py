/**
 * @example UseDHPathCollisionChecker.cpp
 * @brief Demo of use dynamic hierachy collision checker.
 * @date 2021-08-19
 *
 * @copyright Copyright (c) RVBUST, Inc - All rights reserved.
 */
#include <RVS/Common/Time.h>
#include <RVS/Common/Types.h>
#include <RVS/CollisionChecker/FCLCollisionChecker/FCLCollisionChecker.h>
#include <RVS/CollisionChecker/DynamicHierarchyCollisionCheck/DHPathCollisionChecker.h>
#include <RVS/Trajectory/TrajectoryUtilsEx.h>

using namespace RVS;

int main()
{
    auto rvs_log = Logger::GetConsoleLogger("RVS_CL");
    rvs_log->SetLevel(LoggerLevel_Error);


    std::shared_ptr<RobotModel> robot_model = std::make_shared<RobotModel>();
    auto path =
        GetDataPath() + "Multibody/RobotModels/UniversalRobots/UR5/UR5.rvdf";
    robot_model->InitFromRVDF(path.c_str());
    robot_model->SetBaseTransformation(SE3d(0, 0, 0, 0, 0, 0, 1));
    auto home_position =
        Rxd({-1.48524, -1.80172, 2.00258, -1.48595, 4.64519, -4.79932});
    robot_model->SetJointPositions(home_position);
    auto manipulator = robot_model->GetActiveManipulator();

    std::vector<std::shared_ptr<Multibody>> obstacles;
    // obstacle 0
    auto obstacle0 = std::make_shared<Multibody>();
    obstacle0->InitFromBox(0.2, 0.2, 0.2);
    obstacle0->SetName("obstacle0");
    obstacle0->SetBaseTransformation(
        SE3d(0.43356, -0.144609, 0.419158, 0, 0, 0, 1));
    obstacles.push_back(obstacle0);

    // obstacle 1
    auto obstacle1 = std::make_shared<Multibody>();
    obstacle1->InitFromBox(0.1, 0.2, 0.25);
    obstacle1->SetName("obstacle1");
    obstacle1->SetBaseTransformation(SE3d(0.2, -0.3, 0.1, 0, 0, 0, 1));
    obstacles.push_back(obstacle1);

    // obstacle 2
    auto obstacle2 = std::make_shared<Multibody>();
    obstacle2->InitFromBox(0.25, 0.1, 0.2);
    obstacle2->SetName("obstacle2");
    obstacle2->SetBaseTransformation(SE3d(0.4, 0.2, 0.3, 0, 0, 0, 1));
    obstacles.push_back(obstacle2);

    // Environment
    auto env = std::make_shared<Environment>();
    env->AddBody(robot_model);
    env->AddBodies(obstacles);

    auto start_point = manipulator->GetDoFPositions();
    auto end_point = Rxd({0.5, -0.5, 0.3, 0.1, 0.1, 0.1});
    auto raw_path =
        CreatePath<JointVector>(std::list<JointVector>{start_point, end_point});

    // create a collision checker, herein, FCL collision checker is constructed
    CollisionCheckerBasePtr fcl_collision_checker =
        std::make_shared<FCLCollisionChecker>();
    auto path_length = raw_path->GetLength();
    double t_ins = 0.0;
    bool is_collided = false;
    double sample_resolution = 0.05;
    std::vector<std::shared_ptr<Link>> arm_links, obs_links;
    for (auto &obs : obstacles) {
        obs_links.emplace_back(obs->GetLink("BaseLink"));
    }
    TimeStamp start_time = TimeStamp::Now();
    CollisionReport collision_report;
    while (t_ins < path_length) {
        auto curr_pos = raw_path->GetConfig(t_ins);
        manipulator->SetDoFPositions(Rxd(curr_pos.Coeffs()));
        arm_links.clear();
        for (auto &link : robot_model->GetLinks()) {
            auto vis_geom = link->GetVisualGeometry();
            if (vis_geom->type != GeometryType_None) {
                arm_links.emplace_back(link);
            }
        }
        for (auto &link : arm_links) {
            for (auto &obs_l : obs_links) {
                fcl_collision_checker->Clear();
                fcl_collision_checker->AddCollisionObject(*link, true);
                fcl_collision_checker->AddCollisionObject(*obs_l, true);
                is_collided = fcl_collision_checker->CheckCollision(
                    *link, *obs_l, collision_report);
                if (is_collided) {
                    break;
                }
            }
            if (is_collided) {
                break;
            }
        }
        if (is_collided) {
            break;
        }
        else {
            t_ins += sample_resolution;
        }
    }
    TimeStamp end_time = TimeStamp::Now();
    TimeDuration duration = end_time - start_time;
    std::cout << "Runtime (by FCL): " << duration << " seconds." << std::endl;
    if (is_collided) {
        std::cout
            << "There exists collision between the manipulator and obstacle."
            << std::endl;
    }
    else {
        std::cout << "No collision." << std::endl;
    }
    auto dh_path_collision_checker =
        std::make_shared<DHPathCollisionChecker>(fcl_collision_checker);
    // start_time = TimeStamp::Now();
    dh_path_collision_checker->InitFromEnv(env, manipulator);
    // end_time = TimeStamp::Now();
    // std::cout << "The time of initializing the object environment is: "
    //   << (end_time - start_time) << " seconds." << std::endl;
    dh_path_collision_checker->UpdateContDistThreshold(0.0);
    dh_path_collision_checker->UpdatePathResolution(sample_resolution);
    start_time = TimeStamp::Now();
    is_collided = dh_path_collision_checker->CheckPathCollision(raw_path);
    // is_collided = dh_path_collision_checker->CheckLinearPathCollision(
    //     start_point, end_point);
    end_time = TimeStamp::Now();
    duration = end_time - start_time;
    std::cout << "Runtime (by DH): " << duration << " seconds." << std::endl;
    if (is_collided) {
        std::cout
            << "There exists collision between the manipulator and obstacle."
            << std::endl;
    }
    else {
        std::cout << "No collision." << std::endl;
    }
    return 0;
}