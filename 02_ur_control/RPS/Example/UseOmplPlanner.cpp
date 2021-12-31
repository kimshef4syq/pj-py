/**
 * @example UseOmplPlanner.cpp
 * @brief Demo of how to use ompl planner.
 * @date 2021-08-19
 * 
 * @copyright Copyright (c) RVBUST, Inc - All rights reserved.
 */
#include <RVS/PlannerHeader.h>
#include <RVS/Common/Time.h>

int main()
{
    using namespace RVS;

    std::shared_ptr<RobotModel> robot_model = std::make_shared<RobotModel>();
    std::shared_ptr<Multibody> object = std::make_shared<Multibody>();

    std::string file_path1 =
        GetDataPath()
        + "Multibody/RobotModels/UniversalRobots/UR5/UR5Limit.rvdf";
    std::string file_path2 =
        GetDataPath() + "Multibody/3DModels/RVBUSTLogo.stl";

    robot_model->InitFromRVDF(file_path1.c_str());
    object->InitFromMeshFile(file_path2.c_str());
    object->SetBaseTransformation(SE3d(.5, 0, .3, 0, -0.4, 0, 1));

    auto manip = robot_model->GetActiveManipulator();

    auto env = std::make_shared<Environment>();

    env->AddBody(robot_model);
    env->AddBody(object);

    auto configuration =
        std::make_shared<OmplParallelPlanMotionPlanner::Configuration>(
            env, std::vector<std::shared_ptr<Manipulator>>{manip});

    configuration->planning_time = 10;
    configuration->hybridize = false;
    configuration->smooth = false;
    configuration->simplify = true;

    OmplParallelPlanMotionPlanner planner;

    planner.SetConfiguration(configuration);

    CVecXd data(6);
    data << -0.16891239, 0.67904491, -1.90386973, -1.09577485, 1.85397128,
        0.95098244;
    JointVector start_vec(data);
    data << -0.28952, -1.58203, 2.10125, -2.85594, -1.1842, 0.323;
    JointVector goal_vec(data);

    std::vector<JointVector> path;

    MotionPlannerRequest request;
    request.viapoints.clear();
    request.viapoints.reserve(2);
    request.viapoints.push_back(
        std::make_shared<JointViapoint>(start_vec, true));
    request.viapoints.push_back(
        std::make_shared<JointViapoint>(goal_vec, true));

    MotionPlannerResponse response;

    TimeStamp start_time = TimeStamp::Now();

    bool res = planner.Solve(request, response, false);

    TimeDuration planning_time = TimeStamp::Now() - start_time;

    RVS_INFO("Planning result is {}, and planning time is {}", res,
             planning_time);
    if (res) {
        for (const auto &waypoint : response.joint_trajectory) {
            RVS_INFO("Traj is {}", waypoint);
        }
    }
    else {
        RVS_INFO("Cannot find valid solution in {} sec",
                 planner.GetConfiguration()->planning_time);
    }

    return 0;
}
