/**
 * @example UseOptSolverIK.cpp
 * @brief Example of use OptSovler to solve robot inverse kinematics problem.
 * @date 2021-08-19
 *
 * @copyright Copyright (c) RVBUST, Inc - All rights reserved.
 */
#include <RVS/Kinematics/CostCntTerms.h>
#include <RVS/OptSolver/NloptSolver.h>
#include <RVS/OptModel/OptModel.h>
#include <RVS/Common/Time.h>

using namespace std;
using namespace RVS;


std::vector<Bounds> GetDoFBounds(std::shared_ptr<Manipulator> manip)
{

    auto dof_limits = manip->GetDoFLimits();
    std::vector<Bounds> vec_bounds;
    RVS_INFO("dof limit \n {}", dof_limits.size());
    for (size_t i = 0; i < dof_limits.size(); ++i) {
        double lu = dof_limits[i].GetMaxPosition();
        double ll = dof_limits[i].GetMinPosition();
        Bounds bounds(ll, lu);
        vec_bounds.push_back(bounds);
    }
    return vec_bounds;
}
int main()
{

    std::string file_path =
        GetDataPath() + "Multibody/RobotModels/UniversalRobots/UR5/UR5.rvdf";

    std::shared_ptr<RobotModel> robot_model = std::make_shared<RobotModel>();
    robot_model->InitFromRVDF(file_path.c_str());
    robot_model->SetDefaultManipulator();
    std::shared_ptr<Manipulator> manip = robot_model->GetActiveManipulator();

    CVecXd qv(6);
    qv << -1.8169, -1.66768, 1.32147, 0.252495, 1.55772, -1.64367;
    Rxd q(qv);

    string link_name = manip->GetEndEffectorLinkName();
    // string link_name = "Link4";

    manip->SetDoFPositions(q);
    SE3d world_to_base(0, 0, 0, 0, 0, 0, 1);
    SE3d end_to_tool(0.1, -0.13, 0, 0, 0, 0, 1);
    // SE3d end_to_tool(0.155, -0.157, 0, -0.707107, 0, 0, 0.707107);


    SE3d target_pose = world_to_base
                       * robot_model->GetLink(link_name)->GetPose()
                       * end_to_tool;


    int num_dof = manip->GetDoF();
    OptModelPtr problem = std::make_shared<OptModel>();
    std::vector<Bounds> bounds = GetDoFBounds(manip);
    RVS_INFO("bounds: \n {}", bounds.size());
    problem->SetVariableSet(std::make_shared<VariableSet>(num_dof, bounds));

    using IdxVec = CostTerm::IdxVec;
    IdxVec idx(num_dof);
    std::iota(idx.begin(), idx.end(), 0);

    FunctorPtr pose_err_func = std::make_shared<CartPoseErrFunctor>(
        manip, link_name, target_pose, end_to_tool, world_to_base);


    CostFromFunctorPtr pose_cost = std::make_shared<CostFromFunctor>(
        pose_err_func, true, idx, CVecXd::Ones(6),
        PenaltyType::PenaltyType_Squared, "pose_cost");

    problem->AddCostTerm(pose_cost);
    problem->PrintCurrent();

    NloptSolver solver;
    vector<double> x_init = {-0.190814, -1.63731, 1.01861,
                             -0.953768, -1.57096, -0.106058};
    solver.Initialize(x_init, nlopt::AUGLAG_EQ);
    nlopt::opt opt_local(nlopt::algorithm::LD_MMA, x_init.size());
    opt_local.set_xtol_rel(1e-6);
    opt_local.set_initial_step(0.01);
    solver.GetNloptApp()->set_local_optimizer(opt_local);
    solver.GetNloptApp()->set_initial_step(1e-3);
    solver.GetNloptApp()->set_xtol_rel(1e-6);

    TimeStamp t1 = RVS::TimeStamp::Now();
    solver.Solve(problem);
    TimeStamp t2 = RVS::TimeStamp::Now();
    RVS_INFO("Using numerical jacobian optimization cost time {} s",
             (t2 - t1).ToSec());


    pose_cost->SetUseNumJacobi(false);
    TimeStamp t3 = RVS::TimeStamp::Now();
    solver.Solve(problem);
    TimeStamp t4 = RVS::TimeStamp::Now();
    RVS_INFO("Using analytic jacobian optimization cost time {} s",
             (t4 - t3).ToSec());

    auto opt_vars = problem->GetVariableSet()->GetValueVec();
    double obj_val = problem->GetCostValue(opt_vars.data(), opt_vars.size());
    PrintVec(opt_vars.data(), opt_vars.size(), "Optimized variables");

    Rxd solved_q(opt_vars);
    manip->SetDoFPositions(solved_q);

    SE3d pose_t = manip->GetRobotModel()->GetLink(link_name)->GetPose();
    pose_t = world_to_base * pose_t * end_to_tool;

    RVS_INFO("target pose: \n[{}]", target_pose);
    RVS_INFO("solved_pose: \n[{}]", pose_t);
    printf("cost valve: %f\n", obj_val);


    return 0;
}