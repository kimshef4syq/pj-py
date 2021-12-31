/**
 * @example UseRTSPPlanner.cpp
 * @brief Demo of how to use RTSP planner.
 * @date 2021-08-19
 * 
 * @copyright Copyright (c) RVBUST, Inc - All rights reserved.
 */
#include <RVS/PlannerHeader.h>

int main()
{
    using namespace RVS;
    std::shared_ptr<RobotModel> robot_model = std::make_shared<RobotModel>();


    std::string file_path =
        GetDataPath()
        + "Multibody/RobotModels/ABB/IRB1200_5_90/IRB1200_5_90.rvdf";

    robot_model->InitFromRVDF(file_path.c_str());
    auto manipulator = robot_model->GetActiveManipulator();
    auto endeffector_active = manipulator->GetActiveEndEffector();
    auto temp = SE3d(-0.3, -0.000122204, 0.03, 0.707107, 0.000144019, 0.707107,
                     -0.000144019);
    endeffector_active->SetTCP(temp);
    auto joint_limits = manipulator->GetDoFLimits();
    MatXd waypoints(10, 7);
    waypoints.row(0) << 0.59160, -0.17180, 0.20300, 0.05020, 0.02840, -0.72750,
        0.68370;
    waypoints.row(1) << 0.23140, 0.36890, 0.18910, -0.01650, -0.03500, 0.99900,
        0.02280;
    waypoints.row(2) << 0.68570, 0.38240, 0.20130, 0.02980, 0.01730, -0.00050,
        -0.99940;
    waypoints.row(3) << 0.77370, 0.08610, 0.21760, -0.00890, 0.01580, 0.32270,
        -0.94630;
    waypoints.row(4) << 0.78500, -0.17630, 0.20920, -0.06500, -0.01370, 0.45160,
        -0.88970;
    waypoints.row(5) << 0.38890, -0.41940, 0.16140, -0.03060, -0.02370, 0.90110,
        -0.43190;
    waypoints.row(6) << 0.37700, -0.25470, 0.19360, -0.02610, -0.02890, 0.96470,
        -0.26060;
    waypoints.row(7) << 0.38750, -0.15360, 0.20230, 0.03320, 0.00640, -0.69470,
        0.71850;
    waypoints.row(8) << 0.35930, 0.08860, 0.22150, -0.00060, 0.01490, 0.11880,
        -0.99280;
    waypoints.row(9) << 0.43650, 0.25950, 0.23300, 0.48530, -0.02670, -0.87220,
        -0.05520;
    std::vector<ViapointPtr> viapoints(10);
    for (size_t i = 0; i < viapoints.size(); i++) {
        SE3d temp = SE3d(waypoints.row(i));
        viapoints[i] = std::make_shared<CartesianViapoint>(temp);
    }
    // define GTSP problem and configure GLKH solver
    GLKHProblemDescription glkh_prob_des;
    auto glkh_config = std::make_shared<GLKHConfig>(glkh_prob_des);
    // glkh_config->m_trace_level = 1;

    // construct a graph search method to find shortest path at joint space
    auto gs_with_a_star = std::make_shared<GraphSearchWithAStar>();
    // gs_with_a_star->UpdateHeuristic(HeuristicMethod_Duration);
    // gs_with_a_star->UpdateDofLimits(joint_limits);
    // **************customize a metric distance function*********(OPTIONAL)
    // std::vector<double> temp_vel_limits;
    // for (size_t i = 0; i < joint_limits.size(); i++) {
    //     auto temp_j = joint_limits[i].GetMaxVelocity();
    //     temp_vel_limits.emplace_back(temp_j);
    // }
    // CVecXd vel_limits_with_eigen = Eigen::Map<CVecXd, Eigen::Unaligned>(
    //     temp_vel_limits.data(), temp_vel_limits.size());
    // auto temp_metric_dist = [vel_limits_with_eigen](const Rxd &lhs,
    //                                                 const Rxd rhs,
    //                                                 const CVecXd &weights) {
    //     RVS_UNUSED(weights);
    //     auto angle_diff = (lhs - rhs).Coeffs();
    //     angle_diff = angle_diff.array() / vel_limits_with_eigen.array();
    //     // absolute value (to keep the weight positive)
    //     angle_diff = angle_diff.cwiseAbs();
    //     std::vector<double> temp_res(angle_diff.data(),
    //                                  angle_diff.data() + angle_diff.size());
    //     auto max_time_iter = std::max_element(temp_res.begin(),
    //     temp_res.end()); return *max_time_iter;
    // };
    // gs_with_a_star->SetMetricFunc(temp_metric_dist);
    // ************************ end **************************

    // graph search with Dijkstra's algorithm
    // auto gs_with_dijkstra = std::make_shared<GraphSearchWithDijkstra>();

    // Instantiate a dummy (nonsense) motion planner
    auto path_planner_factory = std::make_shared<PathPlannerFactory>();
    auto pp_with_dummy = path_planner_factory->Create("PathPlannerDummy");

    // construct a RTSP planner and its config in order to solve rtsp problem
    auto rtsp_planner = std::make_shared<RTSPPlanner>();
    auto rtsp_config = std::make_shared<RTSPPlanner::Configuration>(
        glkh_config, gs_with_a_star, pp_with_dummy);
    // auto rstp_config = std::make_shared<RTSPConfig>(
    //     glkh_config, gs_with_a_dijkstra, mp_with_dummy);
    rtsp_planner->SetConfiguration(rtsp_config);
    CVecXd temp_weights(6);
    temp_weights << 1, 1, 1, 0, 0, 0;
    rtsp_planner->SetPoseWeights(temp_weights);
    rtsp_planner->SetSampleParams(manipulator, RotationAxis_Z,
                                  Constants<double>::Pi() / 8.0);
    rtsp_planner->SetReferStartTargetAtCS(JointVector::IdentityStatic(6));
    rtsp_planner->EnableConsistency(true);
    MotionPlannerRequest rtsp_request;
    rtsp_request.viapoints = viapoints;
    MotionPlannerResponse rtsp_response;
    auto plan_res = rtsp_planner->Solve(rtsp_request, rtsp_response);
    if (!plan_res) {
        RVS_ERROR("Failed to solve rtsp problem!");
        return 0;
    }
    else {
        std::cout << "The optimal joint trajectory is as follow:" << std::endl;
        auto res_joint_trajs = rtsp_response.joint_trajectory;
        for (const auto &v : res_joint_trajs) {
            std::cout << v << std::endl;
        }
    }
    return 0;
}
