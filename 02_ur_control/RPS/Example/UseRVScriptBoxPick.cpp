/**
 * @example UseRVScriptBoxPick.cpp
 * @brief using RVScript to program a box picking task.
 * @date 2021-08-19
 * 
 * @copyright Copyright (c) RVBUST, Inc - All rights reserved.
 */
#include <RVS/Controller/SimController.h>
#include <RVS/RVScriptHeader.h>
#include <filesystem>


int main()
{
    using namespace RVS;
    namespace fs = std::filesystem;
    RVS_SET_LEVEL(LoggerLevel_Error);

    const auto HOME_JOINTS = Rxd({0, 0, 0, 0, -M_PI_2, 0});
    const auto gp12rvdf =
        fs::path(getenv("HOME"))
        / "Rvbust/Data/Multibody/RobotModels/Motoman/GP12/GP12.rvdf";
    auto robot = std::make_shared<RobotModel>();
    robot->InitFromRVDF(gp12rvdf.c_str());
    auto mani = robot->GetActiveManipulator();
    auto controller = SimController::Create(mani);
    controller->Connect();
    controller->EnableRobot();
    controller->MoveJoints(HOME_JOINTS);

    auto box = std::make_shared<Multibody>();
    box->InitFromBox(0.3, 0.2, 0.1);
    box->SetBaseTransformation(SE3d(1, 0, 0.5, 0, 0, 0, 1));

    auto env = std::make_shared<Environment>();
    env->AddBody(robot);
    env->AddBody(box);

    auto main_prog = ScriptContainer::Create();
    *main_prog += ScriptBlock::CreateBlockMsg("Start to move home");

    // move to home
    auto traj_data = std::make_shared<TrajectoryData>();
    traj_data->SetIsConstantVelocity(false);
    Rxd q;
    controller->GetJointPosition(q);
    auto waypoint1 = std::make_shared<Waypoint>(q);
    auto waypoint2 = std::make_shared<Waypoint>(HOME_JOINTS);
    traj_data->AddWaypoint(waypoint1);
    traj_data->AddWaypoint(waypoint2);
    *main_prog += ScriptBlock::CreateBlockTraj(traj_data);

    // loop statements
    // receive box pose
    auto statements = ScriptContainer::Create();
    *statements += ScriptBlock::CreateBlockRemoteUpdateBodyPose(
        box->GetUniqueName(), "127.0.0.1", 8888);

    //     std::string codes = R"#(
    // kin_solver = controller.GetKinSolver()
    // pose = env.GetBody("Box").GetBaseTransformation()
    // q = controller.GetJointPosition()[1]
    // ret, ik_ret, min_dist = kin_solver.GetNearestIK(pose, q)
    // if ik_ret.success:
    //     controller.SetDigitalOutput(1, True)
    // else:
    //     controller.SetDigitalOutput(1, False)
    // )#";
    //     auto block = ScriptBlock::CreateBlockPyCode(codes);
    //     *statements += block;

    // go to pick
    auto ifstatements = ScriptContainer::Create();
    *ifstatements += ScriptBlock::CreateBlockMsg("Go to pick box");
    traj_data->ClearWaypoints();
    traj_data->SetMotionType(MotionType_MoveL);
    traj_data->SetTrajType(TrajType_Trapezoidal);
    traj_data->AddWaypoint(std::make_shared<Waypoint>(HOME_JOINTS));
    traj_data->AddWaypoint(std::make_shared<Waypoint>(
        SE3d(-0.016, 0, 0.2, 1, 0, -0, 0), q, box->GetUniqueName()));
    traj_data->AddWaypoint(std::make_shared<Waypoint>(
        SE3d(-0.016, 0, 0.05, 1, 0, -0, 0), q, box->GetUniqueName()));
    *ifstatements += ScriptBlock::CreateBlockTraj(traj_data);
    *ifstatements += ScriptBlock::CreateBlockGrab(box->GetUniqueName(), 1, true);

    // go to place
    traj_data->ClearWaypoints();
    traj_data->SetMotionType(MotionType_MoveJ);
    traj_data->SetTrajType(TrajType_Trapezoidal);
    traj_data->AddWaypoint(std::make_shared<Waypoint>(
        SE3d(-0.016, 0, 0.05, 1, 0, -0, 0), q, box->GetUniqueName()));
    traj_data->AddWaypoint(std::make_shared<Waypoint>(
        SE3d(-0.016, 0, 0.2, 1, 0, -0, 0), q, box->GetUniqueName()));
    traj_data->AddWaypoint(
        std::make_shared<Waypoint>(Rxd({1.25, 1.0, -0.4, 0, -0.2, 0})));
    *ifstatements += ScriptBlock::CreateBlockTraj(traj_data);
    *ifstatements += ScriptBlock::CreateBlockRelease(box->GetUniqueName(), 1, false);

    *statements += ScriptBlock::CreateBlockLogicIf(
        ScriptBlock::CreateBlockGetDO(1), true, ifstatements);

    // move to home
    traj_data->ClearWaypoints();
    traj_data->SetMotionType(MotionType_MoveJ);
    traj_data->SetTrajType(TrajType_Trapezoidal);
    traj_data->AddWaypoint(std::make_shared<Waypoint>(HOME_JOINTS));
    *statements += ScriptBlock::CreateBlockTraj(traj_data);

    *main_prog += ScriptBlock::CreateBlockLogicFor(statements, 5);

    auto prog_path = fs::path(getenv("HOME")) / "BinPick.json";
    std::ofstream outfile(prog_path);
    outfile << main_prog->ToJson();
    outfile.close();

    auto executor = std::make_shared<ScriptExecutor>(controller, env);
    executor->ExecuteScriptContainer(main_prog);
    return 0;
}