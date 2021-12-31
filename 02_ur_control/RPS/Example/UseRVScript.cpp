/**
 * @example UseRVScript.cpp
 * @brief Using RVScript demo.
 * @date 2021-08-19
 * 
 * @copyright Copyright (c) RVBUST, Inc - All rights reserved.
 */
#include <RVS/RVScript/RVScript.h>
#include <RVS/RVScript/RVScriptExecutor.h>
#include <RVS/Kinematics/URsKinematics.h>
#include <RVS/Controller/SimController.h>
#include <string>
#include <streambuf>
using namespace RVS;

int main()
{
    RVS_SET_LEVEL(LoggerLevel_Error);
    RVS_DEBUG("Testing..");
    std::shared_ptr<RobotModel> robot_model = std::make_shared<RobotModel>();
    std::string path = GetDataPath()
                       + "Multibody/RobotModels/"
                         "UniversalRobots/UR5/UR5.rvdf";
    robot_model->InitFromRVDF(path.c_str());
    auto kin_solver =
        std::make_shared<URsKinematics>(robot_model->GetActiveManipulator());
    auto traj_data = std::make_shared<TrajectoryData>();
    auto controller =
        SimController::Create(robot_model->GetActiveManipulator());
    controller->Connect();
    controller->EnableRobot();
    auto container = ScriptContainer::Create();
    CVecXd joint(6);

    int io_addr = 1;
    bool io_val = true;
    *container += ScriptBlock::CreateBlockMsg("start");
    RVS_INFO("here");
    *container += ScriptBlock::CreateBlockSetDO(io_addr, io_val);

    *container += ScriptBlock::CreateBlockMsg("move to home");
    // move to home
    {
        traj_data->SetIsConstantVelocity(false);
        joint << 0, 0, 0, 0, 0, 0;
        traj_data->AddWaypoint(joint);
        *container += ScriptBlock::CreateBlockTraj(traj_data);
    }
    // WaitDI, controller UI not yet realized
    double timeout = 1.0;
    *container += ScriptBlock::CreateBlockWaitDI(io_addr, io_val, timeout);

    // SetDO
    io_addr = 2;
    *container += ScriptBlock::CreateBlockSetDO(io_addr, false);
    *container += ScriptBlock::CreateBlockLogicWait(1);

    auto block_if = ScriptBlock::CreateBlockEmpty();
    // if condition: statement
    {
        traj_data = traj_data->Copy();
        traj_data->ClearWaypoints();
        traj_data->SetMotionType(MotionType_MoveJ);
        traj_data->AddWaypoint(joint);
        joint << 0, 0.5, 0, 0, M_PI_2, 0;
        traj_data->AddWaypoint(joint);
        auto statements = ScriptContainer::Create();
        *statements += ScriptBlock::CreateBlockMsg("execut if block");
        *statements += ScriptBlock::CreateBlockTraj(traj_data);
        auto condition = ScriptBlock::CreateBlockGetDO(io_addr);
        block_if = ScriptBlock::CreateBlockLogicIf(condition, true, statements);
    }
    // elif condition: statement
    {
        auto condition = ScriptBlock::CreateBlockGetDO(io_addr);
        traj_data = traj_data->Copy();
        traj_data->ClearWaypoints();
        traj_data->AddWaypoint(joint);
        joint << 0, 0.5, -0.5, 0, M_PI_2, 0;
        traj_data->AddWaypoint(joint);
        auto statements = ScriptContainer::Create();
        *statements += ScriptBlock::CreateBlockMsg("execut elif block");
        *statements += ScriptBlock::CreateBlockTraj(traj_data);
        *block_if->elif_container +=
            ScriptBlock::CreateBlockLogicElif(condition, false, statements);
    }
    // else: statement
    {
        traj_data = traj_data->Copy();
        traj_data->ClearWaypoints();
        traj_data->AddWaypoint(joint);
        joint << 0, 1.5, -0.5, 0, M_PI_2, 0;
        traj_data->AddWaypoint(joint);
        auto statements = ScriptContainer::Create();
        *statements += ScriptBlock::CreateBlockMsg("execut else block");
        *statements += ScriptBlock::CreateBlockTraj(traj_data);
        block_if->else_container = ScriptContainer::Create(
            ScriptBlock::CreateBlockLogicElse(statements));
    }
    *container += block_if;
    io_addr = 4;
    *container += ScriptBlock::CreateBlockSetDO(io_addr, true);
    // while block
    {
        auto condition = ScriptBlock::CreateBlockGetDO(io_addr);
        traj_data = traj_data->Copy();
        traj_data->ClearWaypoints();
        traj_data->AddWaypoint(joint);
        joint << 0, 0.5, 0, 0, -M_PI_2, 0;
        traj_data->AddWaypoint(joint);
        auto statements = ScriptContainer::Create(
            {ScriptBlock::CreateBlockMsg("execut while block"),
             ScriptBlock::CreateBlockTraj(traj_data),
             ScriptBlock::CreateBlockSetDO(io_addr, false)});
        *container +=
            ScriptBlock::CreateBlockLogicWhile(condition, true, statements);
    }

    // for block
    {
        io_addr = 5;
        *container += ScriptBlock::CreateBlockSetDO(io_addr, true);
        int loops = 2;
        auto statements = ScriptContainer::Create(
            ScriptBlock::CreateBlockMsg("execute for block"));
        traj_data->Copy();
        traj_data->ClearWaypoints();
        traj_data->AddWaypoint(joint);
        joint << 0, 0, 0, M_PI_4, 0, 0;
        traj_data->AddWaypoint(joint);
        auto condition_embed = ScriptBlock::CreateBlockGetDO(io_addr);
        auto statements_embed = ScriptContainer::Create(
            {ScriptBlock::CreateBlockTraj(traj_data),
             ScriptBlock::CreateBlockSetDO(io_addr, false)});
        *statements += ScriptBlock::CreateBlockLogicIf(condition_embed, true,
                                                       statements_embed);
        *container += ScriptBlock::CreateBlockLogicFor(statements, loops);
    }
    *container += ScriptBlock::CreateBlockMsg("end");
    RVS_DEBUG("End test");

    /// Save to RVScript
    path = "/home/rvbust/.Temp/RVS_AUTO.rvscript";
    std::cout << container->Repr() << std::endl;
    container->SaveAsScript(path);

    // Save to json
    auto j_str = container->ToJson();
    std::string json_path = std::string(getenv("HOME"))
                            + "/Rvbust/Sources/RVS/Modules/RVScript/"
                              "Example/RVScript.json";
    std::ofstream ofile(json_path);
    ofile << j_str << std::endl;
    ofile.close();

    /// load from json
    std::ifstream ifile(json_path);
    std::string j_str2((std::istreambuf_iterator<char>(ifile)),
                       std::istreambuf_iterator<char>());
    ifile.close();

    auto container2 = ScriptContainer::Create();
    container2->FromJson(j_str2);
    path = "/home/rvbust/.Temp/RVS_AUTO2.rvscript";
    container2->SaveAsScript(path);

    /// Print program
    std::cout
        << "================================================================"
        << std::endl;
    std::string prog;
    RVS_ENSURE(container->ToScript(prog), "Convert to RVScript program failed");
    std::cout << prog << std::endl;
    std::cout
        << "================================================================"
        << std::endl;

    /// Test executing program

    auto executor = std::make_shared<ScriptExecutor>(
        controller); // must use shared_ptr instead of object
#ifdef Build_PyInterpreter
    // the following two commands have same effects
    executor->ExecuteCommands(prog, false);
    while (executor->IsBusy()) {
        RVS::TimeDuration(0.1).Sleep();
    }
    std::cout << "result1: "
              << GetRVSReturnString(
                     std::get<RVSReturn>(executor->GetExecuteProgResult()))
              << ", info: "
              << std::get<std::string>(executor->GetExecuteProgResult())
              << std::endl;
#endif
    // sleep(5);
    // executor->Terminate();
    // sleep(5); // wait to be terminated
    executor->ExecuteScriptContainer(container, true);
    std::cout << "result2: "
              << GetRVSReturnString(
                     std::get<RVSReturn>(executor->GetExecuteProgResult()))
              << ", info: "
              << std::get<std::string>(executor->GetExecuteProgResult())
              << std::endl;

    return 0;
}