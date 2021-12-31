// Copyright (c) RVBUST, Inc - All rights reserved.

#include <RVS/Controller/SimController.h>
#include <RVS/RVScript/RVScriptExecutor.h>

using namespace RVS;

int main()
{
    auto controller = SimController::Create("Motoman_GP12");
    controller->Connect("127.0.0.1");
    controller->EnableRobot();
    auto executor = std::make_shared<ScriptExecutor>(controller);

    auto container = ScriptContainer::Create("TestSingleBlock");
    std::string py_code = R"#(
res, q = controller.GetJointPosition()
print(res, q)
)#";
    auto block = ScriptBlock::CreateBlockPyCode(py_code);
    container->InsertScriptBlock(block);
    RVS_INFO(
        "start to execute block: {}, contents:\n==========\n{}\n===========\n",
        block->Repr(), block->py_codes);
    executor->ExecuteScriptContainer(container, true);
    RVS_INFO("finish executing block: {}", block->Repr());
    return 0;
}
