// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include <RVS/Controller/GenericRobotController.h>
#include "RVScript.h"
#include <list>

namespace RVS
{
/// @addtogroup RVScript
/// @{

class ScriptExecutor : public std::enable_shared_from_this<ScriptExecutor>
{
public:
    ScriptExecutor(std::shared_ptr<GenericRobotController> controller,
                   EnvironmentPtr env = nullptr);

    ~ScriptExecutor();

    /// @brief Whether the executor is busy with executing program.
    inline bool IsBusy() const { return m_is_busy; }

    /// @brief Get executing program result. return [RVSReturn,
    /// result_detailed_info]
    std::tuple<RVSReturn, std::string> GetExecuteProgResult() const
    {
        return m_execute_prog_result;
    }

    /**
     * @brief Terminate program after finishing executing current ScriptBlock,
     * aka stop executing program as soon as possible.
     *
     * @param wait blocking or not
     * @return true
     * @return false
     */
    bool Terminate(bool wait = true);

    /// @brief Reset all flags as initialized values
    void Reset();

    /**
     * @brief Execute a ScriptContainer
     *
     * @param container : script container
     * @param wait : if true, execute in current thread, otherwise in a new
     * thread
     * @param from_start : if true, execute from the first script block,
     * otherwise continue from last paused block.
     * @return RVSReturn
     */
    RVSReturn ExecuteScriptContainer(ScriptContainerPtr container,
                                     const bool wait = true,
                                     const bool from_start = true);

    /**
     * @brief Execute a ScriptBlock
     *
     * @param block : script block
     * @param wait : if true, execute in current thread, otherwise in a new
     * thread
     * @return RVSReturn
     */
    RVSReturn ExecuteScriptBlock(ScriptBlockPtr bock, const bool wait = true);

#ifdef Build_PyInterpreter
    /**
     * @brief Move robot along some waypoints in joint space, throw an exception
     * if failed
     *
     * @param waypoints  joint space points
     * @param blend_tolerance allowed tolerance when blending
     * @param velocity velocity of trajectory, <= 0 will use time opt traj
     * @return void
     */
    void MoveJ(const std::list<CVecXd> &waypoints, double blend_tolerance = 0.1,
               double velocity = -1.0);

    /**
     * @brief Move robot along some waypoints in cartesian space, throw an
     * exception if failed
     *
     * @param waypoints a list cartesian waypoints, [[x, y, z, ox, oy, oz, ow],
     * ...]
     * @param blend_tolerance allowed tolerance when blending
     * @param velocity velocity of trajectory, <= 0 will use time opt traj
     * @return void
     */
    void MoveL(const std::list<CVecXd> &waypoints, double blend_tolerance = 0.1,
               double velocity = -1.0);

    /**
     * @brief Move robot along an arc in cartesian space, throw an exception if
     * failed
     *
     * @param waypoints must be a list of 3 cartesian waypoints, [[x, y, z, ox,
     * oy, oz, ow], ...]
     * @param velocity velocity of trajectory, <= 0 will use time opt traj
     * @return void
     */
    void MoveC(const std::list<CVecXd> &waypoints, double velocity = -1.0);
#endif

    /**
     * @brief Set the Execute Commands Result, used for ExecuteCommands
     *
     * @param info result information
     * @param ret result status
     */
    void SetExecuteProgResult(const std::string &info,
                              const RVSReturn ret = RVSReturn_Success);

    /**
     * @brief Set controller digital output value, throw an exception if failed
     *
     * @param io_addr IO address
     * @param io_val IO value
     * @return void
     */
    void SetDO(const int io_addr, const bool io_val);

    /**
     * @brief Set controller analog output value, throw an exception if failed
     *
     * @param io_addr IO address
     * @param io_val IO value
     * @return void
     */
    void SetAO(const int io_addr, const double io_val);

    /**
     * @brief Get controller digital input value, if cannot read io value, an
     * exception will be thrown out
     *
     * @param io_addr IO address
     * @return true IO value is true
     * @return false IO value is false
     */
    bool GetDI(const int io_addr);

    /**
     * @brief Get controller digital output value, if cannot read io value, an
     * exception will be thrown out
     *
     * @param io_addr IO address
     * @return true IO value is true
     * @return false IO value is false
     */
    bool GetDO(const int io_addr);

    /**
     * @brief Get controller analog input value, if cannot read io value, an
     * exception will be thrown out
     *
     * @param io_addr IO address
     * @return double analog input value
     */
    double GetAI(const int io_addr);

    /**
     * @brief Wait until digital input at io_addr is same with io_val, or
     * waiting time is more than timeout (seconds)
     *
     * @param io_addr : digital input address
     * @param io_val : required digital input value
     * @param timeout : time out for waiting (seconds), if time out < 0, waiting
     * forever
     * @return true : IO value is same with required value
     * @return false : time out
     */
    bool WaitDI(const int io_addr, const bool io_val,
                const double timeout = -1.0);

    /**
     * @brief Wait for a while
     *
     * @param timeout : time out in seconds
     */
    void Wait(double timeout);

    /**
     * @brief Get the Controller
     *
     * @return std::shared_ptr<GenericRobotController<DOF>>
     */
    std::shared_ptr<GenericRobotController> GetController() const
    {
        return m_controller.lock();
    }

    bool IsTerminateFlagSet() const { return m_terminate_program; }

#ifdef Build_PyInterpreter
    /**
     * @brief Execute a script
     *
     * @param file_path : script file path
     * @param wait : if true, start a new thread to execute
     * @return true
     * @return false
     */
    RVSReturn ExecuteScript(const std::string &file_path,
                            const bool wait = true);

    /**
     * @brief Execute the RVScript program represented by string
     *
     * @param commands RVScript commands string
     * @param wait : if true, start a new thread to execute
     * @return RVSReturn
     */
    RVSReturn ExecuteCommands(const std::string &commands,
                              const bool wait = true);
#endif

private:
    /**
     * @brief Execute the RVScript program represented by a ScriptContainer,
     * succeeds if no exception is thrown
     * @note This will execute each ScriptBlock in c++, not in embed python
     * interpreter
     *
     * @param container : ScriptContainer
     * @param wait : if true, start a new thread to execute
     * @param from_start : if true, execute from the first script block,
     * otherwise continue from last paused block.
     * @return void
     */
    void _ExecuteScriptContainer(ScriptContainerPtr container,
                                 const bool wait = true,
                                 const bool from_start = true);

    /**
     * @brief Execute a single ScriptBlock, if no exception, the execution
     * succeeds with a block return value
     * @note The return value doesn't indicate whether the execution is
     * successful or not, but the block's return value, eg. a GetDI block,
     * should return true/false of the DI
     * @todo Handle the condition if block return value is double or int, maybe
     * use std::variant<double, int, bool> as return value
     *
     * @param block : ScriptBlock
     * @param wait : if true, start a new thread to execute
     * @return true :
     * @return false
     */
    bool _ExecuteScriptBlock(ScriptBlockPtr block, const bool wait = true,
                             const bool from_start = true);

    EnvironmentPtr GetEnv() { return m_env.lock(); }

private:
    std::weak_ptr<GenericRobotController> m_controller; ///< robot controller
    EnvironmentWeakPtr m_env; ///< robot controller
    bool m_terminate_program; ///< used to terminate executing program
                              ///< immediately
    bool m_is_busy; ///< is executor busy with a container
    std::tuple<RVSReturn, std::string> m_execute_prog_result;
    bool m_if_conditon_is_true; ///< used to control if/elif/else logic
    bool m_enable_executing{
        false}; ///< if continue executing a container, this will be set true
                ///< after the paused block was executed
    bool m_is_executing_block{false}; ///< ExecuteScriptBlock was called instead
                                      ///< of ExecuteScriptContainer
    bool m_is_first_traj{
        true}; ///< Current trajectory is first one in the whole program
};

/// @}
} // namespace RVS
