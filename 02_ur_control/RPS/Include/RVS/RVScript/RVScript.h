// Copyright (c) RVBUST, Inc - All rights reserved.
/*
we need to create a ScriptContainer class to hold controller UI code block
controller UI code block is a description about an operation, eg.
  - TrajectoryData, for trajectory
  - IOOperation, for IO
  - Logic, for logic control, eg. if/else, for, while
each code block is a struct, contains UICmd, operation arguments
======================================================================================================
function,   | "description",            command,     arg1,            arg2, arg3
======================================================================================================
Msg         | "Print logger"            "Msg"        msg
------------------------------------------------------------------------------------------------------
Traj,       | "trajectory data",                     traj_data
------------------------------------------------------------------------------------------------------
SetDO,      | "IO operation: SetDO",    "SetDO",     io_address,       io_val
WaitDI,     | "IO operation: WaitDI",   "WaitDI",    io_address,       io_val
timeout GetDI,      | "IO operation: GetDI",    "GetDI",     io_address, GetDO,
| "IO operation: GetDO",    "GetDO",     io_address,
------------------------------------------------------------------------------------------------------
if,         | "logic control: if",      "if",        condition*, conditon_result
statements elif,       | "logic control: elif",    "elif",      condition*,
conditon_result   statements else,       | "logic control: else",    "else",
statements while,      | "logic control: while",   "while",     condition*,
conditon_result   statements for,        | "logic control: for",     "for",
statements       loop_times, wait,       | "logic control: wait",    "wait",
timeout
======================================================================================================
*it seems that condition could only be "GetDI/GetDO" currently.
*statements could be another code container
*/
#pragma once
#include <vector>
#include <string>
#include <sstream>
#include <memory>
#include <algorithm>
#include <iostream>
#include <initializer_list>
#include <bitset>

#include <RVS/Trajectory/Waypoints.h>
namespace RVS
{
/// @addtogroup RVScript
/// @{

class ScriptContainer;
class ScriptBlock;
class ScriptExecutor;
using ScriptBlockPtr = std::shared_ptr<ScriptBlock>;
using ScriptContainerPtr = std::shared_ptr<ScriptContainer>;
using ScriptBlockList = std::list<ScriptBlockPtr>;

enum UICmdType
{
    UICmdType_Undef = -1, ///< Uninitialized/undefined cmd

    UICmdType_Msg = 0, ///< Log/Print message
    UICmdType_Comment = 1, ///< Comment

    UICmdType_SetDO = 10, ///< Set digital output
    UICmdType_GetDO, ///< Get digital output value
    UICmdType_GetDI, ///< Get digital input value
    UICmdType_WaitDI, ///< Wait until digital input = xxx value

    UICmdType_Traj = 20, ///< Execute trajectory
    UICmdType_TrajCircle, ///< Cartesian circle trajectory
    UICmdType_TrajPlanning, ///< planning traj

    UICmdType_If = 30, ///< Logic control if
    UICmdType_Elif, ///< Logic control elif
    UICmdType_Else, ///< Logic control else
    UICmdType_For, ///< Logic control for
    UICmdType_While, ///< Logic control while
    UICmdType_Wait, ///< Logic control wait

    UICmdType_Grab = 50, ///< Grab an object
    UICmdType_Release, ///< Release an object

    UICmdType_UpdateBodyPoseRemotely =
        60, ///< receive a pose from socket and update related body pose

    UICmdType_PyCode = 61, ///< contain some python code
    UICmdType_PyScript = 62, ///< contain a python scripy file
};

/**
 * @brief A ScriptBlock is basically conresponding to a ControllerUIApp teaching
 * instruction, eg. MoveJ/MoveL/MoveC, SetDO, GetDI, as well as if/while/for
 * block. After translated to python script, it contains one or more lines of
 * code. It is representation of a piece/block of program.
 *
 */
struct ScriptBlock : public std::enable_shared_from_this<ScriptBlock>
{
    friend class ScriptContainer;
    friend class ScriptExecutor;

    ///@brief Block that doesn't include valid sub-container, exclude the most
    /// Logic blocks
    bool IsLeaf() const
    {
        return UICmdType_If > this->cmd_type
               || this->cmd_type >= UICmdType_Wait;
    }

    ScriptBlockPtr Copy() const;

    /// @brief String expression of this block
    std::string Repr(bool brief = true) const;

    /// @brief Reset all member variables
    void Clear();

    /**
     * @brief Create RVScript program of this block
     *
     * @param prog[out] : python program
     * @return true
     * @return false
     */
    bool ToScript(std::string &prog) const;

    ///@brief Get parent container pointer, check if it is nullptr before use it
    ScriptContainerPtr GetParentContainer() const;

    ///@brief If the block is being executing
    bool IsRunning() const;

    ///@brief If container executing is paused here last time
    bool IsPausedHere() const;

    ///@brief Create a shared_ptr holding empty ScriptBlock
    static ScriptBlockPtr CreateBlockEmpty();

    /**
     * @brief Build a grab object block
     *
     * @param object_name
     * @param io_addr
     * @param io_val
     * @return ScriptBlockPtr
     */
    static ScriptBlockPtr CreateBlockGrab(const std::string &object_name,
                                          const int io_addr, const bool io_val);

    /**
     * @brief Build a release object block
     *
     * @param object_name
     * @param io_addr
     * @param io_val
     * @return ScriptBlockPtr
     */
    static ScriptBlockPtr CreateBlockRelease(const std::string &object_name,
                                             const int io_addr,
                                             const bool io_val);

    ///@breif Build a Commnet ScriptBlock.
    static ScriptBlockPtr CreateBlockCommnet(const std::string &comment);

    /**
     * @brief Build a print msg.
     *
     * @param msg : message need to be print out
     * @return ScriptBlockPtr
     */
    static ScriptBlockPtr CreateBlockMsg(const std::string &msg);

    /**
     * @brief Build a trajectory block. Traj block is to make a
     * movel/movej/movec movement.
     *
     * @param traj_data : trajectory data
     * @return ScriptBlockPtr
     */
    static ScriptBlockPtr
    CreateBlockTraj(std::shared_ptr<TrajectoryData> traj_data = nullptr);

    /**
     * @brief Create a cartesian circle trajectory from arc start point, middle
     * point and end point.
     *
     * @param p0 circle arc start point
     * @param p1 circle arc middle point
     * @param p2 circle arc end point
     * @param speed trajectory speed, if speed > 0, it is constant speed value;
     * if speed < 0, time optimal algorithm will be applied.
     * @param is_full_circle whether it is a full closed circle or an open
     * circle arc
     * @return ScriptBlockPtr
     */
    static ScriptBlockPtr
    CreateBlockTrajCircle(WaypointPtr p0, WaypointPtr p1, WaypointPtr p2,
                          const double speed = 0.1,
                          const bool is_full_circle = false);

    static ScriptBlockPtr CreateBlockTrajPlanning(WaypointPtr start,
                                                  WaypointPtr goal);

    /**
     * @brief Build a a wait block.
     *
     * @param timeout : wait time in seconds, timeout will be clamped to [0,
     * inf)
     * @return ScriptBlockPtr
     */
    static ScriptBlockPtr CreateBlockLogicWait(const double timeout);

    /**
     * @brief Build a SetDO block.
     *
     * @param io_addr : IO addresss
     * @param io_val : IO value
     * @return ScriptBlockPtr
     */
    static ScriptBlockPtr CreateBlockSetDO(const int io_addr,
                                           const bool io_val);

    /**
     * @brief Build a GetDI block.
     *
     * @param io_addr : IO address
     * @return ScriptBlockPtr
     */
    static ScriptBlockPtr CreateBlockGetDI(const int io_addr);

    /**
     * @brief Build a GetDO block.
     *
     * @param io_addr : IO address
     * @return ScriptBlockPtr
     */
    static ScriptBlockPtr CreateBlockGetDO(const int io_addr);

    /**
     * @brief Build a WaitDI block.
     *
     * @param io_addr : IO address
     * @param io_val : IO value
     * @param timeout : most waiting time in seconds, if negative, waiting time
     * is not limited
     * @return ScriptBlockPtr
     */
    static ScriptBlockPtr CreateBlockWaitDI(const int io_addr,
                                            const bool io_val,
                                            const double timeout = -1.0);

    /**
     * @brief Build a If block.
     *
     * @param condition : condition of if block, currently should contains only
     * one GetDI/GetDO ScriptBlock.
     * @param condition_result : required condition result
     * @param statements : statements to be executed when condiont equals to
     * conditon result.
     * @return ScriptBlockPtr
     */
    static ScriptBlockPtr CreateBlockLogicIf(ScriptBlockPtr condition,
                                             bool condition_result,
                                             ScriptContainerPtr statements);

    /**
     * @brief Build a Elif block.
     *
     * @param condition : condition of elif block, currently should contains
     * only one GetDI/GetDO ScriptBlock.
     * @param condition_result : required condition result
     * @param statements : statements to be executed when condiont equals to
     * conditon result.
     * @return ScriptBlockPtr
     */
    static ScriptBlockPtr CreateBlockLogicElif(ScriptBlockPtr condition,
                                               bool condition_result,
                                               ScriptContainerPtr statements);

    /**
     * @brief Build a Else block.
     *
     * @param statements : statements to be executed
     * @return ScriptBlockPtr
     */
    static ScriptBlockPtr CreateBlockLogicElse(ScriptContainerPtr statements);

    /**
     * @brief Build a For block.
     *
     * @param statements : statements to be executed
     * @param loops : how many loops
     * @return ScriptBlockPtr
     */
    static ScriptBlockPtr CreateBlockLogicFor(ScriptContainerPtr statements,
                                              const int loops = 1);

    /**
     * @brief Build a While block.
     *
     * @param condition : condition of while block, currently should contains
     * only one GetDI/GetDO ScriptBlock.
     * @param condition_result : required condition result
     * @param statements : statements to be executed when condiont equals to
     * conditon result.
     * @return ScriptBlockPtr
     */
    static ScriptBlockPtr CreateBlockLogicWhile(ScriptBlockPtr condition,
                                                bool condition_result,
                                                ScriptContainerPtr statements);

    /**
     * @brief Create a Block Remote Update Body Pose object
     *
     * @param body_name body's name, whose pose will be updated
     * @param ip server IP address
     * @param port server port
     * @return ScriptBlockPtr
     */
    static ScriptBlockPtr
    CreateBlockRemoteUpdateBodyPose(const std::string &body_name,
                                    const std::string &ip = "127.0.0.1",
                                    unsigned port = 8888);

    /**
     * @brief Create a Block Python Script object
     *
     * @param codes python codes
     * @return ScriptBlockPtr
     */
    static ScriptBlockPtr CreateBlockPyCode(const std::string &codes);

    /**
     * @brief Create a Block Python Script file
     *
     * @param  file_path file path
     * @return ScriptBlockPtr
     */
    static ScriptBlockPtr CreateBlockPyScript(const std::string &file_path);

public:
    // common data
    UICmdType cmd_type;

    // store a common string data, eg. for Msg/Commnet/Grab/Release block
    std::string data_str1;
    std::string data_str2;
    int data_int1{0};
    int data_int2{0};
    bool data_bool1{false};
    double data_double1{0};

    // trajectory block
    std::shared_ptr<TrajectoryData> traj_data;

    // comment block
    std::string &comment{data_str1};

    // msg block
    std::string &msg{data_str1};

    // grab/release block
    std::string &object_name{data_str1};

    // IO related blocks
    int &io_addr{data_int1}; ///< IO address
    bool &io_val{data_bool1}; ///< IO value
    double &timeout{data_double1}; ///< WaitDI/Wait timeout argument

    // update remote block
    std::string &body_name{data_str1}; ///< body name
    std::string &ip{data_str2}; ///< socket server ip
    int &port{io_addr}; ///< socket server port

    // python code block
    std::string &py_codes{data_str1}; ///< python codes

    // python script block
    std::string &py_script{data_str1}; ///< python script file path

    // logic controlling blocks
    ScriptBlockPtr condition; ///< logic if/elif/while conditon
    ScriptContainerPtr statements; ///< logic if/elif/else/while/for statements
    bool &condition_result{data_bool1}; ///< logic if/elif/while conditon result
    int &loops{data_int1}; ///< logic for loops
    int &current_loop_number{data_int2}; ///< current loop number

    ScriptContainerPtr
        elif_container; ///< if block's attribute, store elif options
    ScriptContainerPtr
        else_container; ///< if block's attribute, store only one else option

private:
    /// @breif Default constructor
    ScriptBlock();

    /// @brief Check if input could be used as a valid condition.
    static bool _IsConditionValid(ScriptBlockPtr condition);
    std::weak_ptr<ScriptContainer>
        parent_container; ///< parent container pointer, only ScriptContainer
                          ///< can set this value
};

std::ostream &operator<<(std::ostream &out, const ScriptBlock &block);

/**
 * @brief ScriptContainer is a container to holding all ScriptBlocks created by
 * user with ControllerUIApp. We can use it to create python script as well as
 * specific robot job. It is representation of whole program.
 *
 */
class ScriptContainer : public std::enable_shared_from_this<ScriptContainer>
{
    friend class ScriptBlock;
    friend class ScriptExecutor;

public:
    ///@brief Create a default empty ScriptContainer shared_ptr
    static ScriptContainerPtr
    Create(const std::string &name = "ScriptContainer");

    ///@brief Create a ScriptContainer shared_ptr from block
    static ScriptContainerPtr Create(ScriptBlockPtr block);

    ///@brief Create a ScriptContainer shared_ptr from a list of blocks
    static ScriptContainerPtr Create(ScriptBlockList blocks);

    ///@brief Get running script block.
    ScriptBlockPtr GetRunningBlock() const;

    ///@brief Copy a ScriptContainer shared_ptr form current one
    ScriptContainerPtr Copy() const;

    ///@brief ScirptContainer content brief intrduction
    std::string Repr() const;

    ///@brief Serialize to json string
    std::string ToJson() const;

    ///@brief Deserialize from json string
    bool FromJson(const std::string &json_str) noexcept;

    /**
     * @brief Add a ScriptBlock
     *
     * @param block : code block
     * @param idx : inserting position, [-inf, inf], negative number or >=
     * m_code_block.size() will append to tail
     * @return ScriptBlockPtr : the ScriptBlock shared pointer that inserted to
     * this container
     */
    ScriptBlockPtr InsertScriptBlock(ScriptBlockPtr block, const int idx = -1);

    /**
     * @brief Inserting a ScriptBlock after a given ScriptBlock
     *
     * @param block : ScriptBlock to be inserted
     * @param ref_block : ScriptBlock used to locating inserting position, if
     * ref_block not in container, false returned, if ref_block is nullptr,
     * inserting to tail
     * @return ScriptBlockPtr : the ScriptBlock shared pointer that inserted to
     * this container
     */
    ScriptBlockPtr InsertScriptBlock(ScriptBlockPtr block, bool after,
                                     ScriptBlockPtr ref_block = nullptr);

    /**
     * @brief Adjust blocks sequence, moving the given block one step backward,
     * eg. [b1, b2], MoveUp(b2)->[b2, b1]
     *
     * @param block : script block need to be moved
     * @return true
     * @return false
     */
    bool MoveUp(ScriptBlockPtr block);

    /**
     * @brief Adjust blocks sequence, moving the given block one step foreward,
     * eg. [b1, b2], MoveDown(b1)->[b2, b1]
     *
     * @param block : script block need to be moved
     * @return true
     * @return false
     */
    bool MoveDown(ScriptBlockPtr block);

    /**
     * @brief Delete a ScriptBlock, if container only has one block,  after
     * deleting, a Commnet block will be automatically added
     *
     * @param block
     * @return ScriptBlockPtr : the next block or the previous block if deleted
     * block is last one
     */
    ScriptBlockPtr Delete(ScriptBlockPtr block);

    /**
     * @brief Insert another ScriptContainer
     *
     * @param block_container : code block container
     * @param idx : index of inserting position
     */
    bool InsertScriptContainer(ScriptContainerPtr block_container,
                               const int idx = -1);

    /// @brief Clear all code blocks
    void Clear();

    /// @brief Get code bock vector
    ScriptBlockList &GetScriptBlocks() const;

    /**
     * @brief Create python program from current code blocks
     *
     * @param script[out] : create RVScript
     * @return true
     * @return false
     */
    bool ToScript(std::string &script) const;

    /**
     * @brief Convert to script and save i
     *
     * @param file_path[in,out] : script file save path, eg.
     * "/home/rvbust/Test.rvscript", if there is no extension name, a
     * ".rvscript" will be appended
     * @return true
     * @return false
     */
    bool SaveAsScript(std::string &file_path) const;

    /// @brief Append ScriptBlock
    ScriptContainer &operator+=(ScriptBlockPtr block);

    /// @brief Append ScriptContainer
    ScriptContainer &operator+=(ScriptContainerPtr other_container);

    /**
     * @brief Index to a ScriptBlock
     *
     * @param idx : index, if idx < 0, the last element will be returned, if idx
     * >= size of elements, an out of range exception will be thrown
     * @return const ScriptBlock&
     */
    ScriptBlockPtr operator[](const int idx) const;

    ///@brief Get parent block pointer, check if it is nullptr before use it.
    ScriptBlockPtr GetParentBlock() const;

    ///@brief Is the block is in the same ScriptContainer tree
    bool IsSameRoot(ScriptBlockPtr block);

    ///@brief Is the container is in the same ScriptContainer tree
    bool IsSameRoot(ScriptContainerPtr block);

    std::string GetType()
    {
        return std::vector<std::string>{"normal", "elif", "else"}[m_type];
    }

    void SetName(const std::string &name);
    const std::string &GetName() const;

private:
    enum ContainerType
    {
        ContainerType_Normal = 0,
        ContainerType_Elif,
        ContainerType_Else,
    };

    ScriptContainer(const std::string &name = "ScriptContainer") : m_name(name)
    {
    }

    /**
     * @brief Get m_code_block iterator at idx
     *
     * @param idx : [-inf, inf], negative number or >= m_code_block.size() will
     * return end iterator.
     * @return std::list<ScriptBlock>::iterator
     */
    ScriptBlockList::iterator _GetIteratorAt(const int idx) const;

    /**
     * @brief Inserting a ScriptBlock to container. Modification of
     * m_code_blocks should finally call the function, instead of operating
     * m_code_blocks directly.
     *
     * @param block : script block to be insterted
     * @param it : position to be inserted
     * @return ScriptBlockPtr : the ScriptBlock shared pointer that inserted to
     * this container
     */
    ScriptBlockPtr _InsertScriptBlock(ScriptBlockPtr block,
                                      ScriptBlockList::const_iterator it);

private:
    std::string m_name{"ScriptContainer"}; ///< container name
    ContainerType m_type{ContainerType_Normal};
    std::weak_ptr<ScriptBlock>
        m_parent_block_ptr; ///< parent block pointer, only ScriptBlock can set
                            ///< this value
    mutable ScriptBlockList m_code_blocks; ///< list of including ScriptBlocks
    mutable ScriptBlockPtr
        m_running_block; ///< currently being executing block of the container
    mutable ScriptBlockPtr
        m_paused_block; ///< block being executing when user paused the program
    mutable bool m_start_executing{
        false}; ///< if continue executing, this will be set true after paused
                ///< block being executed; if executing from start, this will be
                ///< set true immediately
};

std::ostream &operator<<(std::ostream &out, const ScriptContainer &container);

/// @}
} // namespace RVS