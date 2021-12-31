// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include <RVS/Trajectory/TrajectoryWithPath.h>
#include <RVS/Environment/Environment.h>
#include <Json11/json.hpp>
#include <map>
#include <variant>

namespace RVS
{
/// @addtogroup Trajectory
/// @{

enum MotionType
{
    MotionType_MoveJ = 0, ///< Joint trajectory
    MotionType_MoveL = 1, ///< Pose linear trajectory
    MotionType_MoveC = 2, ///< Pose circular trajectory
    // MotionType_MoveP = 3
};

enum WaypointType
{
    WaypointType_Joint = 0,
    WaypointType_Cartesian,
};

class TrajectoryData;

RVS_CLASS_FORWARD(Waypoint);

struct Waypoint
{
    friend class TrajectoryData;
    friend void to_json(Json11::json &j, const Waypoint &waypoint);
    friend void from_json(const Json11::json &j, Waypoint &waypoint);
    friend std::ostream &operator<<(std::ostream &out,
                                    const Waypoint &waypoint);

public:
    /**
     * @brief Construct a new Waypoint object
     *
     * @param pose : pose at given frame, pose and joint should be consistent
     * @param joint_seed : joint values near the pose
     * @param frame_name : specific frame name
     */
    Waypoint(const SE3d &pose, const Rxd &joint_seed,
             const std::string &frame_name = GetWorldName());

    /**
     * @brief Construct a new Waypoint object
     *
     * @param joint : robot joint value
     */
    Waypoint(const Rxd &joint);

    ///@breif Copy a new Waypoint
    std::shared_ptr<Waypoint> Copy() const
    {
        return std::shared_ptr<Waypoint>(
            new Waypoint(type, joint, pose, frame_name, name, nullptr));
    }

    ///@brief Get cartesian space value of the Waypoint
    std::pair<RVSReturn, SE3d>
    GetPose(std::shared_ptr<KinematicsBase> kin_solver,
            EnvironmentPtr env = nullptr) const;

    ///@brief Get joint space value of the Waypoint
    std::pair<RVSReturn, Rxd>
    GetJoint(std::shared_ptr<KinematicsBase> kin_solver,
             EnvironmentPtr env = nullptr) const;

    /**
     * @brief Get the TrajectoryData instance shared pointer that the waypoint
     * is associated
     *
     * @return std::shared_ptr<TrajectoryData>
     */
    std::shared_ptr<TrajectoryData> GetTrajData() const
    {
        return traj_data.lock();
    }

    WaypointType &Type()
    {
        _SetTrajectoryDataChanged();
        return this->type;
    }

    const WaypointType &Type() const { return this->type; }
    Rxd &Joint()
    {
        _SetTrajectoryDataChanged();
        return this->joint;
    }

    const Rxd &Joint() const { return this->joint; }

    SE3d &Pose()
    {
        _SetTrajectoryDataChanged();
        return this->pose;
    }

    const SE3d &Pose() const { return this->pose; }
    std::string &FrameName()
    {
        _SetTrajectoryDataChanged();
        return this->frame_name;
    }

    const std::string &FrameName() const { return this->frame_name; }
    std::string &Name() const { return this->name; }

    void SetUserData(void *data);
    void *GetUserData() const;

private:
    mutable WaypointType type{WaypointType_Joint}; ///< waypoint type
    mutable Rxd joint; ///< joint space value
    mutable SE3d pose{SE3d::IdentityStatic()}; ///< cartesian space pose
    mutable std::string frame_name{
        GetWorldName()}; ///< waypoint reference frame
    mutable std::string name{"waypoint"}; ///< waypoint name
    void *user_data;

private:
    /**
     * @brief Construct a new Waypoint, only could be called from friend class
     * TrajectoryData
     *
     * @note traj_data should be associated with a TrajectoryData class
     * instance, so user is not allowed to modify traj_data
     * @param joint robot joint value
     * @param pose pose at given frame
     * @param frame_name specific frame name
     * @param name waypoint name
     * @param traj_data TrajectoryData instance that will include this waypoint
     */
    Waypoint(const WaypointType type, const Rxd &joint, const SE3d &pose,
             const std::string &frame_name, const std::string &name,
             std::shared_ptr<TrajectoryData> traj_data);

    ///@brief If the traj data exists, set its is_trajectory_data_updated flag
    /// to be true when the waypoint is changed.
    void _SetTrajectoryDataChanged();

    mutable std::weak_ptr<TrajectoryData>
        traj_data; ///< trajectory data pointer that this waypoint belongs to
};

class TrajectoryData : public std::enable_shared_from_this<TrajectoryData>
{

public:
    /**
     * @brief Construct a new Trajectory Data object
     *
     * @param blend_tolerance : path blend_tolerance
     * @param motion_type : MoveC/MoveJ/MoveL
     * @param is_constant_velocity : whether trajectory is constant velocity
     * @param velocity : if constant velocity, define the velocity
     * @param traj_type : trajectory type
     * @param name : trajectory name
     */
    TrajectoryData(const double blend_tolerance = 0.1,
                   const MotionType motion_type = MotionType_MoveJ,
                   const bool is_constant_velocity = false,
                   const double velocity = 0.6,
                   const TrajType traj_type = TrajType_Trapezoidal,
                   const std::string &name = "Trajectory");

    std::shared_ptr<TrajectoryData> Copy() const;

    const std::string &GetName() const { return m_name; }

    void SetName(const std::string &name) { m_name = name; }

    /**
     * @brief Update path blend tolerance
     *
     * @param blend_tolerance
     */
    void SetBlendTolerance(const double blend_tolerance)
    {
        if (blend_tolerance != m_blend_tolerance) {
            m_is_traj_data_updated = true;
            m_blend_tolerance = blend_tolerance;
        }
    }

    double GetBlendTolerance() const { return m_blend_tolerance; }

    /**
     * @brief Update motion type
     *
     * @param motion_type
     */
    void SetMotionType(const MotionType motion_type)
    {
        if (motion_type != m_motion_type) {
            m_is_traj_data_updated = true;
            m_motion_type = motion_type;
        }
    }

    MotionType GetMotionType() const { return m_motion_type; }

    /**
     * @brief Update whether using constant velocity
     *
     * @param is_constant_velocity
     */
    void SetIsConstantVelocity(const bool is_constant_velocity)
    {
        if (is_constant_velocity != m_is_constant_velocity) {
            m_is_traj_data_updated = true;
            m_is_constant_velocity = is_constant_velocity;
        }
    }

    bool GetIsConstantVelocity() const { return m_is_constant_velocity; }

    /**
     * @brief Update constant velocity value
     *
     * @param velocity
     */
    void SetVelocity(const double velocity)
    {
        if (velocity != m_velocity) {
            m_is_traj_data_updated = true;
            m_velocity = velocity;
        }
    }

    double GetVelocity() const { return m_velocity; }

    ///@brief Update trajectory type
    void SetTrajType(const TrajType traj_type)
    {
        if (traj_type != m_traj_type) {
            m_is_traj_data_updated = true;
            m_traj_type = traj_type;
        }
    }

    TrajType GetTrajType() const { return m_traj_type; }

    ///@brief Update is full circle
    void SetIsFullCircle(bool is_full_circle)
    {
        if (is_full_circle != m_is_full_circle) {
            m_is_traj_data_updated = true;
            m_is_full_circle = is_full_circle;
        }
    }

    bool GetIsFullCircle() const { return m_is_full_circle; }

    /**
     * @brief Append a waypoint at end
     *
     * @param waypoint : a copy of waypoint will be appended to the waypoint
     * list tail
     * @return std::shared_ptr<Waypoint>
     */
    std::shared_ptr<const Waypoint>
    AddWaypoint(std::shared_ptr<const Waypoint> waypoint);

    /**
     * @brief Append a waypoint at end
     *
     * @param joint : waypoint joint values
     * @return std::shared_ptr<Waypoint>
     */
    std::shared_ptr<const Waypoint> AddWaypoint(const CVecXd &joint);
    std::shared_ptr<const Waypoint> AddWaypoint(const Rxd &joint);

    /**
     * @brief Append a waypoint at end
     *
     * @param pose : waypoint pose value
     * @param joint_seed : close joint values of current pose
     * @param frame_name : pose reference frame
     * @return std::shared_ptr<Waypoint>
     */
    std::shared_ptr<const Waypoint>
    AddWaypoint(const SE3d &pose, const Rxd &joint_seed,
                const std::string &frame_name = GetWorldName());

    /**
     * @brief Insert a waypoint at given index
     *
     * @param idx : index within range [0, inf), any number >=
     * m_waypoints.size() will append waypoint at last position
     * @param waypoint a copy of waypoint will be inserted to given position
     * @return std::shared_ptr<Waypoint>
     */
    std::shared_ptr<const Waypoint>
    InsertWaypoint(unsigned idx, std::shared_ptr<const Waypoint> waypoint);

    /**
     * @brief Insert a copy of given waypoint to position after/before
     * ref_waypoint
     *
     * @param waypoint : waypoint to be inserted
     * @param after : whether after the ref_waypoint or before it
     * @param ref_waypoint : reference waypoint position
     * @return std::shared_ptr<Waypoint> : waypoint inserted
     */
    std::shared_ptr<const Waypoint>
    InsertWaypoint(std::shared_ptr<const Waypoint> waypoint, bool after = true,
                   std::shared_ptr<const Waypoint> ref_waypoint = nullptr);

    /**
     * @brief Insert a waypoint at given index
     *
     * @param idx : index within range [0, inf), any number >=
     * m_waypoints.size() will append waypoint at last position
     * @param joint : waypoint joint value
     * @return std::shared_ptr<Waypoint>
     */
    std::shared_ptr<const Waypoint> InsertWaypoint(unsigned idx,
                                                   const Rxd &joint);

    /**
     * @brief Insert a waypoint at given index
     *
     * @param idx : index within range [0, inf), any number >=
     * m_waypoints.size() will append waypoint at last position
     * @param pose : waypoint pose value
     * @param joint_seed : close joint values of current pose
     * @param frame_name : pose reference frame
     * @return std::shared_ptr<Waypoint>
     */
    std::shared_ptr<const Waypoint>
    InsertWaypoint(unsigned idx, const SE3d &pose, const Rxd &joint_seed,
                   const std::string &frame_name = GetWorldName());

    /**
     * @brief Update waypoint with given waypoint data
     *
     * @param old_waypoint : waypoint in traj_data
     * @param new_waypoint : waypoint having new data
     * @return true
     * @return false
     */
    bool UpdateWaypoint(std::shared_ptr<const Waypoint> old_waypoint,
                        std::shared_ptr<const Waypoint> new_waypoint);

    /**
     * @brief Update a traj data waypoint joint
     *
     * @param waypoint
     * @param joint
     * @return true
     * @return false
     */
    bool UpdateWaypoint(std::shared_ptr<const Waypoint> waypoint,
                        const Rxd &joint);

    /**
     * @brief Update a traj data waypoint pose
     *
     * @param waypoint
     * @param pose
     * @param joint_seed
     * @param frame_name
     * @return true
     * @return false
     */
    bool UpdateWaypoint(std::shared_ptr<const Waypoint> waypoint,
                        const SE3d &pose, const Rxd &joint_seed,
                        const std::string &frame_name = GetWorldName());

    ///@brief Used to update waypoints pose and joint to be consistent
    bool SyncWaypoints(std::shared_ptr<KinematicsBase> kin_solver);

    /**
     * @brief Delete a waypoint in traj data
     *
     * @param waypoint
     * @return std::shared_ptr<Waypoint> : next waypoint, or nullptr if failed
     * or list is empty
     */
    std::shared_ptr<const Waypoint>
    DeleteWaypoint(std::shared_ptr<const Waypoint> waypoint);

    /**
     * @brief Move a waypoint forward one step in the waypoints list
     *
     * @param waypoint
     * @return true
     * @return false
     */
    bool MoveUp(std::shared_ptr<const Waypoint> waypoint);

    /**
     * @brief Move a waypoint backward one step in the waypoints list
     *
     * @param waypoint
     * @return true
     * @return false
     */
    bool MoveDown(std::shared_ptr<const Waypoint> waypoint);

    /**
     * @brief Clear all the waypoints in traj data
     *
     * @note The traj_data weak_ptr stored in waypoint will also be reset
     *
     */
    void ClearWaypoints()
    {
        m_is_traj_data_updated = true;
        for_each(m_waypoints.begin(), m_waypoints.end(),
                 [](std::shared_ptr<Waypoint> waypoint) {
                     waypoint->traj_data.reset();
                 });
        m_waypoints.clear();
    }

    /**
     * @brief Get the waypoints in traj data
     *
     * @return std::list<std::shared_ptr<Waypoint>>
     */
    const std::list<std::shared_ptr<Waypoint>> &GetWaypoints() const
    {
        return m_waypoints;
    }

    /**
     * @brief Generate a R3xSO3 trajectory when MotionType is MoveL/MoveC
     *
     * @param kin_solver kinematics solver
     * @param cart_limits cartesian limits for velocity, acceleration and jerk
     * of each dof
     * @return std::shared_ptr<TrajectoryBase<Pose>>
     */
    std::shared_ptr<TrajectoryBase<Pose>> GenerateCartesianTrajectory(
        std::shared_ptr<KinematicsBase> kin_solver,
        EnvironmentPtr env = nullptr,
        const Mat<double, 3, 6> &cart_limits = Mat<double, 3, 6>::Ones());

    /**
     * @brief Generate a Rn trajectory when MotionType is MoveJ
     *
     * @param kin_solver kinematics solver
     * @param joint_limits joint limits for velocity, acceleration and jerk of
     * each dof
     * @return std::shared_ptr<TrajectoryBase<Pose>>
     */
    std::shared_ptr<TrajectoryBase<Rxd>> GenerateJointTrajectory(
        std::shared_ptr<KinematicsBase> kin_solver,
        EnvironmentPtr env = nullptr,
        const MatXd &joint_limits = Mat<double, 0, 0>::Identity());

    ///@brief Get the flag of is_trajectory_data_updated
    bool GetIsTrajectoryDataUpdated() const;

    ///@brief Set the is_trajectory_data_updated flag, if true, it will
    /// re-compute the trajectory when GenerateXXXTrajectory is called, else the
    /// cached trajectory is returned.
    void SetIsTrajectoryDataUpdated(bool is_traj_data_updated);

    /**
     * @brief Whether a waypoint is in trajectory data waypoints list or not
     *
     * @param waypoint
     * @return true
     * @return false
     */
    bool IsIncludeWaypoint(std::shared_ptr<const Waypoint> waypoint) const
    {
        if (!waypoint) return false;
        return waypoint->GetTrajData() == shared_from_this();
    };

    /**
     * @brief Create executing trajectory python code that could be executed
     * with controller.ExecuteCommands function
     *
     * @return std::string
     */
    std::string
    ToPythonExpr(std::shared_ptr<KinematicsBase> kin_solver = nullptr);

    /**
     * @brief Get trajectory waypoint at given index, negative index is also
     * allowed, this function could throw an out of range exception
     *
     * @param idx : waypoint index, shoule be within range [-m_waypoints.size(),
     * m_waypoints.size()), or an out of range exception will be thrown
     * @return std::shared_ptr<Waypoint>
     */
    const std::shared_ptr<const Waypoint> operator[](int idx) const;

    static std::map<MotionType, const std::string> m_motion_type_str_expr;

private:
    double m_blend_tolerance; ///< path blend tolerance
    MotionType m_motion_type; ///< motion type
    bool m_is_constant_velocity; ///< whether using constant velocity
    double m_velocity; ///< constant velocity value
    TrajType m_traj_type; ///< traj type

    bool m_is_full_circle{false}; ///< whether a circle arc is full or closed,
                                  ///< valid when motion type is MoveC

    std::string m_name; ///< trajectory name

    /**
     * @brief Convert joint space waypoints to python list
     *
     * @return std::string
     */
    std::string _WaypointsToPythonListStrJoint() const;

    /**
     * @brief Convert cartesian space waypoints to python list
     *
     * @return std::string
     */
    std::string _WaypointsToPythonListStrPose() const;

    std::list<std::shared_ptr<Waypoint>> m_waypoints; ///< included waypoints

    bool m_is_traj_data_updated{true}; ///< whether the waypoints are updated
                                       ///< since GenerateTrajectory was called
    std::variant<std::shared_ptr<TrajectoryBase<Rxd>>,
                 std::shared_ptr<TrajectoryBase<Pose>>>
        m_cached_traj; ///< cached trajectory, if no changes in waypoints, avoid
                       ///< to recompute again
};

std::ostream &operator<<(std::ostream &out, const Waypoint &waypoint);
/// @}
} // namespace RVS