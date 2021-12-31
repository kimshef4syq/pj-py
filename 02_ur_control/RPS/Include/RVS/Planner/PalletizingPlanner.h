// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include <RVS/LieGroupHeader.h>
#include <RVS/Kinematics/KinematicsBase.h>
#include <RVS/CollisionChecker/CollisionCheckerBase.h>

namespace RVS
{
/// @addtogroup Planner
/// @{

/**
 * @brief Palletizing planner, input object's start and end pose, output robot's
 * pick/place pose or pick/place joint values
 *
 */
class PalletizingPlanner
{

public:
    PalletizingPlanner();

    PalletizingPlanner(std::shared_ptr<RobotModel> robot_model,
                       std::shared_ptr<KinematicsBase> kin_solver,
                       CollisionCheckerBasePtr checker,
                       std::shared_ptr<CollisionMatrix> collision_matrix);

    virtual ~PalletizingPlanner();

    RVSReturn
    SetCollisionMatrix(std::shared_ptr<CollisionMatrix> collision_matrix);

    RVSReturn SetKinSolver(std::shared_ptr<KinematicsBase> kin_solver);

    /**
     * @brief Plan pick/place tool pose with given obj_start_pose and target
     * pose, without collision checking and kinematics solver
     *
     * @param[in] object_start_pose: object start pose in robot_base
     * @param[in] object_target_pose: object target pose in robot_base
     * @param[out] pick_pose: pick_pose
     * @param[out] place_pose: place_pose
     * @return RVSReturn
     */
    RVSReturn Plan(const SE3d &object_start_pose,
                   const SE3d &object_target_pose, SE3d &pick_pose,
                   SE3d &place_pose);

    /**
     * @brief Plan collision_free and reachable pick/place joint_state
     *
     * @param[in] object_start_pose: object start pose in robot_base
     * @param[in] object_target_pose: object target pose in robot_base
     * @param[in] start_joint_ref: refrence joint value for start pose, usually
     * use best manipulality state near start area
     * @param[in] target_joint_ref: refrence joint value for target pose,
     * usually use best manipulality state near target area
     * @param[out] start_joint_value: collision_free start joint value
     * @param[out] target_joint_value: collision_free target joint value
     * @param[in] search_resolution: search resolution to search valid pick pose
     * (around Z-axis), default is pi/2
     * @return RVSReturn
     */
    RVSReturn
    Plan(const SE3d &object_start_pose, const SE3d &object_target_pose,
         const JointVector &start_joint_ref,
         const JointVector &target_joint_ref, JointVector &start_joint_value,
         JointVector &target_joint_value,
         const double search_resolution = Constants<double>::HalfPi());

private:
    /**
     * @brief Compute angle-axis between two given vectors
     *
     * @param[in] v1: first vector
     * @param[in] v2: second vector
     * @param[out] angle: angle from v1 to v2, around rotate_axis
     * @param[out] rotate_axis: (v1.cross(v2)).normalized()
     * @return RVSReturn
     */
    RVSReturn _VectorAngle(const CVec3d &v1, const CVec3d &v2, double &angle,
                           CVec3d &rotate_axis);

private:
    CollisionCheckerBasePtr m_checker; ///< collision checker
    std::shared_ptr<CollisionMatrix> m_collision_matrix; ///< collision matrix
    std::shared_ptr<RobotModel> m_robot_model; ///< RobotModel
    std::shared_ptr<KinematicsBase> m_kin_solver; ///< kinematics solver
};

/// @}
} // namespace RVS