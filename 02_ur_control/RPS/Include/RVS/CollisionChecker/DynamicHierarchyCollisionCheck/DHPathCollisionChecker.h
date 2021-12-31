// Copyright (c) RVBUST, Inc - All rights reserved.

#pragma once

#include <RVS/CollisionChecker/CollisionCheckerBase.h>
#include <RVS/CollisionChecker/PathCollisionCheckerBase.h>
#include "CollisionObjectCluster.h"

namespace RVS
{
/// @addgroup CollisionChecker
/// @{

class DHPathCollisionChecker : public PathCollisionCheckerBase
{
public:
    using OptiHierReps = CollisionObjectCluster::OptiHierReps;

    /**
     * @brief Construct a new DHPathCollisionChecker object
     *
     * @param collision_checker The off-the-shelf collision checker, i.e.,
     * FCL and Bullet, etc.
     * @param cont_dist_threshold The threshold of contact distance (see
     * class PathCollisionCheckerBase)
     * @param sample_interval The sampling interval of path in joint space
     * of robot arm
     *
     */
    DHPathCollisionChecker(CollisionCheckerBasePtr collision_checker,
                           double cont_dist_threshold = 0.0,
                           double sample_interval = 0.05);

    /**
     * @brief Initialize the collision detection environment
     * @see Refer to PathCollisionCheckerBase for the meanings of parameters
     *
     */
    virtual bool InitFromEnv(std::shared_ptr<Environment> env,
                             std::shared_ptr<Manipulator> manip) override;

    /**
     * @brief To reset all parameters, such as environment object
     *
     */
    virtual void Reset() override;

    /**
     * @brief Update the sampling interval of path in joint space of robot arm
     *
     * @param sample_interval The new value for the sampling interval of path
     *
     */
    void UpdatePathResolution(double sample_interval);

    /**
     * @brief Get the sampling interval of the path in joint space of robot arm
     *
     * @return double The current sampling interval of path
     *
     */
    double GetPathResolution() const { return m_sampling_interval; }

    /**
     * @brief To check whether there exists collision between manipulator and
     * objects in environment when the manipulator moves along given path
     * @see Refer to PathCollisionCheckerBase for the meanings of parameters
     *
     */
    bool CheckPathCollision(typename PathBase<JointVector>::ConstPtr joint_traj) override;

    /**
     * @brief To check whether there exists collision between manipulator and
     * objects in environment when the manipulator moves along given linear path
     * @see Refer to PathCollisionCheckerBase for the meanings of parameters
     *
     */
    bool CheckLinearPathCollision(const JointVector &start_point,
                                  const JointVector &final_point) override;

private:
    CollisionCheckerBasePtr
        m_collision_checker; ///< The off-the-shelf collision checker, i.e.,
                             ///< Collision Checker in FCL or Bullet
    std::shared_ptr<FixedDynamicObjectCluster>
        m_cluster_manipulator; ///< The cluster consisting of links of robot arm
    double m_sampling_interval; ///< The sampling interval of joint path of
                                ///< robot arm
    static const double
        m_resolution_upper_bound; ///< the lower bound of sampling interval of
                                  ///< joint path
    static const double m_resolution_lower_bound; ///< its upper bound
    std::vector<std::shared_ptr<FixedStaticObjectCluster>>
        m_cluster_obstacles; ///< The cluster comprising fixed static obstacles
};


/// @}

} // namespace RVS
