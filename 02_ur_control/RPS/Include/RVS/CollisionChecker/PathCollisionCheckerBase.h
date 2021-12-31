// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once

#include <RVS/Environment/Manipulator.h>
#include <RVS/Environment/Geometry.h>
#include <RVS/Environment/Environment.h>
#include "Types.h"
#include <RVS/Trajectory/PathBase.h>
#include <RVS/Common/Macros.h>

namespace RVS
{
/// @addgroup CollisionChecker
/// @{

RVS_CLASS_FORWARD(PathCollisionCheckerBase);

/**
 * @brief The abstract base class PathCollisionCheckerBase is oriented toward
 * path collision detection in robot arm joint space
 *
 */
class PathCollisionCheckerBase
{
public:
    /**
     * @brief Construct a new path collision checker base object
     *
     * @param path_checker_name The name of path collision checker
     * @param cont_dist_threshold The threshold that indicates there exists
     * collision between two objects, i.e, If their minimum distance is less
     * than or equal to the threshold, The collision will be reported by the
     * checker
     *
     */
    PathCollisionCheckerBase(const std::string &path_checker_name,
                             const double &cont_dist_threshold = 0.0)
        : m_path_checker_name(path_checker_name)
    {
        RVS_DEBUG("Initialize the path collision checker base...");
        UpdateContDistThreshold(cont_dist_threshold);
    }

    // default destructor
    virtual ~PathCollisionCheckerBase()
    {
        RVS_DEBUG("Destructing path collision checker base...");
    }

    /**
     * @brief Initialize the collision detection environment
     *
     * @param env The environment that holds all objects that will be likely to
     * be performed collision check
     * @return true The result if it succeeds to initialing the collision
     * object environment
     * @return false The result if it fails to do it
     *
     */
    virtual bool InitFromEnv(std::shared_ptr<Environment> env,
                             std::shared_ptr<Manipulator> manip) = 0;

    /**
     * @brief To reset all parameters, such as environment object
     *
     */
    virtual void Reset() = 0;

    /**
     * @brief Get the name of path collision checker
     *
     * @return std::string The name of path collision checker
     *
     */
    std::string GetName() const { return m_path_checker_name; }

    /**
     * @brief To modify threshold of contact distance between two objects that
     * will trigger collision
     *
     * @param cont_dist_threshold The new threshold of contact distance
     *
     */
    void UpdateContDistThreshold(const double cont_dist_threshold)
    {
        if (cont_dist_threshold < 0.0) {
            RVS_WARN("The input contact distance threshold is less than 0.0! "
                     "It will be clamped into 0.0 or you can reset it.");
        }
        m_contact_distance_threshold = std::clamp(
            cont_dist_threshold, 0.0, std::numeric_limits<double>::infinity());
    }

    /**
     * @brief Get the contact distance threshold between two objects when they
     * are determined to be collided
     *
     * @return double The current contact distance threshold
     *
     */
    double GetContDistThreshold() const { return m_contact_distance_threshold; }

    /**
     * @brief To check whether there exists collision between manipulator and
     * objects in environment when the manipulator moves along given path
     *
     * @param joint_traj The joint trajectory that manipulator will follow
     * @return true The result if collision is reported
     * @return false The result if there no exists collision
     *
     */
    virtual bool
    CheckPathCollision(typename PathBase<JointVector>::ConstPtr joint_traj) = 0;

    /**
     * @brief To check whether there exists collision between manipulator and
     * objects in environment when the manipulator moves along given linear path
     *
     * @param start_point The start point of joint trajectory
     * @param final_point The final point of joint trajectory
     * @return true Similarly above
     * @return false Similarly above
     *
     */
    virtual bool CheckLinearPathCollision(const JointVector &start_point,
                                          const JointVector &final_point) = 0;

private:
    std::string m_path_checker_name; ///< the name of path collision checker
    double m_contact_distance_threshold; ///< the threshold that the collision
                                         ///< will be reported when the distance
                                         ///< between robot arm and obstacle is
                                         ///< less than or equal to it
};

/// @}
} // namespace RVS