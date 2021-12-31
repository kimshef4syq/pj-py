// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include <RVS/Common/Utils.h>
#include <RVS/Common/LoggerUtils.h>
#include <RVS/Common/ObserverAndSubject.h>
#include <RVS/LieGroupHeader.h>
#include <RVS/Environment/CollisionMatrix.h>

namespace RVS
{
/// @addtogroup CollisionChecker
/// @{

class Link;

enum CollisionOptions
{
    CollisionOptions_Collision = 0, ///< Return bool collision result
    CollisionOptions_All = 1, ///< Return all collision object
    CollisionOptions_Contacts = 2, ///< Return the Contact points of the
                                   ///< collision in the @ref CollisionReport
    CollisionOptions_Distance = 3, ///< Compute distance measurements
    CollisionOptions_Gradient = 4, ///< Compute the gradients
};

enum ContinuousCollisionOptions
{
    ContinuousCollisionOptions_First =
        0, /// Return at first contact for any time
    ContinuousCollisionOptions_Closest = 1, /// Return the global minimum time
};

struct CollisionReportWithPair
{
    std::shared_ptr<Link const>
        m_link0; ///< the first colliding link if a collision involves a bodies.
    std::shared_ptr<Link const> m_link1; ///< the second colliding link if a
                                         ///< collision involves a bodies.
    double m_min_distance; ///< minimum distance from last query, filled if
                           ///< CollisionOptions_Distance option is set
    CVec3d m_nearest_points0; ///< nearest points in m_link0
    CVec3d m_nearest_points1; ///< nearest points in m_link1
    double m_collision_time; ///< For continuous collision checking, return
                             ///< collision time
    SE3d m_contact_pose0;
    SE3d m_contact_pose1;
    std::string m_link_name0; ///< The name of link0 from the pair of objects
    std::string m_link_name1; ///< The name of link1 from the pair of objects
    CVec3d m_contact_normal; ///< The contact normal with the direction from
                             ///< link0 to link1
    double m_contact_depth; ///< the penetration m_depth, m_positive means
                            ///< the surfaces are penetrating, negative
                            ///< means the surfaces are not colliding (used
                            ///< for distance queries)
public:
    friend std::ostream &
    operator<<(std::ostream &out,
               const CollisionReportWithPair &collision_report_config);

    ///@brief To construct a collision report with a pair of objects and reset
    /// all its variables to initial states
    CollisionReportWithPair()
    {
        RVS_TRACE("Initializing CollisionReportWithPair...");
        this->Reset();
    }

    ~CollisionReportWithPair()
    {
        RVS_TRACE("Destructing CollisionReportWithPair.");
    }

    ///@brief To reset the collision report of the pair of objects to initial
    /// state
    void Reset();

private:
    /**
     * @brief To simplify the names of two collision objects
     *
     * @param obj_name The name of object who is collided with another one
     * @return std::string The simplified object name
     */
    std::string
    _SimplifiedCollisionObjectName(const std::string &obj_name) const;
};

/**
 * @brief Collision Checker report
 *
 */
struct CollisionReport
{
    std::vector<CollisionReportWithPair>
        m_collision_report; ///< To hold collision information from all
                            ///< collided pair of objects
    CollisionOptions
        m_options; ///< the m_options that the CollisionReport was called with.
    ContinuousCollisionOptions m_cc_options; ///< Continuous collision options

    bool m_is_collision; ///< The flag that denotes whether or not there exists
                         ///< collision among objects

public:
    friend std::ostream &operator<<(std::ostream &out,
                                    const CollisionReport &collision_report);

    ///@brief To construct a collision report object and reset its variables to
    /// initial states
    CollisionReport()
    {
        RVS_TRACE("Constructing CollisionReport");
        this->Reset();
    }

    ~CollisionReport() { RVS_TRACE("Destructing CollisionReport."); }

    ///@brief To reset collision report to initial state
    void Reset();
};

using IsContactAllowedFn =
    std::function<bool(const std::string &, const std::string &)>;

/// Contact test data and query results information
struct CollisionCheckData
{
    CollisionCheckData(const CollisionMatrix &collision_matrix,
                       const double &contact_distance,
                       const IsContactAllowedFn &fn, CollisionReport &report)
        : collision_matrix(collision_matrix),
          contact_distance(contact_distance), fn(fn), report(report),
          done(false)
    {
    }
    const CollisionMatrix &collision_matrix;
    const double &contact_distance;
    const IsContactAllowedFn &fn;
    CollisionReport &report;
    bool done; /// Indicate if search is finished
};

inline bool IsContactAllowed(const std::string &name1, const std::string &name2,
                             const IsContactAllowedFn acm, bool verbose = false)
{
    // Do not distance check geoms part of the same object / link / attached
    // body
    if (name1 == name2) return true;

    if (acm != nullptr && acm(name1, name2)) {
        if (verbose) {
            RVS_INFO("Collision between '{}' and '{}' is allowed. No contacts "
                     "are computed.",
                     name1.c_str(), name2.c_str());
        }
        return true;
    }
    if (verbose) {
        RVS_INFO("Actually checking collisions between {} and {}",
                 name1.c_str(), name2.c_str());
    }

    return false;
}

/// @}
} // namespace RVS