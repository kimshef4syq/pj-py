// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include <RVS/Environment/Manipulator.h>
#include <RVS/Environment/Geometry.h>
#include <RVS/Environment/Environment.h>
#include "Types.h"
#include <RVS/Common/Macros.h>

namespace RVS
{

/// @addtogroup CollisionChecker
/// @{

RVS_CLASS_FORWARD(CollisionCheckerBase);

/// @brief CollisionChecker interface
class CollisionCheckerBase : public ObserverBase
{
public:
    /**
     * @brief Construct a new Collision Checker Base object
     *
     */
    CollisionCheckerBase()
    {
        m_contact_distance = 0;
        RVS_TRACE("Constructing CollisionCheckerBase()");
    }

    /**
     * @brief Destroy the Collision Checker Base object
     *
     */
    virtual ~CollisionCheckerBase()
    {
        RVS_TRACE("Destructing CollisionCheckerBase");
    }

    /**
     * @brief Clone the collisinchecker all information.
     *
     * @return CollisionCheckerBasePtr
     */
    virtual CollisionCheckerBasePtr Clone() const = 0;

    /**
     * @brief add collision objejct from Environment
     *
     * @param env
     * @return true
     * @return false
     */
    virtual bool InitFromEnv(std::shared_ptr<Environment> env) = 0;

    /**
     * @brief clear all collision objects
     *
     */
    virtual void Clear() = 0;

    /**
     * @brief add collison oject from Link
     *
     * @param link
     * @param enabled whether if check collision between this obj with others.
     * @return true
     * @return false
     */
    virtual bool AddCollisionObject(const Link &link, bool enabled = true) = 0;

    /**
     * @brief add collion object by geometry info
     *
     * @param name the objec name,should be unique
     * @param shape
     * @param shape_pose
     * @param enabled
     * @return true
     * @return false
     */
    virtual bool AddCollisionObject(const std::string &name,
                                    std::shared_ptr<Geometry> shape,
                                    const SE3d &shape_pose,
                                    bool enabled = true) = 0;


    /**
     * @brief To add collision object into FCL manager
     *
     * @param mul The object waiting for checking collision with others
     * @param enabled  Determine whether or not object will be checked collision
     * with other. If it is true, the object will be checked
     * @param add_into_env Check if the body will added into corresponding
     * environment when adding object into fcl manager
     * @return true Succeed in adding object into the managers
     * @return false Fail to do it
     */
    virtual bool AddCollisionObject(std::shared_ptr<Multibody const> mul,
                                    bool enabled = true,
                                    bool add_into_env = true) = 0;

    /**
     * @brief remove obj by name
     *
     * @param name
     * @return true
     * @return false
     */
    virtual bool RemoveCollisionObject(const std::string &name) = 0;


    /**
     * @brief remove link
     *
     * @param link
     * @return true
     * @return false
     */
    virtual bool RemoveCollisionObject(const Link &link) = 0;

    /**
     * @brief To remove collision object from FCL manager
     *
     * @param mul The object that will be removed from the manager
     * @param remove_from_env Determine whether the body will be removed from
     * corresponding environment when removing collision object from checker
     * @return true Succeed in removing the object
     * @return false Fail to do it
     */
    virtual bool RemoveCollisionObject(std::shared_ptr<Multibody const> mul,
                                       bool remove_from_env = true) = 0;

    /**
     * @brief check whether has collision object
     *
     * @param name
     * @return true
     * @return false
     */
    virtual bool HasCollisionObject(const std::string &name) const = 0;

    virtual bool HasCollisionObject(const Link &link) const = 0;

    /**
     * @brief enalbe collislon object to check collision
     *
     * @param name
     * @return true
     * @return false
     */
    virtual bool EnableCollisionObject(const std::string &name) = 0;

    /**
     * @brief disable collision Object check collision
     *
     * @param name
     * @return true
     * @return false
     */
    virtual bool DisableCollisionObject(const std::string &name) = 0;

    /**
     * @brief Set the Collision Objects Transform object
     *
     * @param name
     * @param pose
     */
    virtual void SetCollisionObjectsTransform(const std::string &name,
                                              const SE3d &pose) = 0;

    /**
     * @brief Set the Contact Distance Threshold object
     *
     * @param contact_distance
     */
    virtual void SetContactDistanceThreshold(const double contact_distance)
    {
        if (contact_distance >= 0) {
            m_contact_distance = contact_distance;
        }
        else {
            m_contact_distance = 0;
            RVS_WARN("contact_distance {}  < 0", contact_distance);
        }
    };

    /**
     * @brief Get the Contact Distance Threshold object
     *
     * @return double
     */
    virtual double GetContactDistanceThreshold() const
    {
        return m_contact_distance;
    };

    /**
     * @brief Check Collision use collision matrix
     *
     * @param[in] collision_matrix
     * @param[out] report collision check result
     * @return true in collision
     * @return false collision free
     */
    virtual bool CheckCollision(const CollisionMatrix &collision_matrix,
                                CollisionReport &report) const = 0;

    /**
     * @brief
     *
     * @param link1
     * @param link2
     * @param report
     * @return true
     * @return false
     */
    virtual bool CheckCollision(const Link &link1, const Link &link2,
                                CollisionReport &report) const = 0;
    /**
     * @brief check continuous collison check between two links
     *
     * @param link1
     * @param pose1_beg
     * @param pose1_end
     * @param link2
     * @param pose2_beg
     * @param pose2_end
     * @param report
     * @return true
     * @return false
     */
    virtual bool
    ContinuousCheckCollision(const Link *link1, const SE3d &pose1_beg,
                             const SE3d &pose1_end, const Link *link2,
                             const SE3d &pose2_beg, const SE3d &pose2_end,
                             CollisionReport &report) const = 0;

    /**
     * @brief To update the observer's state when publishers publish its new
     * subject
     * @see abstract base class ObserverBase
     *
     */
    virtual void UpdateState(const OSDatagram &os_datagram) = 0;

protected:
    /**
     * @brief check manipulator continuous collison check
     *
     * @param manipulator
     * @param start_state
     * @param end_state
     * @param collision_matrix
     * @param report
     * @return true
     * @return false
     */
    virtual bool
    _ContinuousCheckCollision(std::shared_ptr<Manipulator> manipulator,
                              const JointVector &start_state,
                              const JointVector &end_state,
                              const CollisionMatrix &collision_matrix,
                              CollisionReport &report) const = 0;

protected:
    double m_contact_distance;
};

/// @}
} // namespace RVS