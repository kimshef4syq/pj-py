// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once

#include <unordered_set>
#include <unordered_map>

#include <RVS/Common/Time.h>
#include <RVS/CollisionChecker/CollisionCheckerBase.h>
#include "FCLUtils.h"


namespace RVS
{
///@addtogroup CollisionChecker
///@{

RVS_CLASS_FORWARD(FCLCollisionChecker);

/// @brief CollisionChecker use FCL lib
class FCLCollisionChecker : public CollisionCheckerBase
{
public:
    ///@todo: test bv_types and algo_types
    FCLCollisionChecker();

    /**
     * @brief Destroy the FCLCollisionChecker object
     *
     */
    virtual ~FCLCollisionChecker();

    static std::string GetName() { return "FCLCollisionChecker"; }

    static CollisionCheckerBasePtr Create()
    {
        return std::make_shared<FCLCollisionChecker>();
    }

    CollisionCheckerBasePtr Clone() const override;

    virtual bool InitFromEnv(std::shared_ptr<Environment> env) override;

    virtual void Clear() override;

    virtual bool AddCollisionObject(const std::string &name,
                                    std::shared_ptr<Geometry> shape,
                                    const SE3d &shape_pose,
                                    bool enabled = true) override;

    virtual bool AddCollisionObject(const Link &link,
                                    bool enabled = true) override;
    /**
     * @brief To add collision object into FCL manager
     * @see class CollisionCheckerBase
     *
     */
    virtual bool AddCollisionObject(std::shared_ptr<Multibody const> mul,
                                    bool enabled = true,
                                    bool add_into_env = true) override;

    void AddCollisionObject(const CollisionObjectWrapperPtr &cow);

    virtual bool RemoveCollisionObject(const std::string &name) override;

    virtual bool RemoveCollisionObject(const Link &link) override;

    /**
     * @brief To remove collision object from FCL manager
     * @see class CollisionCheckerBase
     *
     */
    virtual bool RemoveCollisionObject(std::shared_ptr<Multibody const> mul,
                                       bool remove_from_env = true) override;

    virtual bool HasCollisionObject(const std::string &name) const override;

    virtual bool HasCollisionObject(const Link &link) const override
    {
        return HasCollisionObject(link.GetUniqueName());
    }

    virtual bool EnableCollisionObject(const std::string &name) override;

    virtual bool DisableCollisionObject(const std::string &name) override;

    virtual void SetCollisionObjectsTransform(const std::string &name,
                                              const SE3d &pose) override;

    void SetIsContactAllowedFn(IsContactAllowedFn fn) { m_fn = fn; }

    virtual bool CheckCollision(const CollisionMatrix &collision_matrix,
                                CollisionReport &report) const override;

    virtual bool CheckCollision(const Link &link1, const Link &link2,
                                CollisionReport &report) const override;
    virtual bool
    ContinuousCheckCollision(const Link *link1, const SE3d &pose1_beg,
                             const SE3d &pose1_end, const Link *link2,
                             const SE3d &pose2_beg, const SE3d &pose2_end,
                             CollisionReport &report) const override;

    /**
     * @brief Get the total time of checking collision
     *
     * @return double  check time (unit: second)
     */
    double GetCheckTime() const;

    /**
     * @brief Get the time for synchronizing state of multibodies into FCL
     * objects
     *
     * @return double synchronization time (unit: second)
     */
    double GetSyncTime() const;

    /**
     * @brief To update the observer's state when publishers publish its new
     * subject
     * @see abstract base class ObserverBase
     *
     */
    virtual void UpdateState( const OSDatagram& os_datagram) override;

protected:
    virtual bool
    _ContinuousCheckCollision(std::shared_ptr<Manipulator> manipulator,
                              const JointVector &start_state,
                              const JointVector &end_state,
                              const CollisionMatrix &collision_matrix,
                              CollisionReport &report) const override;


private:
    void _AllocCollisionBroadPhase() const;
    mutable std::shared_ptr<fcl::BroadPhaseCollisionManager<double>> m_manager;
    Link2COW m_link2cow;
    IsContactAllowedFn m_fn;
    mutable TimeDuration m_check_time; ///< time for checking collison
    mutable TimeDuration m_sync_time; ///< time for synchronizing objects state
    std::list<std::shared_ptr<Multibody>>
        m_publishers; ///< publisher list from which the checker subscribes the
                      ///< subject (i.e., the base transformation of multibody)
    std::shared_ptr<Environment>
        m_env; ///< the environment that holds all collision objects
};
/// @}

} // namespace RVS