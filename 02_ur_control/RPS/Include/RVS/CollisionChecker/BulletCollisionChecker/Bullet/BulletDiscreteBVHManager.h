// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include <RVS/CollisionChecker/BulletCollisionChecker/Bullet/BulletUtils.h>

namespace RVS
{

/** @brief A BVH implementaiton of a bullet manager */
class BulletDiscreteBVHManager
{
public:
    using Ptr = std::shared_ptr<BulletDiscreteBVHManager>;
    using ConstPtr = std::shared_ptr<const BulletDiscreteBVHManager>;

    BulletDiscreteBVHManager();
    ~BulletDiscreteBVHManager();
    BulletDiscreteBVHManager(const BulletDiscreteBVHManager &) = delete;
    BulletDiscreteBVHManager &
    operator=(const BulletDiscreteBVHManager &) = delete;
    BulletDiscreteBVHManager(BulletDiscreteBVHManager &&) = delete;
    BulletDiscreteBVHManager &operator=(BulletDiscreteBVHManager &&) = delete;

    static std::string name() { return "BulletDiscreteBVHManager"; }

    bool addCollisionObject(const std::string &name, const int &mask_id,
                            std::shared_ptr<Geometry> shape,
                            const SE3d &shape_pose, bool enabled = true);

    bool hasCollisionObject(const std::string &name) const;

    bool removeCollisionObject(const std::string &name);

    bool enableCollisionObject(const std::string &name);

    bool disableCollisionObject(const std::string &name);

    void setCollisionObjectsTransform(const std::string &name,
                                      const Eigen::Isometry3d &pose);

    void setCollisionObjectsTransform(const std::vector<std::string> &names,
                                      const std::vector<SE3d> &poses);

    const std::vector<std::string> &getCollisionObjects() const;

    void setActiveCollisionObjects(const std::vector<std::string> &names);

    const std::vector<std::string> &getActiveCollisionObjects() const;

    void setCollisionMarginData(CollisionMarginData collision_margin_data,
                                CollisionMarginOverrideType override_type =
                                    CollisionMarginOverrideType::REPLACE);

    void setDefaultCollisionMarginData(double default_collision_margin);

    void setPairCollisionMarginData(const std::string &name1,
                                    const std::string &name2,
                                    double collision_margin);

    const CollisionMarginData &getCollisionMarginData() const;

    void setIsContactAllowedFn(IsContactAllowedFn fn);

    IsContactAllowedFn getIsContactAllowedFn() const;

    void contactTest(ContactResultMap &collisions,
                     const ContactRequest &request);

    void addCollisionObject(const COW::Ptr &cow);


private:
    std::vector<std::string>
        active_; /**< @brief A list of the active collision objects */
    std::vector<std::string>
        collision_objects_; /**< @brief A list of the collision objects */

    std::unique_ptr<btCollisionDispatcher>
        dispatcher_; /**< @brief The bullet collision dispatcher used for
                        getting object to object collison algorithm */
    btDispatcherInfo dispatch_info_; /**< @brief The bullet collision dispatcher
                                        configuration information */
    BulletCollisionConfiguration
        coll_config_; /**< @brief The bullet collision configuration */
    std::unique_ptr<btBroadphaseInterface>
        broadphase_; /**< @brief The bullet broadphase interface */
    Link2Cow link2cow_; /**< @brief A map of all (static and active) collision
                           objects being managed */

    /**
     * @brief This is used when contactTest is called. It is also added as a
     * user point to the collsion objects so it can be used to exit collision
     * checking for compound shapes.
     */
    ContactTestData contact_test_data_;

    /** @brief Filter collision objects before broadphase check */
    TesseractOverlapFilterCallback broadphase_overlap_cb_;

    /** @brief This function will update internal data when margin data has
     * changed */
    void onCollisionMarginDataChanged();
};

} // namespace RVS
