// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once

#include <RVS/Environment/RobotModel.h>
#include <RVS/Trajectory/PathBase.h>

namespace RVS
{
///@addgroup CollisionChecker
///@{

enum ObjectClusterType
{
    ObjectClusterType_Unknown = 0, ///< Undefined object cluster type
    ObjectClusterType_FixedStatic =
        1, ///< Totally fixed and static object cluster
    ObjectClusterType_FixedDynamic =
        2, ///< Fixed wrt world and shifted wrt inside cluster
    ObjectClusterType_MovedStatic =
        3, ///< Moved wrt world and static wrt inside cluster
    ObjectClusterType_MovedDynamic =
        4 ///< Moved wrt world and shifted wrt inside cluster
};

class CollisionObjectCluster
{
public:
    using OptiHierIndex = std::vector<std::vector<std::set<size_t>>>;
    using OptiHierReps = std::vector<std::vector<std::shared_ptr<Link>>>;

    struct HierarchyIndex
    {
        OptiHierIndex
            m_hier_index; ///< To store the indices of optimal hierarchies
        bool m_is_rel_idx; ///< The type of indices: absolute or relative
    };

    /**
     * @brief Construct a new collision object cluster
     *
     * @param cluster_name  The name of a collision cluster
     * @param cluster_type The type of cluster objects
     */
    CollisionObjectCluster(
        const std::string &cluster_name,
        ObjectClusterType cluster_type = ObjectClusterType_Unknown);


    /**
     * @brief Get the name of the object cluster
     *
     * @return std::string  The object cluster name
     */
    std::string GetName() const { return m_cluster_name; }

    /**
     * @brief Get the type of a collision cluster objects
     *
     * @return const ObjectClusterType
     */
    ObjectClusterType GetClusterType() const { return m_cluster_type; }

    /**
     * @brief Get all object links in the collision cluster
     *
     * @return const std::vector<std::shared_ptr<Link>>& links of object
     */
    const std::vector<std::shared_ptr<Link>> &GetObjectLinks() const;

    size_t GetNumberOfLinks() const;

    /**
     * @brief Set the optimal dynamic hierarchies indices with respect to the
     * some given criterion function
     *
     * @param opti_hier_arch The resulting indices from optimal dynamic
     * hierarchies
     */
    // void SetOptimalDynamicHierarchiesIndex(const
    // HierarchyIndex&opti_hier_arch);

    /**
     * @brief Get the number of level of optimal hierarchies of collision
     * cluster
     *
     * @return const size_t
     */
    size_t GetNumberOfLevelWithDH() const;


    /**
     * @brief Get specific hierarchical representations of objects (links) in
     * terms of the provided optimal hierarchies indices
     *
     * @param[out] objects_in_opti_hier The hierarchical representations of
     * objects
     *
     * @return true The result if it normally returns the hierarchy
     * representation of objects
     * @return false The result if some error occurs in the object cluster,
     * i.e., the object cluster hasn't been initialized successfully
     */
    bool GetHierarchiesReps(OptiHierReps &objects_in_opti_hier);

protected:
    // To obtain the links from input multibody(s)
    virtual void _GetObjectLinks() = 0;
    // To get valid links in set of links
    void _GetValidLinks(const std::vector<std::shared_ptr<Link>> &links);
    // Construct dynamic hierarchies in order to generate the optimal hierarchy
    // indices
    virtual void _ConstructDynamicHierarchies();
    virtual std::vector<size_t> _ComputeTourIndex();
    std::string m_cluster_name; ///< The name of collision cluster
    ObjectClusterType m_cluster_type; ///< the type of object cluster
    std::vector<std::shared_ptr<Link>>
        m_object_links; ///< The object links from robot model or obstacles
                        ///< (multibody)
    std::vector<size_t> m_cluster_tour; ///< the sequence of objects in order to
                                        ///< minimize the cost function
    HierarchyIndex m_opti_hier_idx; ///< The optimal hierarchical indices of the
    ///< collision cluster
private:
    // To calculate the "distance" between any two target poses and the distance
    // relies on the weight value of each component of pose
    double _CalcWeightedDist(const std::vector<SE3d> &target_poses,
                             const std::vector<size_t> &target_indices,
                             const CVecXd &weights);

    // to find nearest neighbour with 2-opt swap algorithm (please refer to
    // https://en.wikipedia.org/wiki/2-opt for more details)
    std::vector<size_t> _SwapWith2Opt(const std::vector<size_t> &target_indices,
                                      int r_i, int c_j);

    std::vector<std::shared_ptr<Multibody>>
        m_intermediate_multibodies; ///< holds the multibody of each link in
                                    ///< hierarchy representations
};

class FixedStaticObjectCluster : public CollisionObjectCluster
{
public:
    /**
     * @brief Construct a new fixed static object cluster. It usually consists
     * of totally static obstacles
     *
     * @param cluser_name The name of fixed and static object cluster
     *
     */
    FixedStaticObjectCluster(const std::string &cluser_name);

    /**
     * @brief Construct a new collision object cluster
     *
     * @param cluser_name  The name of a collision cluster
     * @param objects  The set of objects that there no exist collisions between
     * them
     */
    FixedStaticObjectCluster(const std::string &cluser_name,
                             std::vector<std::shared_ptr<Multibody>> objects);

    /**
     * @brief  Initialize the object cluster
     *
     * @param objects The set of multibody in a object cluster
     *
     * @return true The result if new objects have been successfully added into
     * cluster
     * @return false The result if it fails to do it
     */
    bool
    SetObjectCluster(const std::vector<std::shared_ptr<Multibody>> &objects);

    /**
     * @brief Herein, the object links are from ordinary multibody(s). The pose
     * with all links will be modified by given new poses
     *
     * @param object_poses The new poses of all object links
     */
    // void UpdateObjectsState(const std::vector<SE3d> &object_poses);

protected:
    virtual void _GetObjectLinks() override;


private:
    std::vector<std::shared_ptr<Multibody>>
        m_obstacles; ///< the set of objects in the cluster
};

class FixedDynamicObjectCluster : public CollisionObjectCluster
{
public:
    /**
     * @brief Construct a new fixed dynamic object cluster. The manipulator is
     * typical of fixed dynamic object cluster, i.e, its base can't move in the
     * wrold but articulated link chain derived from manipulator is likely to
     * move with respect to its base
     *
     * @param cluser_name The name of fixed and dynamic object cluster
     */
    FixedDynamicObjectCluster(const std::string &cluser_name);

    /**
     * @brief Construct a new collision object cluster object from given robot
     * model or obstacles
     *
     * @cluser_name Similarly above
     * @objects Similarly above
     *
     */
    FixedDynamicObjectCluster(const std::string &cluser_name,
                              std::shared_ptr<Multibody> objects);

    /**
     * @brief Construct a new fixed dynamic object cluster object
     *
     * @param cluser_name The name of the fixed dynamic object cluster
     * @param manip The smart pointer pointing to active manipulator
     *
     */
    FixedDynamicObjectCluster(const std::string &cluser_name,
                              std::shared_ptr<Manipulator> manip);

    /**
     * @brief  Initialize the object cluster
     *
     * @param objects Similarly above
     *
     * @return true The result if new objects have been successfully added into
     * cluster
     * @return false The result if it fails to do it
     *
     */
    bool SetObjectCluster(std::shared_ptr<Multibody> objects);

    /**
     * @brief Initialize the fixed dynamic object cluster
     *
     * @param objects The smart pointer pointing to active manipulator
     *
     * @return true Similarly above
     * @return false Similarly above
     *
     */
    bool SetObjectCluster(std::shared_ptr<Manipulator> objects);

    /**
     * @brief Assuming the object links from robot model, It will update
     * configuration of manipulator of robot model
     *
     * @param joints_config The joint angles of manipulator
     *
     * @return true The result if it succeeds to update obejct state in the
     * cluster
     * @return false The result if it fails to do it
     */
    bool UpdateObjectsState(const Rxd &joints_config);

protected:
    virtual void _GetObjectLinks() override;

private:
    std::shared_ptr<RobotModel>
        m_robot_model; ///< the robot model that holds some manipulator
};

/// TODO: implement class MovedStaticObjectCluster and class
/// MovedDynamicObjectCluster

class ConstructDynamicHierarchy
{
public:
    using OptiHierIndex = CollisionObjectCluster::OptiHierIndex;
    using HierarchyIndex = CollisionObjectCluster::HierarchyIndex;
    /**
     * @brief Default constructor
     *
     */
    ConstructDynamicHierarchy();

    /**
     * @brief Construct a new ConstructDynamicHierarchy
     *
     * @param objects  The object links from collision cluster
     */
    ConstructDynamicHierarchy(
        const std::vector<std::shared_ptr<Link>> &objects);

    ///@TODO To find the best optimal dymanic hierarchies from a series of
    /// configurations of manipulator of robot model
    ConstructDynamicHierarchy(std::shared_ptr<Manipulator> manipulator,
                              const PathBase<JointVector> &joint_path);

    /**
     * @brief Initialize the object links
     *
     * @param objects The links from collision cluster
     *
     * @return true The result that it has succeeded to initialize the object
     * links in the class
     * @return false The result that it fails to do it
     *
     */
    bool SetObjectLinks(const std::vector<std::shared_ptr<Link>> &objects);

    /**
     * @brief Initialize the object links from robot model
     *
     * @param manipulator The manipulator from some robot model
     * @return true The result if the pointer of manipulator is non-empty
     * @return false The result if it is empty
     *
     */
    bool SetObjectLinks(std::shared_ptr<Manipulator> manipulator);

    /**
     * @brief Set the joint trajectories of manipulator of robot model
     *
     * @param joint_path The reference path of all joints of manipulator
     * @return true The result if the path is valid
     * @return false The result if it is invalid
     *
     */
    bool SetObjectsPath(const PathBase<JointVector> &joint_path);

    /**
     * @brief Get the optimal hierarchical indices according to the criterion
     * function that minimizes the diameter of compound object from lower level
     * objects
     *
     * @return const HierarchyIndex& The indices corresponding to optimal
     * dynamic hierarchies
     */
    const HierarchyIndex &GetOptiHierIndices();

    /**
     * @brief Get the optimal hierarchies relative indices. Concretely, the
     * index of each level (greater than 1) is relative to that of
     * adjacent lower level. Other is similar to the func @GetOptiHierIndices
     *
     * @return const HierarchyIndex& The relative indices of optimal hierarchies
     */
    const HierarchyIndex &GetOptiHierRelativeIndices();

private:
    std::vector<std::shared_ptr<Link>>
        m_object_links; ///< The object links from collision cluster
    std::vector<std::shared_ptr<Box>>
        m_primitive_bbox; ///< the list of bounding boxes of primitive links
    HierarchyIndex m_opti_hier_indices; ///< the indices of optimal hierarchies
    HierarchyIndex m_opti_hier_relative_indices; ///< the relative indices of
                                                 ///< optimal hierarchies
    void _CalcOptimalHierarchy();
    void _CalcRelativeOptimalHierarchy();
};

///@}
} // namespace RVS