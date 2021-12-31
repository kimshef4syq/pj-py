// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once

#include <iostream>
#include <fstream>
#include <time.h>
#include <random>

#include <RVS/Planner/Core/GraphConstructorBase.h>
#include <RVS/Planner/Ompl/OmplParallelPlanMotionPlanner.h>

namespace RVS
{
/// @addtogroup Planner
/// @{

RVS_CLASS_FORWARD(PathGraphConstructor);

/**
 * @brief This class is specifically used to generate graph between two points
 *
 * @todo Use boost graph library to manage data structure
 * @todo
 *       1. Input env, manipulator and two regions;
 *       2. SetP2PPlanner (RRTConnect);
 *       3. Auto/Random sample a state in each region (si, gi);
 *       4. Plan path between (si, gi), return a valid path;
 *       5. Sample state on path, add to graph;
 *       6. GoTo 3, until num_points > xxxx;
 */
class PathGraphConstructor : public GraphConstructorBase
{
public:
    explicit PathGraphConstructor(
        const std::string &name = "PathGraphConstructor")
        : GraphConstructorBase(name)
    {
        RVS_TRACE("Constructing PathGraphConstructor");
    }

    virtual ~PathGraphConstructor()
    {
        RVS_TRACE("Destroying PathGraphConstructor");
    }

    struct Configuration : public GraphConstructorBase::Configuration
    {
    public:
        RVS_DECLARE_PTR_MEMBER(Configuration);

        Configuration(
            std::shared_ptr<Environment> env_in,
            const std::vector<std::shared_ptr<Manipulator>> &manipulators_in);

        virtual ~Configuration()
        {
            RVS_TRACE("Destroying BoxRegionGraphConstructor::Configuration");
        }

        virtual bool Setup() override;

    public:
        SE3d pose_a; ///< The first point
        SE3d pose_b; ///< The second point
        size_t path_num; ///< The number of standard path points
                         ///< between two points
        size_t path_random_num; ///< The number of random sampling
                                ///< points near the standard path
        double path_random_pose_max_dist; ///< The maximum distance limit of
                                          ///< the random point of the path
                                          ///< from the standard path
        JointVector path_seed; ///< A good joint point near the
                               ///< path used for IK
        double path_ik_joint_seed_thresholds; ///< Threshold for IK
        double rot_max_angle;
        double rot_angle1;
        double rot_angle2;

        bool enable_planner;
        bool enable_planner_smooth;
    };

public:
    virtual bool SetConfiguration(const Configuration::Ptr &configuration);

    virtual RVSReturn IsConfigured() const override;

    virtual bool ConstructGraph() override;

    const std::vector<SE3d> &GetDiscretizedPoses() const;
    const std::vector<SE3d> &GetRandomizedPoses() const;
    const std::vector<SE3d> &GetValidRandomizedPoses() const;

protected:
    Configuration::Ptr _CastConfiguration() const
    {
        return std::dynamic_pointer_cast<Configuration>(GetConfiguration());
    }

private:
    void _GeneratePosesFromPathInfo(std::vector<SE3d> &poses);

    /**
     * @brief According to the standard discrete sampling points, generate
     * random valid sampling points
     *
     * @param origin_poses standard discrete sampling poses
     * @param num target valid number
     * @param max_dist Randomized distance
     * @param seed good joint seed for ik
     * @param seed_threshold joint seed threshold for ik
     * @param valid_poses
     * @param joint_states
     */
    void _GenerateValidRandomSamplers(
        const std::vector<SE3d> &origin_poses, const size_t num,
        const double max_dist, const double rot_max_angle,
        const double rot_angle1, const double rot_angle2,
        const JointVector &seed, const double seed_threshold,
        std::vector<SE3d> &valid_poses, std::vector<JointVector> &joint_states);

private:
    std::shared_ptr<KinematicsBase> m_kin_solver;
    std::shared_ptr<OmplParallelPlanMotionPlanner> m_planner;
    SE3d m_world_to_robot;

    std::vector<SE3d> m_discretized_poses;
    std::vector<SE3d> m_valid_randomized_poses;
    std::vector<JointVector> m_valid_randomized_joints;
};
} // namespace RVS
