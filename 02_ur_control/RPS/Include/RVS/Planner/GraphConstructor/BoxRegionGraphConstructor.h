// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once

#include <iostream>
#include <fstream>
#include <time.h>
#include <random>

#include <RVS/Environment/Geometry.h>
#include <RVS/Planner/Core/GraphConstructorBase.h>
#include <RVS/Kinematics/Types.h>

namespace RVS
{
/// @addtogroup Planner
/// @{

RVS_CLASS_FORWARD(BoxRegionGraphConstructor);

/**
 * @brief This class is specifically used to generate joint points inside and
 * above the container
 *
 * @todo Use Multibody container to replace container info
 * @todo Use boost graph library to manage data structure
 */
class BoxRegionGraphConstructor : public GraphConstructorBase
{
public:
    explicit BoxRegionGraphConstructor(
        const std::string &name = "BoxRegionGraphConstructor")
        : GraphConstructorBase(name)
    {
        RVS_TRACE("Constructing BoxRegionGraphConstructor");
    }

    virtual ~BoxRegionGraphConstructor()
    {
        RVS_TRACE("Destroying BoxRegionGraphConstructor");
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
        Box region; ///< Sample region
        double x_step; ///< The resolution of the discretized area in the X
                       ///< direction
        double y_step; ///< The resolution of the discretized area in the Y
                       ///< direction
        double z_step; ///< The resolution of the discretized area in the Z
                       ///< direction
        JointVector joint_seed; ///< Joint seed used for ik
        double ik_threshold; ///< Ik threshold
        size_t num_points; ///< The number of sampling points inside
                           ///< the container
        double max_dist; ///< The maximum distance between
                         ///< the random sampling point
                         ///< inside the container and the
                         ///< standard sampling point
        double rot_max_angle;
        double rot_angle1;
        double rot_angle2;
    };

public:
    virtual bool SetConfiguration(const Configuration::Ptr &configuration);

    virtual RVSReturn IsConfigured() const override;

    virtual bool ConstructGraph() override;

    const std::vector<SE3d> &GetDiscretizedPoses() const;
    const std::vector<SE3d> &GetValidRandomizedPoses() const;

protected:
    Configuration::Ptr _CastConfiguration() const
    {
        return std::dynamic_pointer_cast<Configuration>(GetConfiguration());
    }

private:
    /**
     * @brief Generate standard discrete sampling poses in region
     *
     */
    void _GeneratePosesFromRegionInfo(std::vector<SE3d> &poses);

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
    SE3d m_world_to_robot;

    std::vector<SE3d> m_discretized_poses;
    std::vector<SE3d> m_valid_randomized_poses;
    std::vector<JointVector> m_valid_randomized_joints;
};
} // namespace RVS