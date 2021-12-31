// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once

#include <iostream>
#include <fstream>
#include <time.h>
#include <random>

#include <RVS/Planner/Core/GraphConstructorBase.h>
#include <RVS/Kinematics/Types.h>

namespace RVS
{
/// @addtogroup Planner
/// @{

RVS_CLASS_FORWARD(RobotSectorRegionGraphConstructor);

/**
 * @brief This class is specifically used to generate joint points in Fan-shaped
 * area around the robot
 *
 */
class RobotSectorRegionGraphConstructor : public GraphConstructorBase
{
public:
    explicit RobotSectorRegionGraphConstructor(
        const std::string &name = "RobotSectorRegionGraphConstructor")
        : GraphConstructorBase(name)
    {
        RVS_TRACE("Constructing RobotSectorRegionGraphConstructor");
    }

    virtual ~RobotSectorRegionGraphConstructor()
    {
        RVS_TRACE("Destroying RobotSectorRegionGraphConstructor");
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
            RVS_TRACE(
                "Destroying RobotSectorRegionGraphConstructor::Configuration");
        }

        virtual bool Setup() override;

    public:
        double max_r;
        double min_r;
        double max_z;
        double min_z;
        double max_theta;
        double min_theta;
        double r_step;
        double z_step;
        double theta_step;

        size_t target_valid_joint_num;
        JointVector seed;
        SO3Tangentd
            orientation_range; ///< Orientation range during randomization
        R3Tangentd position_range; ///< Position range during randomization
    };

public:
    virtual bool SetConfiguration(const Configuration::Ptr &configuration);

    virtual RVSReturn IsConfigured() const override;

    virtual bool ConstructGraph() override;

    std::vector<SE3d> GetDiscretePoses();

    std::vector<SE3d> GetValidRandomPoses();

protected:
    Configuration::Ptr _CastConfiguration() const
    {
        return std::dynamic_pointer_cast<Configuration>(GetConfiguration());
    }

private:
    void _GenerateDiscreteSectorPoses(
        const double max_r, const double min_r, const double r_step,
        const double max_z, const double min_z, const double z_step,
        const double max_theta, const double min_theta, const double theta_step,
        const JointVector &seed, std::vector<SE3d> &discrete_poses);

    bool _GenerateRandomValidPoses(const std::vector<SE3d> &discrete_poses,
                                   const size_t num, const JointVector &seed,
                                   const SO3Tangentd &orientation_range,
                                   const R3Tangentd &position_range,
                                   std::vector<SE3d> &valid_poses,
                                   std::vector<JointVector> &valid_joints);

private:
    std::shared_ptr<KinematicsBase> m_kin_solver;
    SE3d m_world_to_robot;

    std::vector<SE3d> m_discrete_poses;
    std::vector<SE3d> m_valid_poses;
    std::vector<JointVector> m_valid_joints;
};
} // namespace RVS