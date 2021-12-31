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

RVS_CLASS_FORWARD(BodySurfaceGraphConstructor);

/**
 * @brief This class is specifically used to generate joint points near
 * Multibody surface
 *
 */
class BodySurfaceGraphConstructor : public GraphConstructorBase
{
public:
    explicit BodySurfaceGraphConstructor(
        const std::string &name = "BodySurfaceGraphConstructor")
        : GraphConstructorBase(name)
    {
        RVS_TRACE("Constructing BodySurfaceGraphConstructor");
    }

    virtual ~BodySurfaceGraphConstructor()
    {
        RVS_TRACE("Destroying BodySurfaceGraphConstructor");
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
            RVS_TRACE("Destroying BodySurfaceGraphConstructor::Configuration");
        }

        virtual bool Setup() override;

    public:
        std::shared_ptr<Multibody> obstacle; ///< target object
        double bbox_extend_dist; ///< The extended distance of the bounding box
        double discrete_step; ///< Discrete step size of bounding box
        double grow_out_step; ///< Grow outside step of each inside points
        double outward_extent_dist; ///< Extend out distance
        std::vector<int>
            outer_normal_direction; ///< support [1,0,0], [-1,0,0], [0,1,0],
                                    ///< [0,-1,0], [0,0,1], [0,0,-1]
        size_t target_valid_joint_num; ///< target sampler number
        JointVector seed; ///< joint seed for better ik
        SO3Tangentd
            orientation_range; ///< Orientation range during randomization
        R3Tangentd position_range; ///< Position range during randomization
    };

public:
    virtual bool SetConfiguration(const Configuration::Ptr &configuration);

    virtual RVSReturn IsConfigured() const override;

    virtual bool ConstructGraph() override;

    // Get discrete points of bounding box
    std::vector<R3d> GetOOBBPoints() const;

    // Get discrete points in obstacle
    std::vector<R3d> GetGeoInsidePoints() const;

    // Get discrete points near the surface of obstacle
    std::vector<R3d> GetGeoOutsidePoints() const;

    // Get poses near the surface of obstacle
    std::vector<SE3d> GetGeoOutsidePoses() const;

    // Get valid poses near the surface of obstacle
    std::vector<SE3d> GetValidOutsidePoses() const;

    // Get valid joints near the surface of obstacle
    std::vector<JointVector> GetValidOutsideJoints() const;

protected:
    Configuration::Ptr _CastConfiguration() const
    {
        return std::dynamic_pointer_cast<Configuration>(GetConfiguration());
    }

private:
    void _GenerateOOBBAndGeometryPoints(const std::shared_ptr<Multibody> &body,
                                        const double bbox_extend_dist,
                                        const double discrete_step,
                                        std::vector<R3d> &oobb_points,
                                        std::vector<R3d> &inside_points);

    void _GenerateGeometryOutsidePoints(const std::shared_ptr<Multibody> &body,
                                        const std::vector<R3d> &inside_points,
                                        const double grow_out_step,
                                        const double outward_extent_dist,
                                        std::vector<R3d> &outside_points);

    void _GenerateGeometryOutsidePoses(
        const std::vector<R3d> &inside_points,
        const std::vector<R3d> &outside_points,
        const std::vector<int> &outer_normal_direction,
        std::vector<SE3d> &outside_poses);

    bool _GenerateValidOutsidePoses(const std::vector<SE3d> &outside_poses,
                                    const size_t target_num,
                                    const JointVector &seed,
                                    const R3Tangentd &position_range,
                                    const SO3Tangentd &orientation_range,
                                    std::vector<SE3d> &valid_poses,
                                    std::vector<JointVector> &valid_joints);

private:
    std::shared_ptr<Multibody> m_obstacle;
    std::shared_ptr<KinematicsBase> m_kin_solver;
    SE3d m_world_to_robot;

    std::vector<R3d> m_oobb_points;
    std::vector<R3d> m_inside_points;
    std::vector<R3d> m_outside_points;
    std::vector<SE3d> m_outside_poses;
    std::vector<SE3d> m_valid_outside_poses;
    std::vector<JointVector> m_valid_outside_joints;
};
} // namespace RVS