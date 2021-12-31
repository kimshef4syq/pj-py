// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once

#include <iostream>
#include <fstream>
#include <time.h>
#include <random>

#include <RVS/Planner/Core/Types.h>
#include <RVS/Planner/Core/MotionPlannerBase.h>
#include <RVS/Planner/GraphConstructor/BodySurfaceGraphConstructor.h>
#include <RVS/Planner/GraphConstructor/RobotSectorRegionGraphConstructor.h>
#include <RVS/Planner/Ompl/OmplP2PRoadmapPlanner.h>

namespace RVS
{
/// @addtogroup Planner
/// @{

RVS_CLASS_FORWARD(P2PPlanner);


class P2PPlanner : public MotionPlannerBase
{
public:
    explicit P2PPlanner(const std::string &name = "P2PPlanner")
        : MotionPlannerBase(name)
    {
        RVS_TRACE("Constructing P2PPlanner");
        m_configuration = nullptr;
        m_planner = nullptr;
    }

    virtual ~P2PPlanner() { RVS_TRACE("Destroying P2PPlanner"); }

    struct Configuration : public ConfigurationBase
    {
    public:
        RVS_DECLARE_PTR_MEMBER(Configuration);

        Configuration(
            std::shared_ptr<Environment> env_in,
            const std::vector<std::shared_ptr<Manipulator>> &manipulators_in);

        virtual ~Configuration()
        {
            RVS_TRACE("Destroying P2PPlanner::Configuration");
        }

        virtual bool Setup() override;

    public:
        std::vector<int>
            outer_normal_direction; ///< support [1,0,0], [-1,0,0], [0,1,0],
                                    ///< [0,-1,0], [0,0,1], [0,0,-1]
        size_t obstacle_sample_num;

        double min_theta;
        double max_theta;
        double min_z;
        double max_z;
        double min_r;
        double max_r;
        size_t sector_sample_num;

        bool rebuild_graph;
        std::string roadmap_file;
    };

public:
    virtual bool SetConfiguration(const Configuration::Ptr &configuration);

    virtual RVSReturn IsConfigured() const;

    const Configuration::Ptr &GetConfiguration() const
    {
        return m_configuration;
    }

    void SaveRoadMap(const std::string &filename = "") const;

    static MotionPlannerRequest CreateRequest(const JointVector &start,
                                              const JointVector &goal);

    bool Solve(const MotionPlannerRequest &request,
               MotionPlannerResponse &response, const bool verbose = false);

    virtual bool Terminate() override { return true; };

    virtual void Clear() override { return; };

private:
    Configuration::Ptr m_configuration;
    std::shared_ptr<OmplP2PRoadmapPlanner> m_planner;
    SE3d m_world_to_robot;
};
} // namespace RVS