// Copyright (c) RVBUST, Inc - All rights reserved
#pragma once
#include <iostream>
#include <RVS/Planner/Core/Graph/Graph.h>
#include <RVS/Planner/Core/MotionPlannerBase.h>
#include <RVS/CollisionCheckerHeader.h>
#include <RVS/Kinematics/Utils.h>

namespace RVS
{
///@addtogroup Planner
///@{
using namespace RVS::graph;

RVS_CLASS_FORWARD(GSPPlanner);

/**
 * @brief GSPPlanner: Global Graph Search Planner enables searching a shortest
 * (defined by the metric function) joint path under the constraint defined by
 * viapoints (which contains pose, time stamp and redundancy informations)
 *
 */
class GSPPlanner : public std::enable_shared_from_this<GSPPlanner>
{
public:
    /** @brief the external properties held by vertex */
    struct LadderVertex
    {
        size_t viapoint_idx;

        TimeConstraint time_cnt;

        Rxd joint_position;
    };

    using LadderGraph = graph::WeightedGraph<LadderVertex>;

    /** @brief the metric function measuring the cost between two joint
     * solutions*/
    using MetricFunction =
        typename std::function<double(const Rxd &, const Rxd &)>;

    explicit GSPPlanner(const std::shared_ptr<Manipulator> manipulator,
                        const std::shared_ptr<Environment> env,
                        bool copy_env = true,
                        const std::string &name = "GSPPlanner");

    ~GSPPlanner() = default;

    inline const std::string &GetName() const noexcept { return m_name; }

    struct Configuration
    {
    public:
        RVS_DECLARE_PTR_MEMBER(Configuration);

        /** @brief weights for defining metric*/
        std::vector<double> dof_weights;

        /** @brief metric space distance threshold */
        double metric_distance_threshold =
            std::numeric_limits<double>::infinity();

        /** @brief joint limits lower bound in planner */
        std::vector<double> joint_limits_lower_bound;

        /** @brief joint limits upper bound in planner */
        std::vector<double> joint_limits_upper_bound;

        /** @brief check collsion */
        bool check_collision = false;

        /** @brief planner will be terminated when max_planning_time exceed */
        double max_planning_time = 1.0;

        /** @brief bound joint vleocity by approximating delta time from time
         * constraint of viapoints*/
        bool bound_joint_velocity = true;

        /** @brief planner will save the graph as .dot file if `graphviz_name`
         * is not empty*/
        std::string graphviz_name = "";
    };

    /** @brief Get a default configuration for the planner*/
    std::shared_ptr<Configuration> GetDefaultConfiguration() const;

    /** @brief set configurations of GSPPlanner*/
    RVSReturn SetConfiguration(std::shared_ptr<Configuration> configuration);

    /**
     * @brief Get the initialized configuration parameters
     * @return const std::shared_ptr<Configuration>
     */
    inline const std::shared_ptr<Configuration> &GetConfiguration() const
    {
        return m_configuration;
    }

    /**
     * @brief check whether the GSPPlanner has been initialized properly or
     * not
     * @return RVSReturn It will return RVSReturn_Success if configured,
     * otherwise, RVSReturn_NotInitialized will be returned
     */
    RVSReturn IsConfigured() const;


    /** @brief Clear the data structures used by the planner */
    void Clear();


    /** @brief terminate the planning process */
    bool Terminate();

    /** @brief build ladder graph by parsing viapoint infomation */
    bool BuildLadderGraph(const std::vector<ViapointPtr> &viapoints,
                          TerminateConditionPtr term_cond =
                              std::shared_ptr<NonTerminateCondition>());

    /** @brief build ladder graph by parsing viapoint infomation */
    bool
    BuildLadderGraphWithTempStart(const std::vector<ViapointPtr> &viapoints,
                                  TerminateConditionPtr term_cond =
                                      std::shared_ptr<NonTerminateCondition>());

    /**
     * @brief Solve the motion planning problem
     * @param[in] request: input viapoints (waypoints with redundancy and time
     * @param[out] response: a vector of (resolutional) optimal joint
     * positions
     * @param verbose: verbose print
     * @return true for success otherwise false
     */
    bool Solve(const MotionPlannerRequest &viapoints,
               MotionPlannerResponse &joint_position_list,
               const bool verbose = false);

    /**
     * @brief Solve the motion planning problem
     * @param[in] viapoints: input viapoints (waypoints with redundancy and time
     * constraint information)
     * @param[out] joint_position_list: a vector of (resolutional) optimal joint
     * positions
     * @param verbose: verbose print
     * @return true for success otherwise false
     */
    bool Solve(const std::vector<ViapointPtr> &viapoints,
               std::vector<Rxd> &joint_position_list,
               const bool verbose = false);


private:
    /** @brief wrapper function for evaluating edge constraints */
    bool EvalEdgeConstraints(const LadderVertex &v1, const LadderVertex &v2);

    /** @brief wrapper function for evaluating vertex constraints */
    bool EvalVertexConstraints(const LadderVertex &v);


    // planner members
    std::shared_ptr<Manipulator> m_manipulator;

    std::shared_ptr<Environment> m_env;

    std::string m_name;

    std::shared_ptr<KinematicsBase> m_kin_solver;

    std::vector<double> m_joint_vel_max;

    LadderGraph m_ladder_graph;

    std::shared_ptr<Configuration> m_configuration;

    std::vector<VertexDescriptor> m_start_set;

    std::vector<VertexDescriptor> m_goal_set;

    MetricFunction m_metric_function;

    bool m_is_initialized;

    std::shared_ptr<TriggeredTerminateCondition> m_trigger_cond;

    CompoundMotionConstraintPtr m_motion_constraints;

    CompoundStateConstraintPtr m_state_constraints;

    size_t m_dof;
};
///@}
}; // namespace RVS