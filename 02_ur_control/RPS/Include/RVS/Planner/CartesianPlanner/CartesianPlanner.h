// Copyright (c) RVBUST, Inc - All rights reserved
#pragma once
#include <RVS/Planner/Core/MotionPlannerBase.h>
#include <RVS/Planner/Core/Viapoint.h>
#include <RVS/Planner/CartesianPlanner/SamplingBased/GlobalGraphSearchPlanner.h>
#include <RVS/Planner/CartesianPlanner/Types.h>
#include <RVS/Planner/CartesianPlanner/TrajectoryGeneration/OptimalSplineTrajGenerator.h>
#include <RVS/Planner/CartesianPlanner/Utils.h>

namespace RVS
{

///@addtogroup Planner
///@{

/** @brief CartesianPlanner is designed to solves cartesian planning problems
 * defined by `CartesianPlanRequest`, a jerk-optimal spline trajectory (and
 * joint positions) will be generated as response */
class CartesianPlanner
{
public:
    CartesianPlanner(std::shared_ptr<Manipulator> manipulator,
                     std::shared_ptr<Environment> env,
                     const std::string &name = "CartesianPlanner");


    struct Configuration
    {
    public:
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

        /** @brief bound joint vleocity */
        bool bound_joint_velocity = true;

        /** @brief bound_joint_acceleration */
        bool bound_joint_acceleration = false;

        /** @brief bound_joint_jerk */
        bool bound_joint_jerk = false;

        /** @brief the joint space distance value to tell whether a extra joint
         * position should be intepolated when linear refactoring is required */
        double linear_refactoring_joint_distance = 1e-3;

        /** @brief using jerk filtering post process */
        bool jerk_filtering = false;

        /** @brief order of the optimal polynomial */
        size_t polynomial_order = 5;

        /** @brief minimize jerk ( consider jerk optimal in polynomial
         * trajectory generation) */
        size_t minimize_order = 3;
    };

    /** @brief Get configuration */
    inline std::shared_ptr<Configuration> GetConfiguration() const noexcept
    {
        return m_configuration;
    }

    /** @brief is planner configured */
    inline bool IsConfigured() const { return m_is_configured; };

    /** @brief get a default configuration for the planner*/
    std::shared_ptr<Configuration> GetDefaultConfiguration() const noexcept;


    /** @brief get name */
    inline const std::string &GetName() const noexcept { return m_name; }

    /** @brief set configurations of GSPPlanner*/
    RVSReturn SetConfiguration(std::shared_ptr<Configuration> configuration);


    /**
     * @brief Solve the motion planning problem
     * @param request CartesianPlanRequest
     * @param response CarteisnPlanResponse
     * @param verbose The related information when solving
     * @return true for success otherwise false
     */
    bool Solve(const CartesianPlanRequest &request,
               CartesianPlanResponse &response, bool verbose = false);

protected:
    std::shared_ptr<Manipulator> m_manipulator;

    std::shared_ptr<Environment> m_env;

    std::shared_ptr<KinematicsBase> m_kin_solver;

    CVecXd m_q_dot_max;

    CVecXd m_q_ddot_max;

    CVecXd m_q_dddot_max;

    std::string m_name;

    std::shared_ptr<Configuration> m_configuration;

    std::shared_ptr<GSPPlanner> m_gsp_planner;

    bool m_is_configured;
};

///@}
} // namespace RVS
