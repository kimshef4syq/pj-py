// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once

#include <RVS/EnvironmentHeader.h>
#include <RVS/Planner/Core/StateCost.h>
#include <RVS/OptSolver/OSQPAdapter.h>


namespace RVS
{

class NullspaceIKSolver : public std::enable_shared_from_this<NullspaceIKSolver>
{
public:
    /** @brief construct NullspaceIKSolver
     *  NullspaceIKSolver finds position and velocity IK for redundant robot
     * (ex. 6-axis with external axes) and optimize the costs given by exploit
     * the nullspace of jacobian matrix
     *  @param env: environment
     *  @param manipulator: manipulator
     *
     */
    explicit NullspaceIKSolver(std::shared_ptr<Environment> env,
                               std::shared_ptr<Manipulator> manipulator,
                               const std::string &name = "NullspaceIKSolver");

    ~NullspaceIKSolver() = default;


    struct Configuration
    {
    public:
        Configuration() = default;

        bool Setup() { return true; };

        /* dof weights of the robot is always an very important metric to be
         * defined*/
        std::vector<double> dof_weights;


        /* when finding velocity IK, `allow_task_scaling` scales the desired
         * velocity and return the maximum scaling factor when it is cannot be
         * reached*/
        bool allow_task_scaling = false;

        /*
         when finding velocity IK, whether position limits should be considered
         or not
        */
        bool position_limits = true;


        /*
        when finding velocity IK, whether velocity limits should be considered
        or not
        */
        bool velocity_limits = true;


        /*
        when finding velocity IK, whether acceleration limits should be
        considered or not
        */
        bool acceleration_limits = false;

        /*
        `dt` is used in shape joint position and acceleration limits in
        approximated manner, should be set to the controller sampling rate
        */
        double dt = 0.004;

        double max_acceleration_coeff = 1.;
    };

    const std::string &GetName() const { return m_name; }


    /**
     * @brief config and initialize the solver from a user-defined configuration
     */
    RVSReturn Config(std::shared_ptr<Configuration> configuration);

    /**
     * @brief whether the solver is completly config and initialized
     */
    inline RVSReturn IsConfigured() const
    {
        return m_is_configured ? RVSReturn_Success : RVSReturn_Failed;
    }


    /**
     * @brief Add a state cost and the corresponding weight
     */
    RVSReturn AddStateCost(std::shared_ptr<StateCost> state_cost,
                           double weight = 1.0);


    /**
     * @brief clear state cost
     */
    void ClearStateCost() { m_state_cost->Reset(); }


    const std::shared_ptr<CompoundStateCost> &GetStateCosts() const
    {
        return m_state_cost;
    }


    /** @brief Get velocity IK
     *  @param[in] x_dot_in: cartesian velocity (or, twist exactly) expressed in
     * body frame
     *  @param[in] q_dot_in: current joint position
     * body frame
     *  @param[out] q_dot_out: output joint velocity
     * body frame
     *
     *  @param manipulator: manipulator
     */
    RVSReturn GetVelocityIK(const SE3Tangentd &x_dot_in, const Rxd &q_in,
                            RxTangentd &q_dot_out, double &task_scaling)
    {
        return GetVelocityIK(x_dot_in, q_in, q_dot_out, task_scaling,
                             m_configuration->position_limits,
                             m_configuration->velocity_limits,
                             m_configuration->acceleration_limits,
                             m_configuration->allow_task_scaling);
    }

    RVSReturn GetVelocityIK(const SE3Tangentd &x_dot_in, const Rxd &q_in,
                            RxTangentd &q_dot_out, double &task_scaling,
                            bool position_limits, bool velocity_limits,
                            bool acceleration_limits, bool allow_task_scaling);

    /** @brief Get Position IK by calling velocity IK repeatly
     *  @param[in] pose: desired pose
     *  @param[in] q_seed: joint seed
     *  @param[out] ik_report: IK report
     *  @param[in] max_iterations: max iterations to find IK
     *
     */

    RVSReturn GetPositionIK(const SE3d &pose, const Rxd &q_seed,
                            IKReport &ik_report, const int max_iterations);


protected:
    bool _IsInTolerance(const SE3d &pose1, const SE3d &pose2, double tolerance);

    bool _GetDoFWeightsMat(const std::vector<double> &dof_weights,
                           MatXd &dof_weights_mat);


    bool _ShapeJointLimit(const Rxd &q, bool position_limits,
                          bool velocity_limits, bool acceleration_limits);

    std::shared_ptr<Environment> m_env;

    std::shared_ptr<Manipulator> m_manipulator;
    std::string m_name;
    std::shared_ptr<CompoundStateCost> m_state_cost;
    int m_dof;
    CVecXd m_q_dot_max, m_q_dot_min;
    std::shared_ptr<KinematicsBase> m_kin_solver;
    std::vector<SE3Tangentd> m_slist;
    MatXd m_dof_weights_mat;
    bool m_is_configured;
    std::shared_ptr<Configuration> m_configuration;
    RxTangentd m_q_dot_last;
    QP::QPSettings m_qp_settings;
};

} // namespace RVS