// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include <RVS/LieGroupHeader.h>
#include <RVS/Kinematics/KinematicsBase.h>
#include <RVS/Kinematics/CostCntTerms.h>
#include <RVS/CollisionCheckerHeader.h>
namespace RVS
{
/// @addtogroup Planner
/// @{

enum InitStateType
{
    InitStateType_Distance = 0, ///< nearest valid state from joint seed
    InitStateType_Manipubility = 1, ///< best manipubility solutions
    InitStateType_Collision = 2 ///< farthest from collision state
};

/**
 * @brief Sampling-based Task Redundancy Planner.
 * various manifacturing tasks call for a planner that resolves the redundancy
 * of (position normal) type tasks.
 * (Sampling-based Task Redundancy Planner) STRplanner handles this by sampling
 * the solution space and finding the one with minimal cost within the constrain
 * set.
 */
class STRPlanner
{
public:
    /**
     * @brief Construct a STRPlanner
     * @param[in] env: Environment
     * @param[in] manipulator: Manipulator
     */
    STRPlanner(std::shared_ptr<Environment> env,
               std::shared_ptr<Manipulator> manipulator);


    /**
     * @brief Initialize the planner by finding a good joint position at start
     *                      point of the path.
     * @param[in] wlist: list of waypoints.
     * @param[in] joint_seed: joint seed for solving IK
     * @param[in] action_set: A sampling set to search in the solution space.
     *                        For PositionNormal task, each element is
                              represented by a angle that rotates around the
                              z-axis of the tool frame.
     * @param[in] dist_threshold: distance threshold for nearest ik. It defines
                              the joint distance constraint (that is, a solution
                              with distance values greater than dist_threshold
                              will be seen as an infeasible solution).
     * @param[in] check_collision: It defines the collision
                              constraint (that is, a solution in collision will
                              be seen as an infeasible solution).)
     * @param[in] num_sample: number of samples to find a minimal distance cost
     *                          start position
     * @param[in] init_type: initialization type for init q, default is joint
     distance from joint seed
     * @param[out] success or failed
     */
    RVSReturn
    Initialize(const std::vector<SE3d> &wlist, const JointVector &joint_seed,
               const std::vector<double> &action_set,
               const double dist_threshold = 1.0,
               const bool check_collision = false,
               const unsigned num_sample = 40,
               const InitStateType init_type = InitStateType_Distance);

    void
    _FindBestInitState(const SE3d &w0, const JointVector &joint_seed,
                       const bool check_collision = false,
                       const unsigned num_sample = 40,
                       const InitStateType init_type = InitStateType_Distance);

    /**
     * @brief set the joint(dof) weight for calculating the joint distance
     * @param[in] dof_weights: dof weights
     * @param[out] success or failed
     */
    bool ResetDoFWeights(const std::vector<double> &dof_weights)
    {
        for (unsigned i = 0; i < dof_weights.size(); ++i) {
            if (dof_weights[i] >= 0) {
                m_dof_weights(i) = dof_weights[i];
            }
        }
        return true;
    }

    /**
     * @brief get the joint(dof) weight for calculating the joint distance
     * @param[out] dof_weights: dof weights
     */
    const CVecXd &GetDoFWeights() const { return m_dof_weights; }


    /**
     * @brief set the action set for samping the solution space
     * @param[in] action_set: action set
     * @param[out] success or failed
     */
    bool ResetActionSet(std::vector<double> &action_set)
    {
        m_action_set = action_set;
        return true;
    }

    /**
     * @brief get the action set for samping the solution space
     * @param[out] action_set
     */
    const std::vector<double> &GetActionSet() const { return m_action_set; }


    /**
     * @brief set the action set for samping the solution space
     * @param[in] action_set: action set
     * @param[out] success or failed
     */
    bool ResetDistThreshold(const double dist_thre)
    {
        m_dist_threshold = dist_thre;
        return true;
    }

    /**
     * @brief get the distance threshold.
     * @param[out] action_set
     */
    double GetDistThreshold() const { return m_dist_threshold; }


    /**
     * @brief get the list of solutions in joint space
     * @param[out] solution set
     */
    const std::vector<JointVector> &Getqlist() const { return m_qlist; }


    /**
     * @brief enable collision checking
     * @param[out] success or failed
     */
    RVSReturn EnableCheckCollision();

    bool SetCollisionStepSize(const double step_size);

    /**
     * @brief disable collision checking
     * @param[out] success or failed
     */
    RVSReturn DisableCheckCollision()
    {
        m_check_collision = false;
        return RVSReturn_Success;
    }

    /**
     * @brief whether the planner enabled collision checking
     * @param[out] enabled or disabled
     */
    bool GetCheckCollision() const { return m_check_collision; }

    /**
     * @brief set the list of waypoints
     * @param[out] success or failed
     */
    bool ResetWaypoints(const std::vector<SE3d> &wlist)
    {
        m_wlist = wlist;
        return true;
    }

    /**
     * @brief get the list of waypoints
     * @param[out] list of waypoints
     */
    const std::vector<SE3d> &GetWaypoints() const { return m_wlist; }


    /**
     * @brief set the coefficients of cost terms
     * @param[in] dist_coeff: coefficients of distance cost term
     * @param[in] singularity_coeff: coefficients of singularity cost term
     * @param[out] success or failed
     */
    RVSReturn ResetCostCoeffs(double dist_coeff, double singularity_coeff)
    {
        m_dist_coeff = dist_coeff;
        m_singularity_coeff = singularity_coeff;
        return RVSReturn_Success;
    }

    /**
     * @brief get the coefficients of cost terms
     * @param[out] coeffs
     */
    std::tuple<double, double> GetCostCoeffs() const
    {
        return std::make_tuple(m_dist_coeff, m_singularity_coeff);
    }


    /**
     * @brief get the start joint position which is usually intialized by
     * calling 'Initialize(...)'
     * @param[out] coeffs
     */
    JointVector GetInitPos() const { return m_init_q; }


    /**
     * @brief Perform planning. Note that, planner should be first initialized
     *                          before plannnig.
     * @param[out] success or failed
     */
    RVSReturn Plan();

    /**
     * @brief One step forward on the given waypoiny list.
     * @param[in] q: current q
     * @param[in] theta: current theta
     * @param[in] idx: current idx
     * @param[out] a pair : ( q[idx+1], theta[idx+] )
     */
    std::pair<JointVector, double> StepForward(JointVector q, double theta,
                                               unsigned idx);


private:
    /**
     * @brief One step forward on the given waypoiny list.
     *              This method improved original one by augmented sampling
     *              at the state where any vaild ik cannot be found.
     * @param[in] q: current q
     * @param[in] theta: current theta
     * @param[in] idx: current idx
     * @param[out] a pair : ( q[idx+1], theta[idx+])
     */
    std::pair<JointVector, double> _StepForwardAug(JointVector q, double theta,
                                                   unsigned idx,
                                                   bool is_augmented = false);

    double _EvalCost(const JointVector &q, const JointVector &last_q);

    bool _CheckCollision(const JointVector &q);

    bool _CheckMotionCollision(const JointVector &q0, const JointVector &q1,
                               double &valid_length);

    double _StepToCollision(const SE3d &start_pose,
                            const JointVector &joint_seed,
                            const double step_size);

    inline std::vector<double> _Linspace(double start, double end, int N);

private:
    std::shared_ptr<Environment> m_env;
    std::shared_ptr<Manipulator> m_manip;
    std::shared_ptr<SingularityFunctor> m_singularity_functor;
    std::shared_ptr<KinematicsBase> m_kin_solver;
    CollisionCheckerBasePtr m_collision_checker;
    std::vector<double> m_action_set;
    std::vector<SE3d> m_wlist;
    std::vector<JointVector> m_qlist;
    double m_dist_threshold;
    double m_dist_coeff;
    double m_singularity_coeff;
    double m_init_theta;
    double m_step_size; ///< motion collision checking step size
    bool m_check_collision;
    bool m_initialized;
    JointVector m_init_q;
    CVecXd m_dof_weights;
};

/// @}
} // namespace RVS