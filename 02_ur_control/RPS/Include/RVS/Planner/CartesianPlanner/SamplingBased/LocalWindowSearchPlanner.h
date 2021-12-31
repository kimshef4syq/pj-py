// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include <RVS/LieGroupHeader.h>
#include <RVS/Kinematics/KinematicsBase.h>
#include <RVS/Kinematics/CostCntTerms.h>
#include <RVS/CollisionCheckerHeader.h>
#include <RVS/Planner/Core/MotionPlannerBase.h>
#include <RVS/Common/Timer.h>
namespace RVS
{
/// @addtogroup Planner
/// @{


enum LSNInitType
{
    LSNInitType_Distance = 0, ///< nearest valid state from joint seed
    LSNInitType_Manipulability = 1, ///< best manipubility solutions
    LSNInitType_Collision = 2 ///< farthest from collision state
};

/**
 * @brief Local Sampling-based Nullspace Planner.
 * Various manifacturing tasks call for a planner that solves the redundancy
 * of (position normal) type tasks.
 * LSNPlanner (Local Sampling-based Nullspace Planner) handles this by sampling
 * the null space and finding the one with minimal cost within the constrain
 * set.
 */
class LSNPlanner : public MotionPlannerBase
{
public:
    explicit LSNPlanner(const std::string &name = "LSNPlanner");

    struct Configuration : public std::enable_shared_from_this<Configuration>
    {
    public:
        RVS_DECLARE_PTR_MEMBER(Configuration);

        Configuration(std::shared_ptr<Environment> env_in,
                      std::shared_ptr<Manipulator> manipulator_in);

        virtual ~Configuration() = default;

        /** @brief Generate LSN planning problem */
        bool Setup();

        /** @brief Environment. ***REQUIRED*** */
        std::shared_ptr<Environment> env = nullptr;

        /** @brief Manipulators used for path planning ***REQUIRED*** */
        std::shared_ptr<Manipulator> manipulator = nullptr;

        /** @brief A sampling set to search in the solution space. For
        PositionNormal task, each element is represented by a angle that rotates
        around the z-axis of the tool frame ***REQUIRED*** */
        std::vector<double> action_set;

        /** @brief joint space distance threshold */
        double dist_thre = std::numeric_limits<double>::infinity();

        /** @brief cost weight of joint distance and manipulability*/
        double dist_cost_coeff = 1.0;
        double singularity_cost_coeff = 0.01;

        /** @brief search region: define the search region where the planner try
         * to find the local optimal */
        int search_region = 1;


        /*********** Init State ******************/
        /** @brief init state type: define the metric to find a proper init
         * state */
        LSNInitType init_type = LSNInitType::LSNInitType_Distance;

        /** @brief init state type: define the joint seed to find a proper init
         * state default value is the current joint position of the manipulator
         */
        JointVector joint_seed;

        /*********** State Constraints ***********/
        /**
         * @brief If true, StateCollisionConstraint will be enabled.
         * Default: true
         */
        bool scn_collision_check = false;

        double scn_collision_distance_threshold = 0;

        double scn_collision_step_size = 0.05;

        /*********** Motion Constraints ***********/
    };

    bool SetConfiguration(const Configuration::Ptr &configuration);

    /**
     * @brief check whether the LSNPlanner has been initialized properly or
     * not
     * @return RVSReturn  It will return RVSReturn_Success if configured,
     * otherwise, RVSReturn_NotInitialized will be returned
     */
    virtual RVSReturn IsConfigured() const override;


    /** @brief Clear the data structures used by the planner */
    virtual void Clear() override;


    virtual bool Terminate() override
    {
        RVS_ERROR("Not Implemented");
        return false;
    }

    /**
     * @brief Solve the motion planning problem
     * @todo why return a boolean instead of RVSReturn
     * @param request The multiple goal poses at task space (it may be TSP
     * (Travelling Salesman Problem) or GTSP)
     * @param response The optimized joint configuration sequences (i.e, joint
     * motion trajectories) accorrding to given optimized criteria
     * @param verbose The related information when solving RTSP
     * @return true Succeed to solve the RTSP
     * @return false Fail to do it
     */
    virtual bool Solve(const MotionPlannerRequest &request,
                       MotionPlannerResponse &response,
                       const bool verbose = false) override;


    /**
     * @brief Find best initial joint position and angle (along the redundant
     * axis)
     * @param w0: the corresponding initial pose
     * @param joint_seed: joint seed for solving IK
     * @param action_set: A sampling set to search in the solution space.
     *                        For PositionNormal task, each element is
                              represented by a angle that rotates around the
                              z-axis of the tool frame.
     * @param[in] check_collision: whether to check collision when finding best
     init state. (indepenet from configuration)
     * @param[in] num_sample: number of samples to find a minimal distance cost
     *                          start position
     * @param[in] init_type: initialization type for init q, default is joint
     distance from joint seed
     * @return best joint position and angle
     *
     */
    std::tuple<JointVector, double>
    FindBestInitState(const SE3d &w0, const JointVector &joint_seed,
                      const bool check_collision = false,
                      const unsigned num_sample = 40,
                      const LSNInitType init_type = LSNInitType_Distance);

private:
    struct State
    {
        State(JointVector q_in = JointVector(), double theta_in = 0,
              int idx_in = -1)
            : q(q_in), theta(theta_in), idx(idx_in)
        {
        }
        JointVector q;
        double theta;
        int idx;
    };

    /**
     * @brief Initialize the planner by the given configuration
     * @note be called after configured
     * @return true if success otherwise false;
     */
    bool _Initialize();


    /**
     * @brief One step forward on the given waypoiny list.
     * @param[in] q: current q
     * @param[in] theta: current theta
     * @param[in] idx: current idx
     * @param[out] a pair : ( q[idx+1], theta[idx+] )
     */
    State StepForward(const State &state, bool is_augmented_searched = false);

    bool _EnableCollisionCheck();

    bool _ProcessRequest(const MotionPlannerRequest &request);

    double _EvalCost(const JointVector &q, const JointVector &last_q) const;

    bool _CheckCollision(const JointVector &q);

    bool _CheckMotionCollision(const JointVector &q0, const JointVector &q1,
                               double &valid_length);

    double _StepToCollision(const SE3d &start_pose,
                            const JointVector &joint_seed,
                            const double step_size);

    inline std::vector<double> _Linspace(double start, double end, int N);

private:
    Configuration::Ptr m_configuration;

    std::vector<CartesianViapointPtr> m_wlist;
    std::vector<State> m_state_list;
    std::shared_ptr<Manipulator> m_manip;
    std::shared_ptr<SingularityFunctor> m_singularity_functor;
    std::shared_ptr<KinematicsBase> m_kin_solver;
    std::shared_ptr<CollisionCheckerBase> m_collision_checker;
    State m_init_state;
    CVecXd m_dof_weights;
    bool m_init_state_found;
    bool m_collision_check;
};

/// @}
} // namespace RVS
