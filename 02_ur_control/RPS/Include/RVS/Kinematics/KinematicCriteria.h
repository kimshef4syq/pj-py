// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include <RVS/Common/Utils.h>
#include <RVS/Common/Types.h>
#include <RVS/Environment/Manipulator.h>
#include <RVS/Kinematics/KinematicsBase.h>

namespace RVS
{

/// @addtogroup Kinematics
/// @{

/**
 * @brief Kinematics criterias, such as manipulability ..
 *
 * @note Pamanes, G. J. A., and Said Zeghloul. "Optimal placement of robotic
 * manipulators using multiple kinematic criteria." Proceedings. 1991 IEEE
 * International Conference on Robotics and Automation. IEEE, 1991.
 *
 */
class KinematicCriteria
{
public:
    /**
     * @brief Construct a new KinematicCriteria object from Manipulator and
     * KinematicsBase
     *
     * @param[in] manipulator initialized Manipulator
     * @param[in] kin_solver kinematics solver, default is nullptr, and this
     * class will construct default kinsolver from Manipulator
     */
    KinematicCriteria(std::shared_ptr<Manipulator> manipulator,
                      std::shared_ptr<KinematicsBase> kin_solver = nullptr);

    ~KinematicCriteria();

    /**
     * @brief This measurement computes the Yoshikawa's manipulability index by
     * computing the Singular value Decomposition (SVD) of the Jacobian.
     *
     *  Two modes are offered:
     *          - all singular values are multiplied
     *          - the ratio of minimum and maximum singular value is returned
     * (also known as (inverted) Condition number).
     *
     * @param[in] joint_values: manipulator active joint values
     * @param[in] type: manipubility type, default is 0 (all singular values are
     * multiplied).
     * @return double
     */
    double
    Manipulability(const JointVector &joint_values,
                   const ManipubilityType type = ManipubilityType_MultiplyAll);

    /**
     * @brief Extended manipulability
     *
     * @param[in] joint_values manipulator active joint values
     * @param[in] type manipubility type, default is 0 (all singular values are
     * multiplied).
     * @return double
     */
    double ExtendedManipulability(
        const JointVector &joint_values,
        const ManipubilityType type = ManipubilityType_MultiplyAll);

    /**
     * @brief Joint limit cost function, gives higher weight to the joints
     * nearing their limits and goes to infinity at the joint bounds
     *
     * @note Chan, Tan Fung, and Rajiv V. Dubey. "A weighted least-norm solution
     * based scheme for avoiding joint limits for redundant joint manipulators."
     * IEEE Transactions on Robotics and Automation 11.2 (1995): 286-292.
     * - JointLimitsCost: H(q) =
     * \sum_{i=1}^{n}{\frac{1}{4}}{\frac{(q_{i,max}-q_{i,min})^2}{(q_{i,max}-q)(q-q_{i,min})}}
     *
     * @todo: I'm not sure if we should add joint weights here
     *
     * @param[in] joint_values joint values
     * @return double cost values
     */
    double JointLimitsCost(const JointVector &joint_values);

    /**
     * @brief Gradient of JointLimitsCost.
     *  - JointLimitsGradient: d{H(q_i)} = \frac{\partial{H(q)}}{\partial{q_i}}
     *
     * @param[in] joint_values joint values
     * @return JointVector gradient in each dof
     */
    JointVector JointLimitsGradient(const JointVector &joint_values);

    /**
     * @brief Joint limit penalilzation terms on manipulability
     *
     * @note Vahrenkamp, Nikolaus, et al. "Manipulability analysis." 2012 12th
     * ieee-ras international conference on humanoid robots (humanoids 2012).
     * IEEE, 2012.
     *
     * @param[in] joint_values joint values
     * @param[out] penalization_low penalization due to near joint lower limit
     * @param[out] penalization_high penalization due to near joint upper limit
     * @return true
     * @return false
     */
    bool JointLimitsPenalization(const JointVector &joint_values,
                                 JointVector &penalization_low,
                                 JointVector &penalization_high);

    // double CollisionCost(const JointVector &joint_values);
    // JointVector CollisionGradient(const JointVector &joint_values);
    // bool CollisionPenalization(const JointVector &joint_values, JointVector
    // &penalization_low, JointVector &penalization_high);

    /**
     * @brief Augmented Jacobian
     *
     *      ajac = joint_limit_penalization_matrix * jac
     *
     *      // ajac = joint_limit_penalization_matrix *
     * collision_penalization_matrix * jac
     *
     * @param[in] joint_values
     * @param[in] direction_vec
     * @return MatXd
     */
    MatXd AugmentedJacobian(const JointVector &joint_values,
                            const std::vector<float> &direction_vec);

    // double ConditionNumber(const JointVector &joint_values);
    // double VelocityMagnitude(const JointVector &joint_values);
    // double VelocityAccuracy(const JointVector &joint_values);
    // double ForceAccuracy(const JointVector &joint_values);
    // double ForceAccuracy(const JointVector &joint_values);

    /// Get robot model
    inline std::shared_ptr<RobotModel> GetRobotModel() const
    {
        return m_robot_model;
    }

    /// Get manipulator
    inline std::shared_ptr<Manipulator> GetManipulator() const
    {
        return GetRobotModel()->GetManipulator(m_manipulator_name);
    }

    /// Get kinematics solver
    inline std::shared_ptr<KinematicsBase> GetKinSolver() const
    {
        return m_kin_solver;
    }

private:
    int m_dof; ///< dof of Manipulator
    double m_penalize_rotation_factor; ///< To align translational and
                                       ///< rotational components
    std::vector<JointLimit> m_joint_limits; ///< dof limits
    std::vector<std::vector<float>>
        m_cart_dim_permutations; ///< permutations for Cartesian dimension
                                 ///< movements

    std::string m_manipulator_name; ///< Manipulator name
    std::shared_ptr<RobotModel> m_robot_model; ///< RobotModel
    std::shared_ptr<KinematicsBase> m_kin_solver; ///< kinsolver

private:
    /// Compute singular values of a given jacobian matrix
    CVecXd _GetSingularValues(const MatXd &jacobian);
    /// Generate 2^6 Quadrants of Cartesian movement
    void _CreateCartDimPermutations(
        std::vector<std::vector<float>> &cart_dim_permutations);
    /// parse information from robot_model, manipulator and kin_sovler
    void _ParseInformation();
};

/// @}
} // namespace RVS
