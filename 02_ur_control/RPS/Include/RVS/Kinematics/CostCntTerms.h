// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include <RVS/OptModel/ComponentBase.h>
#include <RVS/OptModel/GeneralComponent.h>
#include <RVS/EnvironmentHeader.h>

namespace RVS
{

enum PenaltyType
{
    PenaltyType_Squared = 0,
    PenaltyType_Abs = 1 // Not Implemented
};

RVS_CLASS_FORWARD(Functor);

/**
 * @brief Functor is to reduce the complexity of construct a CostTerm or
 *                    ConstrainSet directly. For a term (i.e. a function), such
 *                    as "cartesian pose error", we should be able to define it
 *                    as a cost or a constraint easily. Moreover, the related
 *                    variables of this cost or constraint should have nothing
 *                    to do with the function (e.g. a cartesian pose error can
 *                    be imposed in 1-st waypoint or 50-th waypoint).
 */
class Functor : public std::enable_shared_from_this<Functor>
{
public:
    virtual CVecXd GetValue(const CVecXd &var_values) const = 0;

    virtual MatXd GetJacobian(const CVecXd & /*var_values*/) const
    {
        RVS_THROW("Not Implemented in the base class");
    }

    virtual ~Functor() {}

    CVecXd Call(const CVecXd &var_values, const bool is_new = true)
    {
        if (is_new) {
            CVecXd val = GetValue(var_values);
            m_last_val.resize(val.rows(), 1);
            m_last_val = val;
        }
        return m_last_val;
    }

private:
    CVecXd m_last_val;
};


/**
 * @brief Cartesian Pose Error Functor,
 *        Calculate cartesian pose error of a link with respect to target pose
 */
class CartPoseErrFunctor : public Functor
{

public:
    /**
     * @brief Construct a CartPoseErrFunctor
     * @param[in] manip: manipulator for handling kinematics of the robot
     * @param[in] link_name: index of link to compute
     * @param[in] target_pose: the target pose
     * @param[in] end_to_tool: end to tool transformation
     * @param[in] world_to_base: world_to_base transformation
     */
    CartPoseErrFunctor(std::shared_ptr<Manipulator> manip,
                       const std::string &link_name, const SE3d &target_pose,
                       const SE3d &end_to_tool = SE3d::IdentityStatic(),
                       const SE3d &world_to_base = SE3d::IdentityStatic())
        : m_manip(manip), m_link_name(link_name), m_target_pose(target_pose),
          m_end_to_tool(end_to_tool), m_world_to_base(world_to_base)

    {
        m_slist = m_manip->GetSlist();
    }

    /**
     * @brief GetValue will be called each time the corresponding cost (or
     * constrain)'s value need to be evaluated.
     * @param[in] dof_values: input dof values to find current cartesian pose
     */
    virtual CVecXd GetValue(const CVecXd &dof_values) const override;

    /**
     * @brief GetJacobian will be called each time the corresponding cost (or
     * constrain)'s gradient need to be evaluated.
     * @param[in] dof_values: input dof values to find current cartesian pose
     */
    virtual MatXd GetJacobian(const CVecXd &dof_values) const override;

    const std::string &GetLinkName() const { return m_link_name; }

    SE3d GetTargetPose() const { return m_target_pose; }

    SE3d GetTCP() const { return m_end_to_tool; }

    std::shared_ptr<Manipulator> GetManipulator() const { return m_manip; }

private:
    std::shared_ptr<Manipulator> m_manip;
    std::vector<SE3Tangentd> m_slist;
    std::string m_link_name;
    SE3d m_target_pose;
    SE3d m_end_to_tool;
    SE3d m_world_to_base;
};


/**
 * @brief Joint Position Error Functor
 *       Calculate joint position error with respect to a target position.
 */
class JointPosErrFunctor : public Functor
{
public:
    /**
     * @brief Construct a JointPosErrFunctor
     * @param[in] manip: manipulator for handling kinematics of the robot
     * @param[in] target_pos: the target joint position
     */
    JointPosErrFunctor(std::shared_ptr<Manipulator> manip,
                       const CVecXd &target_pos)
        : m_manip(manip), m_target_pos(target_pos)
    {
    }

    /**
     * @brief GetValue will be called each time the corresponding cost (or
     * constrain)'s value need to be evaluated.
     * @param[in] dof_values: input dof values
     */
    virtual CVecXd GetValue(const CVecXd &dof_values) const override;

    /**
     * @brief GetJacobian will be called each time the corresponding cost (or
     * constrain)'s gradient need to be evaluated.
     * @param[in] dof_values: input dof values
     */
    virtual MatXd GetJacobian(const CVecXd &dof_values) const override;

private:
    std::shared_ptr<Manipulator> m_manip;
    CVecXd m_target_pos;
};

/**
 * @brief  Joint difference Functor
 *         calculates difference betwen two joint positions using numerical
 *         difference method.
 */
class JointDiffFunctor : public Functor
{
public:
    /**
     * @brief compute the joint position difference
     * @param[in] manip: manipulator for handling kinematics of the robot
     * @param[in] limit: if err \in [-tolerance, tolerance], then err will not
     * be optimized
     */
    JointDiffFunctor(std::shared_ptr<Manipulator> manip,
                     const CVecXd &tolerance = CVecXd::Ones(0))
        : m_manip(manip), m_tolerance(tolerance)
    {
        RVS_ENSURE(
            m_tolerance.size() == manip->GetDoF() || m_tolerance.size() == 0,
            "Input of tolerance should be of (num_dof) or 0 dimensions . ");
    }
    /**
     * @brief GetValue will be called each time the corresponding cost (or
     * constrain)'s value need to be evaluated.
     * @param[in] var_values = [ dof_values_1   dof_values_2 ] has a size of (2
     * * num_dof)
     *
     */
    virtual CVecXd GetValue(const CVecXd &var_values) const override;
    /**
     * @brief GetJacobian will be called each time the corresponding cost (or
     * constrain)'s gredient need to be evaluated.
     * @param[in] var_values = [ dof_values_1   dof_values_2 ] has a size of (2
     * * num_dof)
     *
     */
    virtual MatXd GetJacobian(const CVecXd &var_values) const override;

private:
    std::shared_ptr<Manipulator> m_manip;
    CVecXd m_tolerance;
};


/**
 * @brief Cartesian Difference Functor
 *        calculates position difference in cartesian space using numerical
 *        difference method, (only position, orientation difference is not be
 *        considered)
 */
class CartDiffFunctor : public Functor
{
public:
    /**
     * @brief CartDiffFunctor calculates position velocity in cartesian space
     * using numerical difference method
     * @param[in] manip: manipulator for handling kinematics of the robot
     * @param[in] link_name: name of the link
     * @param[in] limit: if error \in [-limit, limit], then error will not be
     * optimized
     */
    CartDiffFunctor(std::shared_ptr<Manipulator> manip,
                    const std::string &link_name, double limit,
                    const SE3d &end_to_tool = SE3d(),
                    const SE3d &world_to_base = SE3d())
        : m_manip(manip), m_link_name(link_name), m_limit(limit),
          m_end_to_tool(end_to_tool), m_world_to_base(world_to_base)
    {
    }

    /**
     * @brief GetValue will be called each time the corresponding cost (or
     * constrain)'s value need to be evaluated.
     * @param[in] dof_values:
     *               dof_values = [ dof_values_1   dof_values_2 ] has a size of
     * (2 * num_dof)
     *
     *           where dof_values1 and dof_values2 are dof_values in step i and
     * step i+1 respectively.
     */
    virtual CVecXd GetValue(const CVecXd &var_values) const override;


private:
    std::shared_ptr<Manipulator> m_manip;
    std::string m_link_name;
    double m_limit;
    SE3d m_end_to_tool;
    SE3d m_world_to_base;
};

/**
 * @brief Singularity Functor
 *      calculate singularity value using jacobian matrix related to the
 *      end-effector. (we only care about the manipulability of the
 *      end-effector)
 */
class SingularityFunctor : public Functor
{
public:
    /**
     * @brief calculate singularity value using jacobian matrix related to the
     * end-effector. (we only care about the manipulability of the end-effector)
     * @param[in] kin_handler:
     * @param[in] ee_idx: the index of the end-effector frame.
     */
    SingularityFunctor(std::shared_ptr<Manipulator> manip,
                       const std::string &ee_link_name)
        : m_manip(manip), m_ee_link_name(ee_link_name)
    {
        m_slist = m_manip->GetSlist();
    }

    /**
     * @brief operator will be called each time the corresponding cost (or
     * constrain)'s value need to be evaluated.
     * @brief[in] dof_values: the dof values of the robot
     */
    virtual CVecXd GetValue(const CVecXd &dof_values) const override;

private:
    std::shared_ptr<Manipulator> m_manip;
    std::vector<SE3Tangentd> m_slist;
    std::string m_ee_link_name;
};

RVS_CLASS_FORWARD(CostFromFunctor);

/**
 * @brief construct a CostTerm from a Functor
 */
class CostFromFunctor : public CostTerm
{

public:
    using Super = CostTerm;

    /**
     * @brief construct a CostTerm from a Functor
     * @param[in] func: the functor to use.
     * @param[in] use_numerical_jacobi: whether use numerical jacobian or not.
     * @param[in] var_idx: the corresponding index vector related to the
     *                     functor.
     * @param[in] coeffs: the coefficiences vector specifying how dimensions are
     *                    combined to calculate cost value (a scalar)
     * @param[in] type: penalty type to calculate cost value
     * @param[in] name: name of the cost term
     */
    CostFromFunctor(FunctorPtr func, const bool use_numerical_jacobi,
                    const IdxVec &var_idx, const CVecXd &coeffs,
                    const PenaltyType type, const std::string &name)

        : Super(name, nullptr, var_idx), m_func(func),
          m_use_numerical_jacobi(use_numerical_jacobi), m_var_idx(var_idx),
          m_coeffs(coeffs), m_type(type)

    {
    }

    void SetUseNumJacobi(bool use_numerical_jacobi)
    {
        m_use_numerical_jacobi = use_numerical_jacobi;
    }

    bool GetUseNumJacobi() const { return m_use_numerical_jacobi; }

    virtual double GetValue(const double *x, unsigned n,
                            const bool new_x = true) const override;

    virtual bool GetJacobian(const double *x, unsigned n, double *jac,
                             bool accumulation = false) const override;

private:
    FunctorPtr m_func;
    bool m_use_numerical_jacobi;
    IdxVec m_var_idx;
    CVecXd m_coeffs;
    PenaltyType m_type;
};


} // namespace RVS
