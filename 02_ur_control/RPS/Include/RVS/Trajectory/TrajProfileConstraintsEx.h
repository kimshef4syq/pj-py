// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include <RVS/Trajectory/TrajProfileConstraints.h>

#include <RVS/Trajectory/PathBase.h>

namespace RVS
{
///@addtogroup Trajectory
///@{
RVS_CLASS_FORWARD(PiecewiseUniformProfileConstraints);
/**
 * @brief Piecewise uniform trajectory profile constraints, aka, the kinematic
 * constraints are same at every point.
 *
 */
class PiecewiseUniformProfileConstraints : public TrajProfileConstraintsBase
{
public:
    /**
     * @brief Construct a new Piecewise Uniform Profile Constraints
     * object, the constraints are uniform within each segment.
     *
     * @param s_arr vector of semgnet break point path paramaters, size of N
     * @param max_vels max velocity of each DoF, shape of (N-1, DoF)
     * @param max_accs max acceleration of each DoF, shape of (N-1, DoF)
     * @param max_jerks max jerk of each DoFa, shape of (N-1, DoF)
     */
    PiecewiseUniformProfileConstraints(
        const std::vector<double> &s_arr, const MatXd &max_vels_arr,
        const MatXd &max_accs_arr,
        const MatXd &max_jerks_arr = MatXd::Zero(0, 0));

    virtual CVecXd GetMaxVelocityConstraint(double s) const override;
    virtual CVecXd GetMaxAccelerationConstraint(double s) const override;
    virtual CVecXd GetMaxJerkConstraint(double s) const override;

private:
    std::vector<double> m_s_arr;
    MatXd m_max_vels_arr; ///< maximum velocities of each DoF of each segment
    MatXd m_max_accs_arr; ///< maximum accelerations of each DoF of each
                          ///< segment
    MatXd m_max_jerks_arr; ///< maximum jerks of each DoF of each segment
};


RVS_CLASS_FORWARD(PiecewiseUniformScaleProfileConstraints);
/**
 * @brief Piecewise uniform scale trajectory profile constraints, each piece of
 * segment scales the max kinematic constraints with a uniform value.
 *
 */
class PiecewiseUniformScaleProfileConstraints
    : public TrajProfileConstraintsBase
{
public:
    /**
     * @brief Construct a new Piecewise Uniform Profile Constraints object
     * @note Even though the contraints are not contiuous on second order
     * (velocity), trajectory will velocity contiuous.
     *
     * @param s_arr path paramater vector, N
     * @param scale_arr max velocity/acceleration/jerk scale ratios at each
     * piece, N-1
     * @param max_vels max velocity of each DoF
     * @param max_accs max accelaration of each DoF
     * @param max_jerks max jerk of each DoF
     */
    PiecewiseUniformScaleProfileConstraints(
        const std::vector<double> &s_arr, const std::vector<double> &scale_arr,
        const CVecXd &max_vels, const CVecXd &max_accs,
        const CVecXd &max_jerks = CVecXd::Zero(0));

    virtual CVecXd GetMaxVelocityConstraint(double s) const override;
    virtual CVecXd GetMaxAccelerationConstraint(double s) const override;
    virtual CVecXd GetMaxJerkConstraint(double s) const override;

private:
    std::vector<double> m_s_arr; ///< path parameters of segment break points
    std::vector<double> m_scale_arr; ///< scale values of each segment
    CVecXd m_max_vels; ///< maximum velocities of each DoF
    CVecXd m_max_accs; ///< maximum accelerations of each DoF
    CVecXd m_max_jerks; ///< maximum jerks of each DoF
};


RVS_CLASS_FORWARD(PiecewiseUniformCartesianProfileConstraints);
/**
 * @brief Piecewise uniform motion of cartesian tip, only for R3xSO3.
 *
 */
class PiecewiseUniformCartesianProfileConstraints
    : public TrajProfileConstraintsBase
{
public:
    /**
     * @brief Construct a kinematic limits about cartesian linear speed and
     * acceleration on TCP.
     * @param path Cartesian path on Pose space
     * @param s_arr path paramater vector, breakpoint of each segment, N
     * @param speed_arr cartesian tip linear speed of each segment
     * @param acc_arr cartesian tip linear acceleration of each segment, if
     * empty, a default value of 5 x speed_arr is used.
     */
    PiecewiseUniformCartesianProfileConstraints(
        PathBase<Pose>::ConstPtr path, const std::vector<double> &s_arr,
        const std::vector<double> &speed_arr,
        const std::vector<double> &acc_arr = {});

    virtual CVecXd GetMaxVelocityConstraint(double s) const override;
    virtual CVecXd GetMaxAccelerationConstraint(double s) const override;
    virtual CVecXd GetMaxJerkConstraint(double /*s*/) const override;

private:
    PathBase<Pose>::ConstPtr
        m_path; ///< geometric path that the trajectory follows
    std::vector<double> m_s_arr; ///< path parameters of segment break points
    std::vector<double> m_speed_arr; ///< TCP speed values of each segment
    std::vector<double> m_acc_arr; ///< TCP acceleration values of each segment
};


RVS_CLASS_FORWARD(PiecewiseRampScaleProfileConstraints);
/**
 * @brief Piecewise ramp scale trajectory profile constraints, each piece of
 * segment scales the max constraints with a linear interpolation value. It is
 * smoother than uniformly scale at the breaking points.
 *
 */
class PiecewiseRampScaleProfileConstraints : public TrajProfileConstraintsBase
{
public:
    /**
     * @brief Construct a new Piecewise Uniform Profile Constraints object
     *
     * @param s_arr path paramater vector, N
     * @param scale_arr max velocity/acceleration/jerk scale ratio at each
     * piece, N-1
     * @param max_vels max velocity of each DoF
     * @param max_accs max accelaration of each DoF
     * @param max_jerks max jerk of each DoF
     */
    PiecewiseRampScaleProfileConstraints(
        const std::vector<double> &s_arr, const std::vector<double> &scale_arr,
        const CVecXd &max_vels, const CVecXd &max_accs,
        const CVecXd &max_jerks = CVecXd::Zero(0));

    virtual CVecXd GetMaxVelocityConstraint(double s) const override;
    virtual CVecXd GetMaxAccelerationConstraint(double s) const override;
    virtual CVecXd GetMaxJerkConstraint(double s) const override;

private:
    std::vector<double> m_s_arr; ///< path parameters of segment break points
    std::vector<double> m_scale_arr; ///< scale values of each segment
    CVecXd m_max_vels; ///< maximum velocities of each DoF
    CVecXd m_max_accs; ///< maximum accelerations of each DoF
    CVecXd m_max_jerks; ///< maximum jerks of each DoF
};


RVS_CLASS_FORWARD(PiecewiseRampCatersianProfileConstraints);
/**
 * @brief Piecewise ramp motion of cartesian tip, only for R3xSO3.
 *
 */
class PiecewiseRampCatersianProfileConstraints
    : public TrajProfileConstraintsBase
{
public:
    /**
     * @brief Construct a kinematic ramp limits about cartesian linear speed
     * and acceleration on TCP.
     * @param path Cartesian path on Pose space
     * @param s_arr path paramater vector, breakpoint of each segment, N
     * @param speed_arr cartesian tip linear speed of each segment
     * @param acc_arr cartesian tip linear acceleration of each segment, if
     * empty, a default value of 5 x speed_arr is used.
     */
    PiecewiseRampCatersianProfileConstraints(
        PathBase<Pose>::ConstPtr path, const std::vector<double> &s_arr,
        const std::vector<double> &speed_arr,
        const std::vector<double> &acc_arr = {});

    virtual CVecXd GetMaxVelocityConstraint(double s) const override;
    virtual CVecXd GetMaxAccelerationConstraint(double s) const override;
    virtual CVecXd GetMaxJerkConstraint(double /*s*/) const override;

private:
    PathBase<Pose>::ConstPtr
        m_path; ///< geometric path that the trajectory follows
    std::vector<double> m_s_arr; ///< path parameters of segment break points
    std::vector<double> m_speed_arr; ///< TCP speed values of each segment
    std::vector<double> m_acc_arr; ///< TCP acceleration values of each segment
};
///@}
} // namespace RVS