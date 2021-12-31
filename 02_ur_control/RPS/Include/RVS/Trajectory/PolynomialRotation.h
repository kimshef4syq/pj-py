// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include "Polynomial.h"
#include <RVS/LieGroup/SO3.h>

namespace RVS
{
///@addtogroup Trajectory
///@{
namespace __RVS_HIDE
{
///@addtogroup Trajectory
///@{
template <typename PointType>
class CubicSpline;

/**
 * @brief PolynomialSO3 is an polynomial interpolation for Rotations.
 * References
 * http://www.ladispe.polito.it/corsi/meccatronica/02JHCOR/2010-11/Slides/Shuster_Pub_1993h_J_Repsurv_scan.pdf
 * https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.RotationSpline.html?highlight=rotationspline#scipy.spatial.transform.RotationSpline
 *
 * @tparam
 */
template <>
class Polynomial<SO3d>
{
    using PolynomialRn = Polynomial<Rxd>;

public:
    RVS_DECLARE_PTR_MEMBER(Polynomial<SO3d>);

    using PointType = SO3d;

    friend class CubicSpline<SO3d>;


    /**
     * @brief Like cubic interpolation for Rn space,  create a cubic
     * interpolation for SO3 space.
     *
     * @param r0 rotation at start
     * @param r1 rotation at end
     * @param rd0 angular rate at start, default zero
     * @param rd1 angular rate at end, default zero
     * @param T duration
     * @return Polynomial<SO3d>::Ptr
     */
    static Polynomial<SO3d>::Ptr
    Cubic(const SO3d &r0, const SO3d &r1,
          const SO3Tangentd &rd0 = SO3Tangentd::ZeroStatic(),
          const SO3Tangentd &rd1 = SO3Tangentd::ZeroStatic(), double T = 1)
    {
        CVec3d rotvec = (r1 - r0).Coeffs();
        CVec3d rotvec_dot = _A(rotvec) * rd1.Coeffs();
        MatXd coeffs(4, 3);
        coeffs.row(0) = CVecXd::Zero(3);
        coeffs.row(1) = rd0.Coeffs();
        coeffs.row(2) =
            (3 * rotvec - 2 * T * rd0.Coeffs() - T * rotvec_dot) / (T * T);
        coeffs.row(3) =
            (-2 * rotvec + T * rd0.Coeffs() + T * rotvec_dot) / (T * T * T);
        return std::make_shared<Polynomial<SO3d>>(coeffs, r0);
    }

    /**
     * @brief Like linear interpolation for Rn space,  create a linear
     * interpolation for SO3 space.
     *
     * @param r0 rotation at start
     * @param r1 rotation at end
     * @param T duration
     * @return Polynomial<SO3d>::Ptr
     */
    static Polynomial<SO3d>::Ptr Linear(const SO3d &r0, const SO3d &r1,
                                        double T = 1)
    {
        CVec3d rotvec = (r1 - r0).Coeffs();
        MatXd coeffs(2, 3);
        coeffs.row(0) = CVecXd::Zero(3);
        coeffs.row(1) = rotvec / T;
        return std::make_shared<Polynomial<SO3d>>(coeffs, r0);
    }

    /**
     * @brief Construct a new Polynomial object. This construction may be not so
     * intuitive as the coeffs are not for rotations, but for rotation vector.
     * Rotation vectors are in Rn space, but rotations not.
     *
     * @param coeffs the polynomial coefficients of rotation vector
     * @param r0 rotation at start
     */
    Polynomial(const MatXd &coeffs, const SO3d &r0 = SO3d::IdentityStatic())
        : m_rotvec(std::make_shared<PolynomialRn>(coeffs)), m_r0(r0)
    {
        RVS_ENSURE(coeffs.cols() > 0, "Paramater coeffs's size is zero");
        RVS_ENSURE(coeffs.rows() > 0, "Paramater coeffs's size is zero");
    }

    /**
     * @brief Compute the Rotation value at t of given derivative.
     *
     * @param t path paramater
     * @param deriv_order derivative order, 0<= deriv_order <= 2
     * @return CVecXd
     * - if deriv_order is 0, return value is the coeffs of SO3,
     * whose size is 4;
     * - if deriv_order is higher, return value is coeffs of SO3Tangent, whose
     * size is 3.
     */
    inline CVecXd operator()(double t, unsigned deriv_order = 0) const
    {
        if (deriv_order == 0) {
            return (m_r0 + SO3Tangentd((*m_rotvec)(t))).Coeffs();
        }
        else if (deriv_order == 1) {
            return _InvA((*m_rotvec)(t)) * (*m_rotvec)(t, 1);
        }
        else if (deriv_order == 2) {
            CVec3d rotvec = (*m_rotvec)(t, 0);
            CVec3d rotvec_deriv1st = (*m_rotvec)(t, 1);
            CVec3d rotvec_deriv2nd = (*m_rotvec)(t, 2);
            CVec3d delta_beta = _DeltaBeta(rotvec, rotvec_deriv1st);
            return _InvA(rotvec) * rotvec_deriv2nd + delta_beta;
        }
        else {
            // RVS_WARN("Higher derivative (>2) is not implemented");
            // not implemented
            return SO3Tangentd::ZeroStatic().Coeffs();
        }
    }

    inline CVecXd Eval(double t, unsigned deriv_order = 0) const
    {
        return (*this)(t, deriv_order);
    }

    /**
     * @brief Compute the maximium 1st derivatives of the PolynomialSO3 in
     * the range [t0, t1]. As it is hard to get maximium value analytically, we
     * simply sample some points and return the maximium value among those
     * points.
     *
     * @param t0 path paramater lower range
     * @param t1 path paramater upper range
     * @param sample_num sample number
     * @return CVec3d
     */
    CVec3d MaxAbsDeriv1st(double t0 = 0, double t1 = 1,
                          unsigned sample_num = 2) const
    {
        const unsigned deriv_order = 1;
        std::vector<double> ts = Linspace(t0, t1, sample_num);
        CVec3d omaga = Eval(t0, deriv_order).cwiseAbs();
        for (unsigned i = 1; i < sample_num; ++i) {
            CVec3d omaga_ = Eval(ts[i], deriv_order).cwiseAbs();
            for (unsigned j = 0; j < 3; ++j) {
                if (omaga[j] < omaga_[j]) omaga[j] = omaga_[j];
            }
        }
        return omaga;
    }

    /**
     * @brief Compute the maximium 2nd derivatives of the PolynomialSO3 in
     * the range [t0, t1]. As it is hard to get maximium value analytically, we
     * simply sample some points and return the maximium value among those
     * points.
     *
     * @param t0 path paramater lower range
     * @param t1 path paramater upper range
     * @param sample_num sample number
     * @return CVec3d
     */
    CVec3d MaxAbsDeriv2nd(double t0 = 0, double t1 = 1,
                          unsigned sample_num = 2) const
    {
        const unsigned deriv_order = 2;
        std::vector<double> ts = Linspace(t0, t1, sample_num);
        CVec3d omaga = Eval(t0, deriv_order).cwiseAbs();
        for (unsigned i = 1; i < sample_num; ++i) {
            CVec3d omaga_ = Eval(ts[i], deriv_order).cwiseAbs();
            for (unsigned j = 0; j < 3; ++j) {
                if (omaga[j] < omaga_[j]) omaga[j] = omaga_[j];
            }
        }
        return omaga;
    }

    CVec3d MaxAbsDeriv3rd(double /*t0 = 0*/, double /*t1 = 1*/,
                          unsigned /*sample_num = 2*/) const
    {
        // not implemented, just keep the interface
        return CVec3d::Zero();
    }

    inline unsigned GetDoF() const { return 3; }
    inline unsigned GetDegree() const { return this->m_rotvec->GetDegree(); }
    inline std::string GetExpr() const
    {
        return fmt::format("<PolynomialSO3 dof 3, degree {}>", GetDegree());
    }

    inline SO3d GetStartRotation() const { return this->m_r0; }
    inline PolynomialRn::ConstPtr GetRotVecPolynomial() const
    {
        return this->m_rotvec;
    }

private:
    Polynomial() = default;

    PolynomialRn::Ptr m_rotvec; ///< rotation vector (angle-axis)
    SO3d m_r0; ///< initial Rotation

    /**
     * @brief A is Mat3d matrix, used to compute rotvec_dot from angular_rate.
     *  rotvec_dot = A·angular_rate;
     * @param rotvec rotation vector, aka angle-axis
     * @return Mat3d
     */
    static Mat3d _A(const CVec3d &rotvec)
    {
        const double eps = Constants<double>::Epsilon();
        double angle = rotvec.norm();
        Mat3d mat = Mat3d::Identity();
        if (angle > eps) {
            Mat3d hat = SO3Tangentd(rotvec).Hat();
            mat += 0.5 * hat;
            mat += (1 - angle / 2 * 1 / (std::tan(angle / 2))) / (angle * angle)
                   * (hat * hat);
        }
        return mat;
    }

    /**
     * @brief Compute the inverse of A,  used to compute angular_rate from
     * rotvec_dot
     * angular_rate = InvA · rotvec_dot;
     *
     * @param rotvec rotation vector, aka angle-axis
     * @return Mat3d
     */
    static Mat3d _InvA(const CVec3d &rotvec)
    {
        const double eps = Constants<double>::Epsilon();
        double angle = rotvec.norm();
        Mat3d mat = Mat3d::Identity();
        if (angle > eps) {
            Mat3d hat = SO3Tangentd(rotvec).Hat();
            mat -= (1 - std::cos(angle)) / (angle * angle) * hat;
            mat += (angle - std::sin(angle)) / (angle * angle * angle)
                   * (hat * hat);
        }
        return mat;
    }

    /**
     * @brief DeltaBeta is the non-linear term of the triangle banded matrix
     * right column.
     *
     * @param rotvec rotation vector, aka angle-axis
     * @param rotvec_dot 1st derivative of rotatino vector
     * @return CVec3d
     */
    static CVec3d _DeltaBeta(const CVec3d &rotvec, const CVec3d &rotvec_dot)
    {
        const double eps = Constants<double>::Epsilon();
        CVec3d delta_beta;
        delta_beta.setZero();
        double a = rotvec.norm();
        if (a > eps) {
            double sa = std::sin(a);
            double ca = std::cos(a);
            double a3 = a * a * a;
            double a4 = a * a3;
            double a5 = a * a4;
            double dot = (rotvec.transpose() * rotvec_dot);
            CVec3d cross = rotvec.cross(rotvec_dot);
            delta_beta = -(a * sa + 2 * (ca - 1)) / a4 * dot * cross;
            delta_beta -=
                (2 * a + a * ca - 3 * sa) / a5 * dot * (rotvec.cross(cross));
            delta_beta += (a - sa) / a3 * (rotvec_dot.cross(cross));
        }
        return delta_beta;
    }
};
///@}
} // namespace __RVS_HIDE

using PolynomialSO3 = __RVS_HIDE::Polynomial<SO3d>; // SO3d
///@}
} // namespace RVS
