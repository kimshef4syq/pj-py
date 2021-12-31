// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include "PolynomialRotation.h"
#include "Splines.h"

namespace RVS
{

///@addtogroup Trajectory
///@{
namespace __RVS_HIDE
{

///@addtogroup Trajectory
///@{
/*
 * References
 * http://qspline.sourceforge.net/qspline.pdf
 * http://www.ladispe.polito.it/corsi/meccatronica/02JHCOR/2010-11/Slides/Shuster_Pub_1993h_J_Repsurv_scan.pdf
 * https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.RotationSpline.html?highlight=rotationspline#scipy.spatial.transform.RotationSpline
 */
template <>
class CubicSpline<SO3d>
{
public:
    using Ptr = std::shared_ptr<CubicSpline>;
    using ConstPtr = std::shared_ptr<const CubicSpline>;

    using LieGroup = SO3d;
    using PolynomialElementType = typename Polynomial<SO3d>::ConstPtr;

    CubicSpline(unsigned /*dof*/) : m_knots({0}) {}

    /**
     * @brief Construct a new CubicSpline in SO3 space with first derivative
     * difined at two ends. The spline accrosses all given rotation points at
     * ts, and keeps continuous of 2nd derivative.
     *
     * @param rs coeffs of SO3 rotation points, shape is (m, 4)
     * @param ts path paramaters, aka knots, by default a uniform ts will be
     * computed. spline will accross rs at ts. size is m.
     * @param rd0 coeffs of angular rate (SO3Tangent) at start, default zero
     * @param rd1 coeffs of angular rate (SO3Tangent) at end, default zero
     */
    CubicSpline(const MatXd &rs, const std::vector<double> &ts = {},
                const CVecXd &rd0 = CVec3d::Zero(),
                const CVecXd &rd1 = CVec3d::Zero())
    {
        std::vector<SO3d> ms;
        for (unsigned i = 0; i < rs.rows(); ++i) {
            ms.emplace_back(LieGroup(rs.row(i)));
        }
        _ComputeCubicSpline(ms, ts, rd0, rd1, rd0, rd1,
                            false); // ends acclerations not defined
    }

    /**
     * @brief Construct a new CubicSpline in SO3 space with first and second
     * derivatives at two ends (defined second derivatives not implemented yet).
     * The spline accrosses all given rotation points at ts, and keeps
     * continuous of 2nd derivative.
     *
     * @param rs coeffs of SO3 rotation points, shape is (m, 4)
     * @param ts path paramaters, aka knots, by default a uniform ts will be
     * computed. spline will accross rs at ts. size is m.
     * @param rd0 coeffs of angular rate (SO3Tangent) at start, default zero
     * @param rd1 coeffs of angular rate (SO3Tangent) at end, default zero
     * @param rdd0 coeffs of angular rate derivative (SO3Tangent::Tangent) at
     * start, default zero, not concerned currently
     * @param rdd1 coeffs of angular rate derivative (SO3Tangent::Tangent) at
     * end, default zero, not concerned currently
     */
    CubicSpline(const MatXd &rs, const std::vector<double> &ts,
                const CVecXd &rd0, const CVecXd &rd1,
                [[maybe_unused]] const CVecXd &rdd0,
                [[maybe_unused]] const CVecXd &rdd1)
    {
        std::vector<SO3d> ms;
        for (unsigned i = 0; i < rs.rows(); ++i) {
            ms.emplace_back(LieGroup(rs.row(i)));
        }
        _ComputeCubicSpline(ms, ts, rd0, rd1, rdd0, rdd1,
                            true); // ends acclerations defined
    }

    std::shared_ptr<PathBase<LieGroup>> ToPath() const;

    ///@brief Return the i-th PolynomialSO3
    PolynomialElementType operator[](unsigned i) { return m_polys.at(i); }

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
    inline CVecXd Eval(double t, unsigned deriv_order = 0) const
    {
        return (*this)(t, deriv_order);
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
        unsigned seg_idx = BinSearch(m_knots.data(), m_knots.size(), t, true);
        return (*m_polys[seg_idx])(t - m_knots[seg_idx], deriv_order);
    }

    /**
     * @brief Compute the maximium 1st derivatives of  each PolynomialSO3
     * of the spline. As it is hard to get maximium value
     * analytically, we simply sample some points duing each PolynomialSO3
     * and return the maximium value among those points.
     *
     * @param sample_num sample number
     * @return MatXd shape of (num_segs, dof)
     */
    MatXd MaxAbsDeriv1st(unsigned sample_num = 3) const
    {
        unsigned n = m_polys.size();
        MatXd val(n, 3);
        for (unsigned i = 0; i < n; ++i) {
            val.row(i) = m_polys[i]->MaxAbsDeriv1st(
                0, m_knots[i + 1] - m_knots[i], sample_num);
        }
        return val;
    }

    /**
     * @brief Compute the maximium 2nd derivatives of each PolynomialSO3
     * of the spline. As it is hard to get maximium value
     * analytically, we simply sample some points duing each PolynomialSO3
     * and return the maximium value among those points.
     *
     * @param sample_num sample number
     * @return MatXd shape of (num_segs, dof)
     */
    MatXd MaxAbsDeriv2nd(unsigned sample_num = 3) const
    {
        unsigned n = m_polys.size();
        MatXd val(n, 3);
        for (unsigned i = 0; i < n; ++i) {
            val.row(i) = m_polys[i]->MaxAbsDeriv2nd(
                0, m_knots[i + 1] - m_knots[i], sample_num);
        }
        return val;
    }

    MatXd MaxAbsDeriv3rd([[maybe_unused]] unsigned sample_num = 3) const
    {
        // not implemented, but keep the interface as some algorithms needs
        // these value.
        unsigned n = m_polys.size();
        MatXd val(n, 3);
        val.setZero();
        return val;
    }

    bool PushBack(PolynomialElementType poly, double T)
    {
        // Note, we didn't check the continuity here
        m_polys.emplace_back(poly);
        m_knots.emplace_back(T + m_knots.back());
        return true;
    }

    inline unsigned GetDoF() const { return 3; }
    inline int GetNumOfSegs() const { return m_polys.size(); }
    inline double GetKnotLength() const { return m_knots.back(); }
    inline const std::vector<double> &GetKnots() const { return m_knots; }

    std::string GetExpr() const
    {
        return fmt::format("<CubicSplineRotation with {} segments>",
                           m_polys.size());
    }

private:
    std::vector<PolynomialSO3::ConstPtr> m_polys;
    std::vector<double> m_knots;

    bool _ComputeCubicSpline(
        const std::vector<SO3d> &rs, const std::vector<double> &ts = {},
        const CVecXd &rd0 = CVec3d::Zero(), const CVecXd &rd1 = CVec3d::Zero(),
        [[maybe_unused]] const CVecXd &rdd0 = CVec3d::Zero(),
        [[maybe_unused]] const CVecXd &rdd1 = CVec3d::Zero(),
        bool use_acc_bounds = false)
    {
        unsigned n = rs.size() - 1; // number of segments
        RVS_ENSURE(n >= 1, "Paramater rs.size {} not >= 2", rs.size());
        RVS_ENSURE(rs.size() == ts.size() || ts.size() == 0,
                   "rs.size and ts.size not match");
        CVecXd ts_ = Eigen::Map<const CVecXd>(ts.data(), ts.size());
        CVecXd Ts(n + 1); // the first value is dummy
        Ts.setZero();
        bool update_ts = true;
        if (ts.size()) {
            Ts.segment(1, n) = ts_.segment(1, n) - ts_.segment(0, n);
            update_ts = false;
        }
        // angle-axis vector, SO3Tangent, Theta in paper
        std::vector<CVec3d> rotvecs(n + 1); // first value is dummy
        for (unsigned i = 1; i <= n; ++i) {
            rotvecs[i] = (rs[i] - rs[i - 1]).Coeffs();
            if (update_ts) {
                Ts[i] = rotvecs[i].norm();
            }
        }
        std::vector<CVec3d> angular_rates(n + 1); // angular velocities
        angular_rates[0] = rd0;
        angular_rates[n] = rd1;
        for (unsigned i = 1; i < n; ++i) { // assign initial value
            angular_rates[i] = rotvecs[i] / Ts[i];
        }

        // scipy bounds, used to check the spline result
        // angular_rates[0] = rotvecs[1] / Ts[1];
        // angular_rates[n] = rotvecs[n] / Ts[n];

        // chasing method for banded  diagonal matrix
        // a, b, c, d0  is the diagonal banded matrix non-zero coefficients
        std::vector<Mat3d> a(n - 1); // a[0] is dummy
        std::vector<Mat3d> b(n - 1);
        std::vector<Mat3d> c(n - 1);
        std::vector<CVec3d> d0(n - 1); // without minus delta_beta
        std::vector<Mat3d> As(n + 1); // As[0] is dummpy
        std::vector<Mat3d> h_inv(n - 1);
        std::vector<CVec3d> y(n - 1);
        std::vector<Mat3d> ls(n - 2); // equals to b.inverse() * a
        As[1] = PolynomialSO3::_A(rotvecs[1]);
        for (unsigned i = 0; i <= n - 2; ++i) {
            if (i > 0) {
                a[i] = 2 / Ts[i + 1] * PolynomialSO3::_InvA(rotvecs[i + 1]);
            }
            b[i] = (4 / Ts[i + 1] + 4 / Ts[i + 2]) * Mat3d::Identity();
            As[i + 2] = PolynomialSO3::_A(rotvecs[i + 2]);
            c[i] = (2 / Ts[i + 2]) * As[i + 2];
            d0[i] = 6 / (Ts[i + 1] * Ts[i + 1]) * rotvecs[i + 1]
                    + 6 / (Ts[i + 2] * Ts[i + 2]) * rotvecs[i + 2];
            if (i == 0) {
                d0[i] -= 2 / Ts[1] * PolynomialSO3::_InvA(rotvecs[1])
                         * angular_rates[0];
            }
            else if (i == n - 2) {
                d0[i] -= 2 / Ts[n] * As[n] * angular_rates[n];
            }
        }
        h_inv[0] = b[0].inverse();
        for (unsigned i = 1; i < n - 1; ++i) {
            ls[i - 1] = a[i] * b[i - 1].inverse();
            h_inv[i] = (b[i] - ls[i - 1] * c[i - 1]).inverse();
        }

        unsigned iter = 0;
        double tol_ratio = 1e-9;
        bool converged = false;
        CVec3d angular_rate, delta_angular_rate, allowed_tol;
        CVec3d rotvec_dot, delta_beta;
        CVec3d d_pre, d_cur;
        while (!converged && iter++ < 100) {
            converged = true;
            for (unsigned i = 0; i <= n - 2; ++i) {
                rotvec_dot = As[i + 1] * angular_rates[i + 1];
                delta_beta =
                    PolynomialSO3::_DeltaBeta(rotvecs[i + 1], rotvec_dot);
                d_cur = d0[i] - delta_beta;
                y[i] = i > 0 ? d_cur - ls[i - 1] * d_pre : d_cur;
                d_pre = d_cur;
            }
            angular_rate = h_inv[n - 2] * y[n - 2];
            delta_angular_rate =
                (angular_rates[n - 1] - angular_rate).cwiseAbs();
            allowed_tol =
                tol_ratio * (CVec3d::Ones() + angular_rate.cwiseAbs());
            // converge condition
            if (!VectorCompare(delta_angular_rate, allowed_tol)) {
                converged = false;
            }
            angular_rates[n - 1] = angular_rate;
            for (int i = n - 3; i >= 0; --i) {
                angular_rate = h_inv[i] * (y[i] - c[i] * angular_rates[i + 2]);
                delta_angular_rate =
                    (angular_rates[i + 1] - angular_rate).cwiseAbs();
                allowed_tol =
                    tol_ratio * (CVec3d::Ones() + angular_rate.cwiseAbs());
                if (!VectorCompare(delta_angular_rate, allowed_tol)) {
                    converged = false;
                }
                angular_rates[i + 1] = angular_rate;
            }
        }
        if (!converged) {
            RVS_ERROR("Failed to compute Rotation cubic spline");
        }

        m_knots = std::vector<double>{0};
        // // if not with ends accelerations defined
        // for (unsigned i = 0; i < n; ++i) {
        //     PolynomialSO3::Ptr poly = PolynomialSO3::Cubic(
        //         rs[i], rs[i + 1], SO3Tangentd(angular_rates[i]),
        //         SO3Tangentd(angular_rates[i + 1]), Ts[i + 1]);
        //     PushBack(poly, Ts[i + 1]);
        // }

        // if cubic with defined accelerations, there will be two more virtual
        // points added in first and last segment. This hasn't been implemented
        // for Rotation space. But to keep the number of segments to be same, we
        // just simple split the frist and last semgnet into two parts.
        for (unsigned i = 0; i < n; ++i) {
            PolynomialSO3::Ptr poly = PolynomialSO3::Cubic(
                rs[i], rs[i + 1], SO3Tangentd(angular_rates[i]),
                SO3Tangentd(angular_rates[i + 1]), Ts[i + 1]);
            if (use_acc_bounds && (i == 0 || i == n - 1)) {
                double Tm = Ts[i + 1] * 0.5;
                SO3d r_mid((*poly)(Tm));
                SO3Tangentd angular_rate_mid((*poly)(Tm, 1));
                PolynomialSO3::Ptr poly1 = PolynomialSO3::Cubic(
                    rs[i], r_mid, SO3Tangentd(angular_rates[i]),
                    angular_rate_mid, Tm);
                PolynomialSO3::Ptr poly2 =
                    PolynomialSO3::Cubic(r_mid, rs[i + 1], angular_rate_mid,
                                         SO3Tangentd(angular_rates[i + 1]), Tm);
                PushBack(poly1, Tm);
                PushBack(poly2, Tm);
            }
            else {
                PushBack(poly, Ts[i + 1]);
            }
        }
        return true;
    };
};
///@}
} // namespace __RVS_HIDE

using CubicSplineSO3 = __RVS_HIDE::CubicSpline<SO3d>;

///@}
} // namespace RVS
