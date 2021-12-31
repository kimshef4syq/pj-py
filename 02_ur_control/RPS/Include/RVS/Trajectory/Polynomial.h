// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include <RVS/Trajectory/PSpline1d.h>
#include <RVS/Common/Macros.h>
#include <vector>

namespace RVS
{

///@addtogroup Trajectory
///@{
namespace __RVS_HIDE
{
///@addtogroup Trajectory
///@{
template <typename PointType>
class Polynomial;

/**
 * @brief One piece of multi-dimension polynomial expression.
 *
 * @tparam PointType Currently Rxd or SO3d or double
 */
template <typename PointType>
class Polynomial : public std::enable_shared_from_this<Polynomial<PointType>>
{
public:
    RVS_DECLARE_PTR_MEMBER(Polynomial<PointType>);

    ///@brief Generate linear polynomial giving start and end state
    static Ptr Linear(const CVecXd &q0, const CVecXd &q1, double T = 1.0)
    {
        const unsigned dof = q0.size();
        MatXd coeffs(2, dof);
        for (unsigned i = 0; i < dof; ++i) {
            coeffs.col(i) =
                Polynomial<double>::Linear(q0[i], q1[i], T)->GetCoeffs();
        }
        return std::make_shared<Polynomial<PointType>>(coeffs);
    }

    ///@brief Generation cubic polynomial giving start and end state
    static Ptr Cubic(const CVecXd &q0, const CVecXd &q1, const CVecXd &v0,
                     const CVecXd &v1, double T = 1.0)
    {
        const unsigned dof = q0.size();
        MatXd coeffs(4, dof);
        for (unsigned i = 0; i < dof; ++i) {
            coeffs.col(i) =
                Polynomial<double>::Cubic(q0[i], q1[i], v0[i], v1[i], T)
                    ->GetCoeffs();
        }
        return std::make_shared<Polynomial<PointType>>(coeffs);
    }

    ///@brief Generation quintic polynomial giving start and end state
    static Ptr Quintic(const CVecXd &q0, const CVecXd &q1, const CVecXd &v0,
                       const CVecXd &v1, const CVecXd &a0, const CVecXd &a1,
                       double T = 1.0)
    {
        const unsigned dof = q0.size();
        MatXd coeffs(6, dof);
        for (unsigned i = 0; i < dof; ++i) {
            coeffs.col(i) = Polynomial<double>::Quintic(q0[i], q1[i], v0[i],
                                                        v1[i], a0[i], a1[i], T)
                                ->GetCoeffs();
        }
        return std::make_shared<Polynomial<PointType>>(coeffs);
    }

    ///@brief Default construction method
    Polynomial() = default;

    /**
     * @brief Construct a new Polynomial object from given coefficients. The
     * i-th DoF polynomial is:
     *  \f$ p_i(t) = c_{0, i} + c_{1, i} \cdot t + c_{2, i} \cdot t^2 + c_{3, i}
     * \cdot t^3 + \cdots \f$
     *
     * @param coeffs polynomial coefficients
     */
    Polynomial(const MatXd &coeffs)
        : m_coeffs(coeffs), m_dop(coeffs.rows() - 1), m_dof(coeffs.cols())
    {
        static_assert(
            std::is_same_v<PointType, Rxd> || std::is_same_v<PointType, R3d>,
            "Unsupported PointType, only Rxd | R3d is supported");
        RVS_ENSURE(coeffs.cols() > 0, "Paramater coeffs's col size is zero");
        RVS_ENSURE(coeffs.rows() > 0, "Paramater coeffs's row size is zero");
    }

    ///@brief Evaluate polynomial value at t of given deriv_order
    inline CVecXd operator()(double t, unsigned deriv_order = 0) const
    {
        if (deriv_order > 0) {
            return (*Deriv(deriv_order))(t, 0);
        }
        CVecXd v(m_coeffs.row(m_dop));
        for (unsigned i = 0; i < m_dop; ++i) {
            v *= t;
            v += m_coeffs.row(m_dop - i - 1);
        }
        return v;
    }

    ///@brief Evaluate polynomial value at t of given deriv_order
    inline CVecXd Eval(double t, unsigned deriv_order = 0) const
    {
        return (*this)(t, deriv_order);
    }

    ///@brief Find minium and maxium value between t0 and t1
    std::pair<CVecXd, CVecXd> MinMaxValue(double t0 = 0, double t1 = 1) const
    {
        CVecXd val_min = (*this)(t0);
        CVecXd val_max = (*this)(t1);
        for (unsigned j = 0; j < m_dof; ++j) {
            if (val_max[j] < val_min[j]) {
                std::swap(val_min[j], val_max[j]);
            }
        }
        std::vector<CVecXd> roots_all = Deriv(1)->Roots(t0, t1);
        for (unsigned j = 0; j < m_dof; ++j) {
            const CVecXd &roots = roots_all[j];
            for (unsigned i = 0; i < roots.size(); ++i) {
                if (t0 <= roots[i] && roots[i] <= t1) {
                    CVecXd val = (*this)(roots[i]);
                    if (val[j] > val_max[j]) {
                        val_max[j] = val[j];
                    }
                    else if (val[j] < val_min[j]) {
                        val_min[j] = val[j];
                    }
                }
            }
        }
        return {val_min, val_max};
    }

    ///@brief Find maxium abs value between t0 and t1
    CVecXd MaxAbsValue(double t0 = 0, double t1 = 1) const
    {
        auto [vmin, vmax] = MinMaxValue(t0, t1);
        vmin = vmin.cwiseAbs();
        vmax = vmax.cwiseAbs();
        for (unsigned j = 0; j < m_dof; ++j) {
            if (vmin[j] > vmax[j]) vmax[j] = vmin[j];
        }
        return vmax;
    }

    ///@brief Find maxium first derivative abs value between t0 and t1
    inline CVecXd MaxAbsDeriv1st(double t0 = 0, double t1 = 1) const
    {
        return Deriv(1)->MaxAbsValue(t0, t1);
    }

    ///@brief Find maxium second derivative abs value between t0 and t1
    inline CVecXd MaxAbsDeriv2nd(double t0 = 0, double t1 = 1) const
    {
        return Deriv(2)->MaxAbsValue(t0, t1);
    }

    ///@brief Find maxium third derivative abs value between t0 and t1
    inline CVecXd MaxAbsDeriv3rd(double t0 = 0, double t1 = 1) const
    {
        return Deriv(3)->MaxAbsValue(t0, t1);
    }

    ///@brief Compute the derivative polynomial of given deriv_order
    ConstPtr Deriv(unsigned deriv_order = 1) const
    {
        if (deriv_order == 0) {
            return this->shared_from_this();
        }
        else if (m_dop == 0) {
            return std::make_shared<Polynomial<PointType>>(
                MatXd::Zero(1, m_dof));
        }
        else if (deriv_order <= m_cached_derivs.size()
                 && m_cached_derivs[deriv_order - 1]) {
            return m_cached_derivs[deriv_order - 1];
        }
        else if (deriv_order == 1) {
            MatXd coeffs(m_coeffs.bottomRows(m_dop));
            for (unsigned i = 1; i < m_dop; ++i) {
                coeffs.row(i) *= (i + 1);
            }
            m_cached_derivs[deriv_order - 1] =
                std::make_shared<Polynomial<PointType>>(coeffs);
            return m_cached_derivs[deriv_order - 1];
        }
        else {
            return Deriv(1)->Deriv(deriv_order - 1);
        }
    }

    ///@brief Compute roots between t0 and t1
    std::vector<CVecXd> Roots(double t0 = 0, double t1 = 1) const
    {
        std::vector<CVecXd> roots_all;
        for (unsigned i = 0; i < m_dof; ++i) {
            const CVecXd &coeffs = m_coeffs.col(i);
            std::vector<double> roots;
            switch (m_dop) {
            case 0: {
            } break;
            case 1: {
                roots = SolveLinearEquation(coeffs[0], coeffs[1]);
            } break;
            case 2: {
                roots = SolveQuadraticEquation(coeffs[0], coeffs[1], coeffs[2]);
            } break;
            case 3: {
                roots = SolveCubicEquation(coeffs[0], coeffs[1], coeffs[2],
                                           coeffs[3]);
            } break;
            default: {
                // https://yutsumura.com/companion-matrix-for-a-polynomial/
                MatXd companion(m_dop, m_dop);
                companion.topLeftCorner(1, m_dop).setZero();
                companion.bottomLeftCorner(m_dop - 1, m_dop).setIdentity();
                companion.rightCols<1>() = coeffs.segment(0, m_dop);
                companion.rightCols<1>() /= coeffs[m_dop];
                Eigen::EigenSolver<MatXd> solver(companion);
                auto eigenvalues = solver.eigenvalues();
                for (unsigned k = 0; k < eigenvalues.size(); ++k) {
                    if (std::abs(eigenvalues[k].imag())
                        < Constants<double>::Epsilon()) { // just want real root
                        roots.emplace_back(eigenvalues[k].real());
                    }
                }
            } break;
            }
            std::vector<double> roots_in_range;
            for (const auto &root : roots) {
                if (t0 <= root && root <= t1) {
                    roots_in_range.emplace_back(root);
                }
            }
            roots_all.emplace_back(Eigen::Map<CVecXd>(roots_in_range.data(),
                                                      roots_in_range.size()));
        }
        return roots_all;
    }

    ///@brief Get string expression of polynomial
    const std::string &GetExpr() const
    {
        if (!m_expr.size()) {
            m_expr =
                fmt::format("<PolynomialRn dof {} degree {}", m_dof, m_dop);
            for (unsigned j = 0; j < m_dof; ++j) {
                m_expr += fmt::format("\ndof {}: ", j + 1);
                for (unsigned i = 0; i <= m_dop; ++i) {
                    m_expr += i == 0   ? fmt::format("{:.2f}", m_coeffs(i, j))
                              : i == 1 ? fmt::format(" + ({:.2f})*t",
                                                     m_coeffs(i, j), i)
                                       : fmt::format(" + ({:.2f})*t^{}",
                                                     m_coeffs(i, j), i);
                }
            }
            m_expr += " >";
        }
        return m_expr;
    }

    ///@brief Get polynomial coefficients.
    inline const MatXd &GetCoeffs() const { return m_coeffs; }

    ///@brief Get degree of polynomial
    inline unsigned GetDegree() const { return m_dop; }

    ///@brief Get DoF of polynomial
    inline unsigned GetDoF() const { return m_dof; }

private:
    MatXd m_coeffs; ///< coeffs of polynomiala, (m_dop + 1) x dof
    unsigned m_dop; ///< degree of polynomial
    unsigned m_dof; ///< DoF
    mutable std::string m_expr; ///< string expression
    mutable std::array<ConstPtr, 3> m_cached_derivs; ///< cached derivatives
};
///@}
} // namespace __RVS_HIDE

using PolynomialRn = __RVS_HIDE::Polynomial<Rxd>; // Rxd
using PolynomialR3 = __RVS_HIDE::Polynomial<R3d>; // R3d

///@}
} // namespace RVS
