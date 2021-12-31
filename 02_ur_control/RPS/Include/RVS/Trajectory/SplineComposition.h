// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once

#include "SplinesRotation.h"

namespace RVS
{
///@addtogroup Trajectory
///@{
namespace __RVS_HIDE
{
///@addtogroup Trajectory
///@{
/**
 * @brief Compose two Spline into one, kind of like LieGroup DirectProduct. The
 * two splines share the same knots vector. The interface is same with @ref
 * PSpline.
 *
 * @tparam SplineType1
 * @tparam SplineType2
 */
template <typename SplineType1, typename SplineType2>
class SplineComposition
{
public:
    using Ptr = std::shared_ptr<SplineComposition>;
    using ConstPtr = std::shared_ptr<const SplineComposition>;
    using _LieGroup = DirectProduct<typename SplineType1::LieGroup,
                                    typename SplineType2::LieGroup>;
    using LieGroup =
        std::conditional_t<std::is_same_v<_LieGroup, R3xSO3d>, SE3d, _LieGroup>;
    using PolynomialElementType =
        std::pair<typename SplineType1::PolynomialElementType,
                  typename SplineType2::PolynomialElementType>;

    SplineComposition(unsigned /*dof*/)
        : m_spline1(std::make_shared<SplineType1>(m_dof1)),
          m_spline2(std::make_shared<SplineType2>(m_dof2))
    {
        // dof == m_dof or dof == LieGroup::NUM_PARAMS
    }

    ~SplineComposition() { RVS_TRACE("Destruct SplineComposition"); }

    SplineComposition(const MatXd &wps, const std::vector<double> &ts,
                      const CVecXd &v0 = CVecXd::Zero(m_dof),
                      const CVecXd &v1 = CVecXd::Zero(m_dof))
    {
        // Rxd's NUM_PARAMS is not fixed
        static_assert(!std::is_same_v<SplineType1,
                                      Rxd> && !std::is_same_v<SplineType2, Rxd>,
                      "Rxd is not supported for SplineComposition, use fixed "
                      "DoF please!");

        m_spline1 = std::make_shared<SplineType1>(
            wps.leftCols<SplineType1::LieGroup::NUM_PARAMS>(), ts,
            v0.topRows<m_dof1>(), v1.topRows<m_dof1>());

        m_spline2 = std::make_shared<SplineType2>(
            wps.rightCols<SplineType2::LieGroup::NUM_PARAMS>(), ts,
            v0.topRows<m_dof2>(), v1.topRows<m_dof2>());
    }

    SplineComposition(const MatXd &wps, const std::vector<double> &ts,
                      const CVecXd &v0, const CVecXd &v1,
                      [[maybe_unused]] const CVecXd &a0,
                      [[maybe_unused]] const CVecXd &a1)
    {
        // Rxd's NUM_PARAMS is not fixed
        static_assert(!std::is_same_v<SplineType1,
                                      Rxd> && !std::is_same_v<SplineType2, Rxd>,
                      "Rxd is not supported for SplineComposition, use fixed "
                      "DoF please!");

        m_spline1 = std::make_shared<SplineType1>(
            wps.leftCols<SplineType1::LieGroup::NUM_PARAMS>(), ts,
            v0.topRows<m_dof1>(), v1.topRows<m_dof1>(), a0.topRows<m_dof1>(),
            a1.topRows<m_dof1>());

        m_spline2 = std::make_shared<SplineType2>(
            wps.rightCols<SplineType2::LieGroup::NUM_PARAMS>(), ts,
            v0.bottomRows<m_dof2>(), v1.bottomRows<m_dof2>(),
            a0.bottomRows<m_dof2>(), a1.bottomRows<m_dof2>());
    }

    std::shared_ptr<PathBase<LieGroup>> ToPath() const;

    CVecXd operator()(double t, unsigned deriv_order = 0) const
    {
        CVecXd value1 = (*m_spline1)(t, deriv_order);
        CVecXd value2 = (*m_spline2)(t, deriv_order);
        CVecXd value(value1.size() + value2.size());
        value.topRows(value1.size()) = value1;
        value.bottomRows(value2.size()) = value2;
        return value;
    }

    CVecXd Eval(double t, unsigned deriv_order = 0) const
    {
        return (*this)(t, deriv_order);
    }

    MatXd MaxAbsDeriv1st() const
    {
        MatXd value1 = m_spline1->MaxAbsDeriv1st();
        MatXd value2 = m_spline2->MaxAbsDeriv1st();
        MatXd value(value1.rows(), m_dof);
        value.leftCols(value1.cols()) = value1;
        value.rightCols(value2.cols()) = value2;
        return value;
    }

    MatXd MaxAbsDeriv2nd() const
    {
        MatXd value1 = m_spline1->MaxAbsDeriv2nd();
        MatXd value2 = m_spline2->MaxAbsDeriv2nd();
        MatXd value(value1.rows(), m_dof);
        value.leftCols(value1.cols()) = value1;
        value.rightCols(value2.cols()) = value2;
        return value;
    }

    MatXd MaxAbsDeriv3rd() const
    {
        MatXd value1 = m_spline1->MaxAbsDeriv3rd();
        MatXd value2 = m_spline2->MaxAbsDeriv3rd();
        MatXd value(value1.rows(), m_dof);
        value.leftCols(value1.cols()) = value1;
        value.rightCols(value2.cols()) = value2;
        return value;
    }
    inline unsigned GetDoF() const { return m_dof; }
    inline double GetKnotLength() const { return m_spline1->GetKnotLength(); }
    inline const std::vector<double> &GetKnots() const
    {
        // spline1 and spline2 have same knots
        return m_spline1->GetKnots();
    }

    inline unsigned GetNumOfSegs() const
    {
        return std::max<unsigned>(m_spline1->GetNumOfSegs(),
                                  m_spline2->GetNumOfSegs());
    }

    PolynomialElementType operator[](unsigned i) const
    {
        return std::make_pair((*m_spline1)[i], (*m_spline2)[i]);
    }

    bool PushBack(PolynomialElementType poly_pair, double T)
    {
        return m_spline1->PushBack(poly_pair.first, T)
               && m_spline2->PushBack(poly_pair.second, T);
    }

    typename SplineType1::ConstPtr GetSpline1() { return m_spline1; }
    typename SplineType2::ConstPtr GetSpline2() { return m_spline2; }
    std::string GetExpr() const
    {
        return fmt::format("<Composition of:\n{}\n{}>", m_spline1->GetExpr(),
                           m_spline2->GetExpr());
    }

private:
    typename SplineType1::Ptr m_spline1;
    typename SplineType2::Ptr m_spline2;

    static constexpr int m_dof1 = SplineType1::LieGroup::DOF;
    static constexpr int m_dof2 = SplineType2::LieGroup::DOF;
    static constexpr int m_dof = m_dof1 + m_dof2;
};
///@}
} // namespace __RVS_HIDE

using CubicSplineSE3 =
    __RVS_HIDE::SplineComposition<CubicSplineR3, CubicSplineSO3>;

///@}
} // namespace RVS