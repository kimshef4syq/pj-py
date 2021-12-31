// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
#include "Polynomial.h"
#include <RVS/Trajectory/PathBase.h>

// maximium degree of polynomial supported by BSpline currently
#define MAX_DoP 6

namespace RVS
{
///@addtogroup Trajectory
///@{
namespace __RVS_HIDE
{
///@addtogroup Trajectory
///@{
using Tensor3D = std::vector<MatXd>;

/**
 * @brief PSpline, piecewise polynoimial form spline, could be used to represent
 * a variety of high dimension space curves. The curve consists of many piece of
 * segments. Each segment is represented by a Polynomial. With PSpline, it's
 * very convinent to evaluate polynomial value and its derivatives.
 *
 */
template <typename PointType>
class PSpline
{
public:
    using Ptr = std::shared_ptr<PSpline>;
    using ConstPtr = std::shared_ptr<const PSpline>;

    using LieGroup = PointType;
    using PolynomialElementType = typename Polynomial<PointType>::ConstPtr;

    PSpline(unsigned dof) : m_knots(std::vector<double>(1, 0)), m_dof(dof) {}

    std::shared_ptr<PathBase<PointType>> ToPath() const;

    virtual ~PSpline() { RVS_TRACE("Destructing PSpline"); }

    /**
     * @brief Construct a new Piecewise Polynomial object
     *
     * @param coeffs polynomial coefficients of each polynomial segment of each
     * DoF, m x (p+1) x dof, m is number of segments
     * @param knots path paramater knots of each segment, all DoF share the
     * same knots, (m + 1)
     */
    PSpline(const Tensor3D &coeffs, const std::vector<double> &knots);

    /**
     * @brief Eval the spline  value of given derivative order at s
     *
     * @param s path paramater
     * @param deriv_order dirivative order, >= 0; if 0, path position was
     * returned.
     * @return CVecXd
     */
    CVecXd Eval(double s, unsigned deriv_order = 0) const
    {
        return (*this)(s, deriv_order);
    }

    CVecXd operator()(double s, unsigned deriv_order = 0) const;

    ///@brief Get the i-th polynomial
    PolynomialElementType operator[](unsigned i) const { return m_polys.at(i); }

    /**
     * @brief Get the max 1st derivative value of each segment of each dof
     * @return MatXd seg_num x dof
     */
    MatXd MaxAbsDeriv1st() const;

    /**
     * @brief Get the max second derivative value of each segment of each dof
     * @return MatXd seg_num x dof
     */
    MatXd MaxAbsDeriv2nd() const;

    /**
     * @brief Get the max thrid derivative value of each segment of each dof
     * @return MatXd seg_num x dof
     */
    MatXd MaxAbsDeriv3rd() const;

    /**
     * @brief Push a new Polynomial to the end of current p-form spline.
     * Continuity is not checked.
     *
     * @param poly Polynomial of same type
     * @param T duration of the Polynomial
     */
    inline bool PushBack(PolynomialElementType poly, double T = 1)
    {
        // Note, we didn't check the continuity here
        if (GetDoF() != poly->GetDoF()) {
            RVS_ERROR("Cannot merge polynomial with different dof");
            return false;
        }
        m_polys.emplace_back(poly);
        m_knots.emplace_back(T + m_knots.back());
        return true;
    }

    /**
     * @brief Push a new Spline to the end of current p-form spline. Continuity
     * is not checked.
     *
     * @param spline PSpline of same point type
     */
    inline bool PushBack(PSpline::ConstPtr spline)
    {
        // Note, we didn't check the continuity here
        if (GetDoF() != spline->GetDoF()) {
            RVS_ERROR("Cannot merge spline with different dof");
            return false;
        }
        const std::vector<double> &knots = spline->GetKnots();
        unsigned seg_num = spline->GetNumOfSegs();
        for (unsigned i = 0; i < seg_num; ++i) {
            PushBack((*spline)[i], knots[i + 1] - knots[i]);
        }
        return true;
    }

    ///@brief Get DoF of spline.
    inline unsigned GetDoF() const { return m_dof; }

    ///@brief Get number of polynomial segments of spline.
    inline unsigned GetNumOfSegs() const { return m_polys.size(); }

    ///@brief Get length of knot, aka path parameter or path length.
    inline double GetKnotLength() const { return m_knots.back(); }

    ///@brief Get path knots vector.
    inline const std::vector<double> &GetKnots() const { return m_knots; }

    std::string GetExpr() const
    {
        return fmt::format("<PSpline, DoF {}, num of segs: {}>", GetDoF(),
                           GetNumOfSegs());
    }

protected:
    std::vector<double>
        m_knots; ///< knots vector, the start and end timestamps
                 ///< of each polynomial, number of knots = 1 + seg_num
    std::vector<PolynomialElementType>
        m_polys; ///< each polynomial may be have different degree, but must
                 ///< same DoF
    unsigned m_dof; ///< DoF
};

/**
 * @brief CubicSpline construction. Compution of CubicSpline is faster than
 * compution of BSpline. But BSpline could keep jerk contiunous.
 *
 */
template <typename PointType>
class CubicSpline : public PSpline<PointType>
{
public:
    using Ptr = std::shared_ptr<CubicSpline>;
    using ConstPtr = std::shared_ptr<const CubicSpline>;
    using LieGroup = PointType;

    ///@brief Construct an CubicSpline of given dof.
    CubicSpline(unsigned dof) : PSpline<PointType>(dof) {}

    /**
     * @brief Construct a new CubicSpline with defined initial/end velocity. The
     * spline will accross points at us.
     *
     * @param points points, size n x dof
     * @param us path paramaters of points, size n
     * @param v0 start velocity, size dof
     * @param v1 end velocity, size dof
     */
    CubicSpline(const MatXd &points, const std::vector<double> &us,
                const CVecXd &v0 = CVecXd::Zero(0),
                const CVecXd &v1 = CVecXd::Zero(0));

    /**
     * @brief Construct a new CubicSpline with defined initial/end
     * velocity/acceleration
     *
     * @param points points, size n x dof
     * @param us path paramaters of points, size n
     * @param v0 start velocity, size dof
     * @param v1 end velocity, size dof
     * @param a0 start acceleration, size dof
     * @param a1 end acceleration, size dof
     */
    CubicSpline(const MatXd &points, const std::vector<double> &us,
                const CVecXd &v0, const CVecXd &v1, const CVecXd &a0,
                const CVecXd &a1);

    /**
     * @brief Construct a new CubicSpline approximating to given waypoints
     * instead of interpolation.
     *
     * @param points points, size n x dof
     * @param us path paramaters of points, size n
     * @param weights weights of each point, the larger value means the curve
     * should be closer to the point
     * @param smooth smooth ratio， [0, 1.0], small values get a smoother curve.
     * At the two extremes, 0 means fit the waypoints with a line; 1 means the
     * general natural cubic spline interpolation.
     */
    CubicSpline(const MatXd &points, const std::vector<double> &us,
                const std::vector<double> &weights = {}, double smooth = 0.9);

    virtual ~CubicSpline() { RVS_TRACE("Destructing CubicSpline"); }

    inline unsigned GetDegree() const { return this->m_dop; }

private:
    static constexpr unsigned m_dop = 3; ///< degree of cubic spline

    ///@brief Initial/end velocity/acceleration defined cubic spline
    void _ComputeCubicSpline(MatXd &points, const std::vector<double> &us,
                             const CVecXd &v0, const CVecXd &v1,
                             const CVecXd &a0, const CVecXd &a1);

    ///@brief Initial/end velocity defined cubic spline
    void _ComputeCubicSpline(const MatXd &points, const std::vector<double> &us,
                             const CVecXd &v0, const CVecXd &v1);
};

/**
 * @brief Basis function form spline. Note: BSpline order is differnt from
 * degree. Generally degree = order + 1. BSpline degree is the max power
 * coefficient of a piecewise polynomial. Reference book "Trajectory Planning
 * for Automatic Machines and Robots".
 */
template <typename PointType>
class BSpline : public PSpline<PointType>
{
public:
    using Ptr = std::shared_ptr<BSpline>;
    using ConstPtr = std::shared_ptr<const BSpline>;
    using LieGroup = PointType;

    /**
     * @brief Construct a new BSpline object, equation "n = m + p + 1" must be
     * kept
     *
     * @param control_points controler points of BSpline, note: control points
     * are not accrossed except the first/last ones and its size not equals to
     * knots's size, shape: m x dof
     * @param knots BSpline knot vector, shape: n
     * @param polynomial_degree polynomial degree, aka. p
     */
    BSpline(const MatXd &control_points, const std::vector<double> &knots,
            unsigned polynomial_degree);

    ~BSpline() { RVS_TRACE("Destructing BSpline"); }

    inline const std::vector<double> &GetKnotsBForm() const
    {
        return m_knots_bform;
    }

    /**
     * @brief By default we use PSpline to evaluate, but you can call this
     * function to evaluate with BSpline. PSpline should be a little faster.
     *
     * @param s path paramater
     * @param deriv_order derivate order, eg. if d == 0, BSpline point is
     * returned; if d == 1, BSpline tangent is returned.
     * @return CVecXd
     */
    CVecXd EvalWithBForm(double s, unsigned deriv_order = 0) const;

    inline const MatXd &GetControlPoints() const { return m_control_points; }
    inline unsigned GetDegree() const { return m_dop; }

protected:
    BSpline(unsigned dof) : PSpline<PointType>(dof) {}

    ///@brief Convert from B-form to Piecewise Polynomial form (pp-form)
    void _ComputePPForm();

    /**
     * @brief Compute non-zero basis functions of BSpline at u. Note: BSpline
     * order is differnt from degree. Generall degree = order + 1. BSpline
     * degree is the max power coefficient of a piecewise polynomial.
     *
     * @param span_idx index of knot span including u
     * @param s path paramater
     * @param p degree of the spline
     * @param knots  knot vector
     * @param basis_value[out] value of the nonvanishing basis function at u
     */
    static void BasisFuns(int span_idx, double s, int p, const double knots[],
                          double basis_value[]);

    /**
     * @brief value of the independent variable
     *
     * @param s path paramater
     * @param knots knot vector
     * @param n_knot length of knots - 1, aka. max index of knots
     * @param p degree of the spline
     * @return unsigned int,  an idx, s.t. knots[idx] <= u < knots[idx+1]
     * if u < knots[n_knot], else n_knot -1.
     */
    static inline unsigned WhichSpan(double &s, const double knots[],
                                     int n_knot, int p)
    {
        return BinSearch(knots + p, n_knot - 2 * p + 1, s, true) + p;
    }

    /**
     * @brief Get value of the B-spline at u
     *
     * @param s path paramater
     * @param knots Knot vector
     * @param n_knot length of knots - 1, aka. max index of knots
     * @param p degree of the spline
     * @param control_points control points vector, m x dof, row major
     * @param dof dimensions of a control point
     * @param spline_value[out] value of the B-spline at u
     */
    static void BSplinePoint(double s, const double knots[], int n_knot, int p,
                             const double control_points[], int dof,
                             double spline_value[]);

    /**
     * @brief Get values of B-spline basis functions and theirs derivatives at u
     *
     * @param s path paramater
     * @param span_idx index of knot span including u
     * @param p degree of the spline
     * @param deriv_order max order of differentiation of B-spline
     * @param knots Knot vector
     * @param derivs[out] values of B-spline basis functions and theirs
     * derivatives at u
     */
    static void DersBasisFuns(double s, int span_idx, int p, int deriv_order,
                              const double knots[],
                              double derivs[MAX_DoP + 1][MAX_DoP + 1]);

protected:
    unsigned m_dop; ///< bspline degree

    // following paramaters used to evaluate spline value with basis functions
    std::vector<double> m_knots_bform; ///< knots vector
    MatXd m_control_points; ///< control points
    unsigned m_max_idx_bform_knots; ///< max index of bform knots vector
};

template <typename PointType>
class BSpline4th : public BSpline<PointType>
{
public:
    using Ptr = std::shared_ptr<BSpline4th>;
    using ConstPtr = std::shared_ptr<const BSpline4th>;
    using LieGroup = PointType;

    BSpline4th(unsigned dof) : BSpline<PointType>(dof) { this->m_dop = 4; }

    /**
     * @brief Construct a new BSpline4th, the spline will accross points at
     * given instants `ts`. ts.size() == points.size().
     *
     * @param points interpolating points that need to be accrossed
     * @param ts  path paramaters of given points, aka. knots vector
     * @param v0 start 1st derivative
     * @param v1 end 1st derivative
     * @param a0 start 2nd derivative
     * @param a1 end 2nd derivative
     */
    BSpline4th(const MatXd &points, const std::vector<double> &ts,
               const CVecXd &v0 = CVecXd::Zero(0),
               const CVecXd &v1 = CVecXd::Zero(0),
               const CVecXd &a0 = CVecXd::Zero(0),
               const CVecXd &a1 = CVecXd::Zero(0));

private:
    void _ComputeKnots(const std::vector<double> &ts);
    bool _ComputeControlPoints(const MatXd &points,
                               const std::vector<double> &ts, const CVecXd &v0,
                               const CVecXd &v1, const CVecXd &a0,
                               const CVecXd &a1);
};
///@}
} // namespace __RVS_HIDE

using PSplineRn = __RVS_HIDE::PSpline<Rxd>;
using CubicSplineRn = __RVS_HIDE::CubicSpline<Rxd>;
using BSplineRn = __RVS_HIDE::BSpline<Rxd>;
using BSpline4thRn = __RVS_HIDE::BSpline4th<Rxd>;
using PSplineR3 = __RVS_HIDE::PSpline<R3d>;
using CubicSplineR3 = __RVS_HIDE::CubicSpline<R3d>;

///@}
} // namespace RVS
