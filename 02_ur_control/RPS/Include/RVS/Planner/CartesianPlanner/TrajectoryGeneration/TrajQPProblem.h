#pragma once
#include <RVS/OptSolver/OSQPAdapter.h>
#include <RVS/Planner/Core/Types.h>

namespace RVS
{

struct NodeEqCntProfile;

struct NodeIneqCntProfile;

struct ContinousCntProfile;

struct SegmentIneqCntProfile;

RVS_CLASS_FORWARD(TrajQPSolver);

/**
 * @brief TrajQPSolver define a abstract interface for optimal polynomial spline
 * trajectory solver based on quadratic programming
 */
class TrajQPProblem
{
public:
    explicit TrajQPProblem(const CVecXd &ts, double poly_order);

    virtual ~TrajQPProblem() {}

    /** @brief set the optimizing order of the entile spline trajectory */
    void SetOptimizeOrder(const size_t optimize_order);

    /** @brief add a node equlity constraint to the QP problem */
    virtual bool AddNodeEqCnt(const NodeEqCntProfile node_eq_cnt) = 0;

    /** @brief add a node inequlity constraint to the QP problem */
    virtual bool AddNodeIneqCnt(const NodeIneqCntProfile node_ineq_cnt) = 0;

    /** @brief add a continous constraint to the QP problem */
    virtual bool AddContinousCnt(const ContinousCntProfile continous_cnt) = 0;

    /** @brief add a segment inequlity constraint to the QP problem*/
    virtual bool
    AddSegmentIneqCnt(const SegmentIneqCntProfile seg_ineq_cnt) = 0;

    /** @brief construct and solve QP problem */
    virtual bool Solve(CVecXd &poly_coeffs);

    /** @brief get number of segments*/
    inline const size_t &GetNumSeg() const noexcept { return m_num_seg; }

    /** @brief get order of the polynomial*/
    inline const size_t &GetPolyOrder() const noexcept { return m_poly_order; }

    /** @brief get number of variables*/
    inline const size_t &GetNumVar() const noexcept { return m_num_var; }

protected:
    /** @brief Get decision variable index from a segment index */
    inline size_t GetVarIdx(size_t seg_idx) const
    {
        return seg_idx * (m_poly_order + 1);
    }

    // numpy-like matrix concatenating to generator the A and G matrix of QP
    // canonical form
    void ConcatenateMat(const std::list<MatXd> &mat_list,
                        MatXd &mat_dest) const;

    // numpy-like matrix concatenating to generator the b, h1 and h2 vector of
    // QP canonical form
    void ConcatenateMat(const std::list<CVecXd> &mat_list,
                        CVecXd &mat_dest) const;


    // time stamps of each waypoints
    CVecXd m_ts;

    // time period of each segments
    CVecXd m_Ts;

    // polynomial order
    size_t m_poly_order;

    // number of segment
    size_t m_num_seg;

    // number of variables
    size_t m_num_var;

    // list of constraint matices
    std::list<MatXd> m_Aeq, m_G_ineq;

    std::list<CVecXd> m_beq, m_h1_ineq, m_h2_ineq;

    // P and q of QP problem
    MatXd m_P_cost;

    CVecXd m_q_cost;
};

/**
 * @brief NodeEqCntProfile defines a value constraint at any point of the
 * trajectory, `seg_idx` tells which segment(polynomial) the node lies in, and
 * 'time stamp'(say, t in [0, T]) indicates the node with time stamp t.
 */
struct NodeEqCntProfile
{
    /** @brief segment index of the spline */
    size_t seg_idx;

    /** @brief  derivative order of the constraint
     * 0 for position, 1 for velocity, 2 for acceleration ...
     */
    size_t deriv_order;

    /** @brief time stamp */
    double time_stamp;

    /** @brief constraint value*/
    double cnt_value;
};

/**
 * @brief NodeIneqCntProfile defines a value inequality constraint at any
 * point of the trajectory, `seg_idx` tells which segment(polynomial) the node
 * lies in, and 'time stamp'(say, t in [0, T]) indicates the node with time
 * stamp t.
 */
struct NodeIneqCntProfile
{
    /** @brief segment index */
    size_t seg_idx;

    /** @brief  derivative order of the constraint
     * 0 for position, 1 for velocity, 2 for acceleration ...
     */
    size_t deriv_order;

    /** @brief time stamp*/
    double time_stamp;

    /** @brief constraint lower bound*/
    double lower_bound = -std::numeric_limits<double>::infinity();

    /** @brief constraint upper bound*/
    double upper_bound = std::numeric_limits<double>::infinity();
};

/**
 * @brief ContinousCntProfile defines the continous constraint of the overall
 * trajectory, e.x. position continuity, velocity continuity ...
 */
struct ContinousCntProfile
{
    /** @brief  derivative order of the constraint
     * 0 for position, 1 for velocity, 2 for acceleration ...
     */
    size_t deriv_order;
};

/**
 * @brief SegmentIneqCntProfile defines an inequlity constraint of the specified
 * segment, such that the value of any point in the segment is in the given
 * bound
 */
struct SegmentIneqCntProfile
{
    /** @brief segment index */
    size_t seg_idx;

    /** @brief  derivative order of the constraint
     * 0 for position, 1 for velocity, 2 for acceleration ...
     */
    size_t deriv_order;

    /** @brief constraint lower bound*/
    double lower_bound = -std::numeric_limits<double>::infinity();

    /** @brief constraint upper bound*/
    double upper_bound = std::numeric_limits<double>::infinity();

    /** @brief  number of discrete samples  */
    size_t num_discrete_samples;
};

} // namespace RVS