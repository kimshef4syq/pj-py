#pragma once
#include <RVS/Planner/CartesianPlanner/TrajectoryGeneration/TrajQPSolverBase.h>
#include <RVS/Planner/CartesianPlanner/Utils.h>

namespace RVS
{
class BezierQP : public TrajQPSolver
{
public:
    explicit BezierQP(const CVecXd &ts, size_t poly_order)
        : TrajQPSolver(ts, poly_order)
    {
    }

    /** @brief set the quadratic cost matrix of the QP problem*/
    virtual bool SetQuadCostMat(size_t deriv_order) override;

    /** @brief add a node equlity constraint to the QP problem*/
    virtual bool AddNodeEqCnt(const NodeEqCntProfile node_eq_cnt) override;

    /** @brief add a node inequlity constraint to the QP problem*/
    virtual bool
    AddNodeIneqCnt(const NodeIneqCntProfile node_ineq_cnt) override;

    /** @brief add a continous constraint to the QP problem*/
    virtual bool
    AddContinousCnt(const ContinousCntProfile continous_cnt) override;

    /** @brief add a segment inequlity constraint to the QP problem*/
    virtual bool
    AddSegmentIneqCnt(const SegmentIneqCntProfile seg_ineq_cnt) override;

    /** @brief construct and solve QP problem */
    virtual bool Solve(CVecXd &poly_coeffs, bool verbose) override;

protected:
    MatXd GetPolyQuadCostMat(size_t deriv_order, double T);
};
} // namespace RVS