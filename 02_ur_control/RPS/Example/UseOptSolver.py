'''!@example UseOptSolver.py
@brief Using OptSovler to solve the following problem:
\f{eqnarray*}{
min.\quad &3x_0^2 - x_1^2 - 2x_2^2 \\
st.\quad &x_0 - x_1 \ge 0 \\
    &x_0^2 + x_1^2 + x_2^2 = 3 \\
    &x_0, x_1, x_2 \ge 0
\f}
@date 2021-08-19
@copyright Copyright (c) RVBUST, Inc - All rights reserved.
'''

import numpy as np
from IPython import embed
from RVBUST.RPS import *


class Prob:
    def __init__(self):
        """ Some data could be stored here to compute Cost/Constraint
        """
        pass

    def GetCost(self, x: np.ndarray, var_idx: np.ndarray = np.empty(0)):
        """Get NLP problem cost

        Args:
            x (np.ndarray): input problem variables, immutable
            var_idx (np.ndarray): linked variable indexes, immutable 

        Returns:
            float: cost value
        """
        return -3.0 * x[0] * x[0] - x[1] * x[1] - 2. * x[2] * x[2]

    def GetCostJac(self, x: np.ndarray, jac: np.ndarray, accumulation: bool = False, var_idx: np.ndarray = np.empty(0)):
        """ Get NLP problem cost jacobian

        Args:
            x (np.ndarray): input problem variables, immutable 
            jac (np.ndarray): output jacboian, 1D array, need assign value in place
            accumulation (bool, optional): [description]. Defaults to False.
            var_idx (np.ndarray): linked variable indexes, immutable 

        Returns:
            bool: [description]
        """
        if not accumulation:
            jac[:] = 0
        jac[0] += -6. * x[0]
        jac[1] += - 2. * x[1]
        jac[2] += - 4. * x[2]
        return True

    def GetConstraintValue(self, x: np.ndarray, c: np.ndarray, var_idx: np.ndarray = np.empty(0)) -> bool:
        """ Get NLP problem constraint values 

        Args:
            x (np.ndarray): input problem variables, immutable 
            c (np.ndarray): output constraint values, 1D array, need assign value in place
            var_idx (np.ndarray): linked variable indexes, immutable 

        Returns:
            bool: [description]
        """
        c[0] = x[0] - x[1]
        c[1] = np.square(x[var_idx]).sum()
        return True

    def GetConstraintJac(self, x: np.ndarray, jac: np.ndarray, var_idx: np.ndarray = np.empty(0)) -> bool:
        """Get NLP problem constraint jacobian

        Args:
            x (np.ndarray): input problem variables, immutable 
            jac (np.ndarray): output constraint jacobian values, 1D array, need assign value in place
            var_idx (np.ndarray): linked variable indexes, immutable 

        Returns:
            bool: [description]
        """

        """
        you can reshape the jacobian from 1d array to m x n matrix in place for better understanding, 
        m is number of constraints, n is number of variables.
        Don't use jac = jac.reshape(2, 3), that will probably release the original memory.
        """
        jac.shape = (-1, x.shape[0])
        jac[0, :] = 1.0, -1.0, 0.0
        jac[1, :] = 2.0 * x[0], 2.0 * x[1], 2.0 * x[2]

        # or as following:
        # jac[0] = 1.0
        # jac[1] = -1.0
        # jac[2] = 0.0
        # jac[3] = 2.0 * x[0]
        # jac[4] = 2.0 * x[1]
        # jac[5] = 2.0 * x[2]
        return True


prob = Prob()
model = OptModel()
var_set = VariableSet(3, [Bounds(0, 1e10), Bounds(0, 1e10), Bounds(0, 1e10)])
model.SetVariableSet(var_set)

cost = ExCost(prob.GetCost, prob.GetCostJac)  # using analytical jacobian
# pass None to use numberical cost jacobian if it's diffcult to get analytical one
# cost = ExCost(prob.GetCost, None, "cost", list(range(6)))
model.AddCostTerm(cost)

constraint = ExConstraint(2, [Bounds(-1e10, 0), Bounds(3, 3)],
                          prob.GetConstraintValue, prob.GetConstraintJac, "constraint", list(range(3)))
# you can use numerical jacobian of constraint, too
# constraint = ExConstraint(2, [Bounds(-1e10, 0), Bounds(3, 3)],
#                           prob.GetConstraintValue, None, "constraint",list(range(3)))
model.AddConstraintSet(constraint)

np.random.seed(1)
x0 = np.random.random(3)

solver = NloptSolver()
solver.Initialize(x0, NloptAlgorithm.LD_SLSQP)
ret = solver.Solve(model)
x = model.GetVariableValue()
print(
    f"#### NloptSovler Result ###\n\nsolver result: {ret}, opt vars: {x}, opt cost: {model.GetCostValue(x)}")

solver = IpoptSolver()
solver.Initialize(x0)
solver.SetOption("jacobian_approximation", "finite-difference-values") # or "exact"
ret = solver.Solve(model)
x = model.GetVariableValue()
print(
    f"#### IpoptSovler Result ###\n\nsolver result: {ret}, opt vars: {x}, opt cost: {model.GetCostValue(x)}")