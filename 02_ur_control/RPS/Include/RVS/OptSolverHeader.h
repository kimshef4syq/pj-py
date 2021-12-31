// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once

/**
 *  @defgroup OptSolver OptSolver
 *  @brief This package contains a common interface for optimization solver,
 * includes wrapper for Nlopt and Ipopt
 */

#include "OptSolver/SolverBase.h"
#include "OptSolver/NloptSolver.h"
#include "OptSolver/IpoptAdapter.h"
#include "OptSolver/IpoptSolver.h"
#include "OptSolver/OSQPAdapter.h"