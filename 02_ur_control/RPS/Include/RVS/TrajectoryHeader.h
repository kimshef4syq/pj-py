// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
/**
 *  @defgroup Trajectory Trajectory
 *  @brief This module contains a common inferface for Path and Trajectory (for
 * different Lie group) and several implemetantions of trajectory timer.
 */
#include "Trajectory/PathSegmentBezier2nd.h"
#include "Trajectory/PathSegmentCircleCircumscribed.h"
#include "Trajectory/PathSegmentCircleInscribed.h"
#include "Trajectory/PathSegmentCustomed.h"

#include "Trajectory/PathBezier2ndBlend.h"
#include "Trajectory/PathCircleBlend.h"

#include "Trajectory/TrajectoryConstantSpeed.h"
#include "Trajectory/TrajectoryTrapezoidal.h"
#include "Trajectory/TrajectoryToppra.h"
#include "Trajectory/TrajectoryTotp.h"
#include "Trajectory/TrajectoryTotp3.h"

#include "Trajectory/Polynomial.h"
#include "Trajectory/PolynomialRotation.h"
#include "Trajectory/Splines.h"
#include "Trajectory/SplinesRotation.h"
#include "Trajectory/SplineComposition.h"

#include "Trajectory/TrajectorySplineBase.h"
#include "Trajectory/TrajectorySpline.h"
#include "Trajectory/TrajectorySplineEx.h"
#include "Trajectory/TrajectoryRnP4.h"
#include "Trajectory/TrajectoryISP.h"

#include "Trajectory/OnlineTrajectory.h"
#include "Trajectory/OnlineTrajectoryRn.h"

#include "Trajectory/TrajectoryUtilsEx.h"
#include "Trajectory/TrajProfileConstraintsEx.h"

#include "Trajectory/Waypoints.h"
#include "Trajectory/TrajectoryReachability.h"
#include "Trajectory/P2PTrajectory.h"
#include "Trajectory/BlendedTrajectory.h"
#include "Trajectory/ConstraintTransportTrajectory.h"
