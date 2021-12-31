// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once
/**
 * @defgroup CollisionChecker CollisionChecker
 * @brief This module contains a common interface for collision checking (only
 * FCL collision library is supported now).
 */
#include "CollisionChecker/Types.h"
#include "CollisionChecker/CollisionCheckerBase.h"
#include "CollisionChecker/FCLCollisionChecker/FCLCollisionChecker.h"
#include "CollisionChecker/Utils.h"
#include "CollisionChecker/PropagateDistanceField/VoxelStack.h"
#include "CollisionChecker/DistanceFieldBase.h"