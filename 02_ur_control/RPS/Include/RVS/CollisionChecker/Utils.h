// Copyright (c) RVBUST, Inc - All rights reserved.
#include <RVS/Environment/Manipulator.h>
#include <RVS/Trajectory/PathBase.h>
#include <RVS/CollisionChecker/CollisionCheckerBase.h>

namespace RVS
{

/**
 * @brief Auto-compute self collision matrix with randomly check several states
 *
 * The Default Self-Collision Matrix Generator will search for pairs of links on
 * the robot that can safely be disabled from collision checking, decreasing
 * motion planning processing time. These pairs of links are disabled when they
 * are always in collision, never in collision, in collision in the robot's
 * default position or when the links are adjacent to each other on the
 * kinematic chain. Sampling density specifies how many random robot positions
 * to check for self collision. Higher densities require more computation time.
 *
 * @param body: multibody, robot_model or endeffector
 * @param sample_density: number of random states, default value is 1000
 * @param min_collision_faction: min collision fraction for always-in-collision
 * check, default value is 95%
 * @return true
 * @return false
 */
bool ComputeSelfCollisionMatrix(std::shared_ptr<Multibody> body,
                                const int sample_density = 1000,
                                const double min_collision_faction = 0.95);
} // namespace RVS