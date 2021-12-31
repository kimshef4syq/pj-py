#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Copyright (c) RVBUST, Inc - All rights reserved.

from RVBUST.RCI import *

try:
    from .PyRPS import *
except ImportError:
    from RVBUST.RPS.PyRPS import *


def CreateFreespacePlanner(env, manipulator, roadmap_planner=True):
    """Create default freespace motion planner

    Args:
        env (Environment)
        manipulator (Manipulator)
        roadmap_planner (bool, optional): whether use roadmpa planner. Defaults to True.

    Returns:
        RVSPlanner: freespace motion planner
    """
    if roadmap_planner:
        freespace_planner = OmplRoadmapMotionPlanner()

        configuration = freespace_planner.Configuration(env, [
            manipulator])

        configuration.roadmap_construction_time = 10.0

        freespace_planner.SetConfiguration(
            configuration)
    else:
        freespace_planner = OmplParallelPlanMotionPlanner()
        configuration = freespace_planner.Configuration(env, [
            manipulator])

        configuration.hybridize = False
        configuration.planner_types = 4 * [OmplType_RRTConnect]

    configuration.planning_time = 10
    configuration.smooth = True
    configuration.simplify = True
    configuration.collision_check = True
    configuration.collision_distance_threshold = 0
    configuration.longest_valid_segment_length = 0.1

    freespace_planner.SetConfiguration(
        configuration)

    return freespace_planner
