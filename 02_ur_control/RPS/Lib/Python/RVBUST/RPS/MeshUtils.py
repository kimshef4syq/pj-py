#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Copyright (c) RVBUST, Inc - All rights reserved.

from RVBUST.RCI import *

from .PyRPS import *


def MeshSimplifier(visual_mesh, max_convex_hulls=5, padding_distance=0.005, show_in_vis=True, vis=None):
    hs = []

    # Convert visual_mesh into convex meshes
    v2 = VHACDParameters()
    v2.max_convex_hulls = max_convex_hulls

    meshes = visual_mesh.ConvexDecomposition(v2)

    if show_in_vis:
        rvis = RobotVis(vis)
        for m in meshes:
            hs.append(rvis.PlotGeometry(m, pose=Pose(
                [0, -0.5, 0, 0, 0, 0, 1]), color=np.random.random(3)))

    # Merge meshes
    meshall = MergeMesh(meshes)
    if show_in_vis:
        hs.append(rvis.PlotGeometry(meshall, pose=Pose(
            [0, -1, 0, 0, 0, 0, 1]), color=np.random.random(3)))

    # Padding mesh
    meshallpad = meshall.Copy()
    meshallpad.PadMesh(padding_distance)
    if show_in_vis:
        hs.append(rvis.PlotGeometry(meshallpad, pose=Pose(
            [0, -1.5, 0, 0, 0, 0, 1]), color=np.random.random(3)))

    meshallpad = meshallpad.Copy()

    return hs, meshallpad
