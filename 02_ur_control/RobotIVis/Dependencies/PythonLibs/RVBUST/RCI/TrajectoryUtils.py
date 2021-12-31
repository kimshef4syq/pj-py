#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Created on Fri Nov 09 2018
# Copyright (c) 2018 RVBUST, INC. - All rights reserved.

import math
import os
import time
from collections.abc import Iterable

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from RVBUST import Vis

try:
    from .PyRCI import *
    from .RichLogger import RichLogger
except ImportError:
    from RVBUST.RCI.PyRCI import *
    from RVBUST.RCI.RichLogger import RichLogger

pi = np.pi
logger = RichLogger("TrajectoryUtils")


def PlotSplineRot(spl, dq_max=None, ddq_max=None, title="Spline", figsize=None):
    ts = np.linspace(0, spl.GetKnotLength(), 1000)
    pos = Rotation.from_quat(
        np.array([spl.Eval(t, 0) for t in ts])).as_rotvec()
    vel = np.array([spl.Eval(t, 1) for t in ts])
    acc = np.array([spl.Eval(t, 2) for t in ts])
    fig = plt.figure(figsize=figsize)
    fig.tight_layout()
    plt.subplots_adjust(hspace=0.6)
    rows = 3
    cols = 1
    xs = ts
    ax = plt.subplot2grid((rows, cols), (0, 0), fig=fig)
    ax.plot(xs, pos)
    ax.legend(["D%s" % i for i in range(pos.shape[1])])
    ax.set_title("rotation vector")

    ax = plt.subplot2grid((rows, cols), (1, 0), fig=fig)
    ax.set_title("angular rate")
    ax.plot(xs, vel)
    if dq_max is not None:
        ax.plot(xs[[0, -1]], np.tile(dq_max, (2, 1)), "--")
        ax.plot(xs[[0, -1]], np.tile(-dq_max, (2, 1)), "--")
    ax.legend(["D%s" % i for i in range(vel.shape[1])])

    ax = plt.subplot2grid((rows, cols), (2, 0), fig=fig)
    ax.set_title("angular acceleration")
    ax.plot(xs, acc)
    if ddq_max is not None:
        ax.plot(xs[[0, -1]], np.tile(ddq_max, (2, 1)), "--")
        ax.plot(xs[[0, -1]], np.tile(-ddq_max, (2, 1)), "--")
    ax.legend(["D%s" % i for i in range(acc.shape[1])])

    plt.suptitle(title)
    plt.show()


def PlotSpline(spl, dq_max=None, ddq_max=None, dddq_max=None, waypoints=None, waypoints_ts=None, title="Spline", figsize=None):
    ts = np.linspace(0, spl.GetKnotLength(), 1000)
    pos = np.array([spl.Eval(t, 0) for t in ts])
    vel = np.array([spl.Eval(t, 1) for t in ts])
    acc = np.array([spl.Eval(t, 2) for t in ts])
    jerk = np.array([spl.Eval(t, 3) for t in ts])
    fig = plt.figure(figsize=figsize)
    fig.tight_layout()
    plt.subplots_adjust(hspace=0.6)
    rows = 3  # rows of sub-plots
    cols = 2  # cols of sub-plots
    DIM = 1  # DoF
    if pos.ndim == 2:
        DIM = pos.shape[1]
    xs = ts
    ax = plt.subplot2grid((rows, cols), (0, 0), fig=fig)
    ax.plot(xs, pos, "-")
    if waypoints_ts is not None and waypoints is not None:
        ax.plot(waypoints_ts, waypoints, "r^")
    ax.legend(["D%s" % i for i in range(DIM)])
    ax.set_title("pos")

    ax = plt.subplot2grid((rows, cols), (1, 0), fig=fig)
    ax.set_title("vel")
    ax.plot(xs, vel)
    if dq_max is not None:
        ax.plot(xs[[0, -1]], np.tile(dq_max, (2, 1)), "--")
        ax.plot(xs[[0, -1]], np.tile(-dq_max, (2, 1)), "--")
    ax.legend(["D%s" % i for i in range(DIM)])

    ax = plt.subplot2grid((rows, cols), (2, 0), fig=fig)
    ax.set_title("acc")
    ax.plot(xs, acc)
    if ddq_max is not None:
        ax.plot(xs[[0, -1]], np.tile(ddq_max, (2, 1)), "--")
        ax.plot(xs[[0, -1]], np.tile(-ddq_max, (2, 1)), "--")
    ax.legend(["D%s" % i for i in range(DIM)])

    ax = plt.subplot2grid((rows, cols), (0, 1), fig=fig)
    ax.plot(xs, jerk)
    if dddq_max is not None:
        ax.plot(xs[[0, -1]], np.tile(dddq_max, (2, 1)), "--")
        ax.plot(xs[[0, -1]], np.tile(-dddq_max, (2, 1)), "--")
    ax.legend(["D%s" % i for i in range(DIM)])
    ax.set_title("jerk")

    if pos.ndim == 2:
        if pos.shape[1] == 2:
            ax = plt.subplot2grid((rows, cols), (1, 1), rowspan=2, fig=fig)
            ax.plot(*pos[:, :2].T, "g-", label="path(only first 2 dof)")
            if waypoints is not None:
                ax.plot(*waypoints[:, :2].T, "r*",
                        label="waypoints(only first 2 dof)")
            ax.legend()
            ax.axis("scaled")
            ax.set_title("path")
        elif pos.shape[1] > 2:
            ax = plt.subplot2grid((rows, cols), (1, 1), rowspan=2,
                                  fig=fig, projection="3d")
            ax.plot(*pos[:, :3].T, "g-", label="path(only first 3 dof)")
            if waypoints is not None:
                ax.plot(*waypoints[:, :3].T, "r*",
                        label="waypoints(only first 3 dof)")
            ax.set_title("path")
            ax.legend()
        else:
            pass
    plt.suptitle(title)
    plt.show()


def PlotTraj(traj, dq_max=None, ddq_max=None, dddq_max=None, waypoints=None, sample_num: int = 200, title: str = "Trajectory"):
    """Plot trajectory position/velocity/accleration/jerk and path, and vel/acc/jerk limits

    Args:
        traj ([type]): [description]
        dq_max ([type], optional): [description]. Defaults to None.
        ddq_max ([type], optional): [description]. Defaults to None.
        dddq_max ([type], optional): [description]. Defaults to None.
        waypoints ([type], optional): [description]. Defaults to None.
        sample_num (int, optional): [description]. Defaults to 1000.
        title (str, optional): [description]. Defaults to "Trajectory".
    """
    if not traj.IsValid():
        logger.error("{} is invalid".format(traj))
        return
    if waypoints is not None:
        if not isinstance(waypoints[0], Iterable):
            waypoints = np.array([p.Coeffs() for p in waypoints])
        else:
            waypoints = np.asarray(waypoints)
    ts = np.linspace(0, traj.GetDuration(), sample_num)
    pos = np.array([traj.GetPosition(t).Coeffs() for t in ts])
    vel = np.array([traj.GetVelocity(t).Coeffs() for t in ts])
    acc = np.array([traj.GetAcceleration(t).Coeffs() for t in ts])
    jerk = np.array([traj.GetJerk(t).Coeffs() for t in ts])
    xs = ts
    fig = plt.figure(figsize=(16, 9))
    fig.tight_layout()
    plt.subplots_adjust(hspace=0.6)

    if isinstance(traj, TrajectoryBaseSE3):
        rotvecs = np.asarray([Rotation(p[3:]).Log().Coeffs() for p in pos])
        _pos = np.zeros((pos.shape[0], 6))
        _pos[:, :3] = pos[:, :3]
        _pos[:, 3:] = rotvecs
        pos = _pos
    ax = plt.subplot2grid((4, 2), (0, 0), fig=fig)
    ax.plot(xs, pos)
    ax.legend(["D%s" % i for i in range(traj.GetDoF())])
    ax.set_title("pos")

    ax = plt.subplot2grid((4, 2), (1, 0), fig=fig)
    ax.set_title("vel")
    ax.plot(xs, vel)
    if dq_max is not None:
        ax.plot(xs[[0, -1]], np.tile(dq_max, (2, 1)), "--")
        ax.plot(xs[[0, -1]], np.tile(-dq_max, (2, 1)), "--")
    ax.legend(["D%s" % i for i in range(traj.GetDoF())])

    ax = plt.subplot2grid((4, 2), (2, 0), fig=fig)
    ax.set_title("acc")
    ax.plot(xs, acc)
    if ddq_max is not None:
        ax.plot(xs[[0, -1]], np.tile(ddq_max, (2, 1)), "--")
        ax.plot(xs[[0, -1]], np.tile(-ddq_max, (2, 1)), "--")
    ax.legend(["D%s" % i for i in range(traj.GetDoF())])

    ax = plt.subplot2grid((4, 2), (3, 0), fig=fig)
    ax.plot(xs, jerk)
    if dddq_max is not None:
        ax.plot(xs[[0, -1]], np.tile(dddq_max, (2, 1)), "--")
        ax.plot(xs[[0, -1]], np.tile(-dddq_max, (2, 1)), "--")
    ax.legend(["D%s" % i for i in range(traj.GetDoF())])
    ax.set_title("jerk")

    if pos.shape[1] == 2:
        ax = plt.subplot2grid((4, 2), (0, 1), rowspan=4, fig=fig)
        ax.plot(*pos[:, :2].T, "g-", label="path(only first 2 dof)")
        if waypoints is not None:
            ax.plot(*waypoints[:, :2].T, "r*",
                    label="waypoints(only first 2 dof)")
        ax.legend()
        ax.axis("scaled")
        ax.set_title("path")
    elif pos.shape[1] > 2:
        ax = plt.subplot2grid((4, 2), (0, 1), rowspan=4,
                              fig=fig, projection="3d")
        ax.plot(*pos[:, :3].T, "g-", label="path(only first 3 dof)")
        if waypoints is not None:
            ax.plot(*waypoints[:, :3].T, "r*",
                    label="waypoints(only first 3 dof)")
        ax.set_title("path")
        ax.legend()
    else:
        pass
    plt.suptitle(title)
    plt.show()


def Arrow(ax, Ts, length=4, display="xyzo", **kargs):
    """Draw an arrow in 3d space
    Arguments:
        ax {matplotlib axes} -- fig = plt.figure(), ax = fig.add_subplot(projection='3d')
        Ts {transform} -- a transform (4x4) or an array of transforms (Nx4x4)
    Keyword Arguments:
        length {int} -- the length of the arrow (default: {4})
        display {str} -- draw which part of the arrow (default: {"xyzo"})
        **kargs:
        origin_color: "red", etc, the color of the origin point
    """
    draw_code = ["ax.quiver(*T[:3, 3],*T[:3, 0], length=length, normalize=True, color=(1, 0, 0, 0.5))",
                 "ax.quiver(*T[:3, 3], *T[:3, 1], length=length, normalize=True, color=(0, 1, 0, 0.5))",
                 "ax.quiver(*T[:3, 3], *T[:3, 2], length=length, normalize=True, color=(0, 0, 1, 0.5))",
                 "ax.plot3D(*T[:3, 3].reshape(3, 1), 'o-', markersize=2, color=kargs.get('origin_color', 'red'))"]

    if len(Ts.shape) == 3:
        for T in Ts:
            if display == "xyzo":
                exec("\n".join(draw_code))
                continue
            if 'x' in display.lower():
                exec(draw_code[0])
            if 'y' in display.lower():
                exec(draw_code[1])
            if 'z' in display.lower():
                exec(draw_code[2])
            if 'o' in display.lower():
                exec(draw_code[3])

    elif len(Ts.shape) == 2:
        T = Ts
        exec("".join(draw_code))


def DrawTraj(traj, show_in_vis: bool = False, show_in_plt: bool = False, view=None,
             kin_solver: KinematicsBase = None, points_number: int = 0,
             clear_view: bool = False, show_waypoints: bool = True,
             axis_length: float = 0.02, robot_base_transform: Pose = Pose(),
             display_mode: str = 'o', title="Trajectory", save_path: str = None, **kwargs):
    """Show trajectory in vis or matplot

    :param traj: trajectory that need to be shown
    :type traj: [type]
    :param show_in_vis: show trajectory in Vis, defaults to False
    :type show_in_vis: bool, optional
    :param show_in_plt: [description], defaults to False
    :type show_in_plt: bool, optional
    :param view: [description], defaults to None
    :type view: [type], optional
    :param kin_solver: if want to show R6 trajectory in vis, kin_solver is needed to calculate FK, defaults to None
    :type kin_solver: [type], optional
    :param points_number: sampled points number, defaults to 0
    :type points_number: int, optional
    :param clear_view: clear the handles in view created by this function, defaults to False
    :type clear_view: bool, optional
    :param show_waypoints: [description], defaults to True
    :type show_waypoints: bool, optional
    :param axis_length: [description], defaults to 0.02
    :type axis_length: float, optional
    :param robot_base_transform: [description], defaults to Pose()
    :type robot_base_transform: [type], optional
    :param display_mode: 'xyz' or 'o' or 'xyzo', draw axes or only orgins, defaults to 'o'
    :type display_mode: str, optional
    :param save_path: file name ends with [.png, .jpg, .svg, .pdf, .ps], if show_in_plt is true, and you want to save the figure, giving the saved file path (directory will be created automatically if possible)
    :type save_path: str, optional
    :return: [description]
    :rtype: [type]
    """
    try:
        # user may be pass traj as other type
        if not traj.IsValid() or traj.GetDuration() == 0:
            logger.error("Error: traj is not valid, or traj duration is 0")
            return []
    except Exception as e:
        logger.error("Got Exception : ", e)
        return []

    if hasattr(view, "GetView"):
        view = view.GetView()

    creat_view = view is None

    waypoints = None
    try:
        groupname = kwargs.get("groupname", "")
        if not groupname:
            groupname = traj.GetLieGroupName()
        ret, waypoints, _ = traj.GetWaypointAndTimeStamps()
        if ret == RVSReturn_Success:
            waypoints = np.array([p.Coeffs() for p in waypoints])
        else:
            waypoints = None
    except Exception as e:
        logger.error("Got Exception when execing traj.GetPath: ", e)

    points = traj.GetDuration() * 200
    if points > 1000:
        points = 1000
    points = points_number or points
    t_set = np.linspace(0, traj.GetDuration(), int(points))
    pos = np.array(list(map(lambda t: traj.GetPosition(t).Coeffs(), t_set)))
    velocities = np.array(
        list(map(lambda t: traj.GetVelocity(t).Coeffs(), t_set)))
    accelerations = np.array(
        list(map(lambda t: traj.GetAcceleration(t).Coeffs(), t_set)))

    if view is not None:
        show_in_vis = True
    else:
        show_in_plt = True

    view_handles = []

    if groupname in ("Pose", "SE3"):
        if show_in_vis:
            if view is None:
                view = Vis.View(groupname)
                view_handles.append(
                    view.Axes(np.identity(4).flatten().tolist(), 0.2, 0.2))
            if waypoints is not None and show_waypoints:
                hs = view.Axes(waypoints[:, :3],
                               waypoints[:, 3:], axis_length, 3)
                view_handles.extend(hs)
            hs = view.Axes(pos[:, :3], pos[:, 3:], axis_length, 1)
            view_handles.extend(hs)
            points = np.c_[pos[:-1, :3], pos[1:, :3]]
            h = view.Line(points.ravel(), 0.8, [0, 0, 0])
            view_handles.append(h)
        if show_in_plt:
            fig = plt.figure(figsize=(15, 5))
            fig.suptitle(title)
            fig.tight_layout()
            plt.subplots_adjust(hspace=0.4)
            ax2d1 = fig.add_subplot(2, 3, 1)
            for i in range(3):
                ax2d1.plot(t_set, pos[:, i], "o-",
                           label="%sth dim value" % (i+1), markersize=2)
            ax2d1.legend()
            ax2d1.set_title("Position vector")

            ax2d2 = fig.add_subplot(2, 3, 2)
            rot_vec = np.array(
                list(map(lambda quat: Rotation(quat).Log().Coeffs(), pos[:, 3:])))
            for i in range(rot_vec.shape[1]):
                ax2d2.plot(t_set, rot_vec[:, i], 'o-',
                           label="%sth dim value" % (i+1), markersize=2)
            ax2d2.legend()
            ax2d2.set_title("Rotation Vector")

            ax2d3 = fig.add_subplot(2, 3, 3)
            for i in range(3):
                ax2d3.plot(t_set, velocities[:, i], "o-",
                           label="%sth dim value vel" % (i+1), markersize=2)
            ax2d3.plot(t_set, np.linalg.norm(velocities[:, :3], axis=1),
                       "r.-", label="Composed cartesian velocity", ms=2, lw=1)
            ax2d3.legend()
            ax2d3.set_title("Linear velocity")

            ax2d4 = fig.add_subplot(2, 3, 4)
            for i in range(3, velocities.shape[1]):
                ax2d4.plot(t_set, velocities[:, i], 'o-',
                           label="%sth dim value vel" % (i+1), markersize=2)
            ax2d4.legend()
            ax2d4.set_title("Angle velocity")

            ax2d5 = fig.add_subplot(2, 3, 5)
            for i in range(accelerations.shape[1]):
                ax2d5.plot(t_set, accelerations[:, i], 'o-',
                           label="%sth dim value acc" % (i+1), markersize=2)
            ax2d5.legend()
            ax2d5.set_title("Accelerations")

            pos_T = np.array(
                list(map(lambda pose: Pose(pose).Transform(), pos)))
            ax3d = fig.add_subplot(2, 3, 6, projection='3d')
            arrow_length = 0.3
            if waypoints is not None and show_waypoints:
                lim = (waypoints[:, :3].min(), waypoints[:, :3].max())
                arrow_length = (lim[1] - lim[0]) / 10
                if lim[0] == lim[1]:
                    lim = (-0.2 + lim[0], 0.2+lim[0])
                    arrow_length = (lim[1] - lim[0]) / 2
                ax3d.plot(*waypoints[:, :3].T, 'g*--', label="way points")
                pos_waypoints = np.array(
                    list(map(lambda pose: Pose(pose).Transform(), waypoints)))
                Arrow(ax3d, pos_waypoints, length=arrow_length *
                      2, origin_color="black")
                ax3d.set_xlim(lim)
                ax3d.set_ylim(lim)
                ax3d.set_zlim(lim)
            Arrow(ax3d, pos_T, length=arrow_length, display=display_mode)
            ax3d.legend()
            ax3d.set_title("Trajectory Path")

    elif groupname == "SO3":
        if show_in_vis:
            if view is None:
                view = Vis.View(groupname)
                view_handles.append(
                    view.Axes(np.identity(4).flatten().tolist(), axis_length, 0.2))
            hs = view.Axes(np.zeros((pos.shape[0], 3)), pos, axis_length, 0.1)
            view_handles.extend(hs)

        fig = plt.figure(figsize=(16, 8))
        fig.suptitle(title)
        fig.tight_layout()
        plt.subplots_adjust(hspace=0.4)
        ax2d1 = fig.add_subplot(2, 2, 1)
        ang_vec = np.array(list(map(Quat2Rotvec, pos)))
        for i in range(3):
            ax2d1.plot(t_set, ang_vec[:, i], "o-",
                       label="%sth dim value" % (i+1), markersize=2)
        ax2d1.legend()
        ax2d1.set_title("Position")

        ax2d2 = fig.add_subplot(2, 2, 2)
        for i in range(velocities.shape[1]):
            ax2d2.plot(t_set, velocities[:, i], 'o-',
                       label="%sth dim value" % (i+1), markersize=2)
        ax2d2.legend()
        ax2d2.set_title("Velocity")

        ax2d3 = fig.add_subplot(2, 2, 3)
        for i in range(3):
            ax2d3.plot(t_set, accelerations[:, i], "o-",
                       label="%sth dim value vel" % (i+1), markersize=2)
        ax2d3.legend()
        ax2d3.set_title("Accelerations")

        ax3d = fig.add_subplot(2, 2, 4, projection="3d")
        Ts = np.array(list(map(lambda x: np.r_[
                      np.c_[Quat2Mat(x), np.zeros(3).reshape(3, 1)], np.array([0, 0, 0, 1]).reshape(1, 4)], pos)))
        Arrow(ax3d, Ts, length=1)
        lim = (-1, 1)
        ax3d.set_xlim(lim)
        ax3d.set_ylim(lim)
        ax3d.set_zlim(lim)
        ax3d.legend()
        ax3d.set_title("Trajectory Path")

    elif groupname == "SO2":
        fig = plt.figure(figsize=(16, 16))
        fig.suptitle(title)
        ax2d1 = fig.add_subplot(1, 3, 1)
        ax2d2 = fig.add_subplot(1, 3, 2)
        # ax2d3 = fig.add_subplot(2, 2, 3)
        ax3d = fig.add_subplot(1, 3, 3)
        dim = 2
        show_dims = dim
        for i in range(dim):
            ax2d1.plot(t_set, pos[:, i], "o-",
                       label="%sth dim value" % (i+1), markersize=2)
        ax2d1.legend()
        ax2d1.set_title("Angle vector")

        angles = np.arctan2(pos[:, 1], pos[:, 0])
        # ang_vel = np.arctan2(tangents[:, 1], tangents[:, 0])
        # ang_acc = np.arctan2(curvatures[:, 1], curvatures[:, 0])
        ang_vel = velocities
        ang_acc = accelerations
        ax2d2.plot(t_set, angles, label="Angle")
        ax2d2.plot(t_set, ang_vel, label="Angle velocity")
        ax2d2.plot(t_set, ang_acc, label="Angle acceleration")
        ax2d2.legend()
        ax2d2.set_title("Angle")

        ax3d.quiver(0, 0, *pos.T, scale=3, color="red")
        ax3d.set_title("Trajectory Path")
    elif groupname == "SE2" or ('SO2' in groupname and "R2" in groupname):
        fig = plt.figure(figsize=(18, 6))
        fig.suptitle(title)
        ax2d1 = fig.add_subplot(1, 3, 1)
        ax2d2 = fig.add_subplot(1, 3, 2)
        ax2d3 = fig.add_subplot(1, 3, 3)

        ax2d1.plot(*pos[:, :2].T, "o-", color="purple", ms=2)
        ax2d1.quiver(*pos[:, :2].T, *pos[:, 2:].T, color="red", scale=10)
        if waypoints is not None:
            ax2d1.plot(*waypoints[:, :2].T, "o--", color="blue", ms=4)
            ax2d1.quiver(*waypoints[:, :2].T, *(waypoints[:, 2:] / np.linalg.norm(
                waypoints[:, 2:], axis=1).reshape(-1, 1)).T, color="green", scale=10)
        ax2d1.set_title("Pose")

        ax2d2.plot(t_set, velocities, "o-", ms=2, lw=1)
        ax2d2.legend(["Dim 1", "Dim 2", "Dim 3"])
        ax2d2.set_title("Velocity")

        ax2d3.plot(t_set, accelerations, "o-", ms=2, lw=1)
        ax2d3.legend(["Dim"+str(i+1) for i in range(3)])
        ax2d3.set_title("Accelerations")

    elif isinstance(traj, TrajectoryBaseRn):
        if show_in_vis and traj.GetDoF() == kin_solver.GetDoF():
            if view is None:
                view = Vis.View(groupname)
                view_handles.append(
                    view.Axes(np.identity(4).flatten().tolist(), axis_length, 0.2))
            if waypoints is not None and show_waypoints:
                _poses = []
                for waypoint in waypoints:
                    _pose = (robot_base_transform *
                             kin_solver.GetPositionFK(Rx(waypoint))[1]).Coeffs()
                    _poses.append(_pose)
                _poses = np.asarray(_poses)
                view_handles.extend(
                    view.Axes(_poses[:, :3], _poses[:, 3:], axis_length, 3))
            _poses = []
            for i in range(pos.shape[0]):
                _pose = (robot_base_transform *
                         kin_solver.GetPositionFK(Rx(pos[i]))[1]).Coeffs()
                _poses.append(_pose)
            _poses = np.asarray(_poses)
            view_handles.extend(
                view.Axes(_poses[:, :3], _poses[:, 3:], axis_length, 1))
            lines = np.c_[_poses[:-1, :3], _poses[1:, :3]].ravel()
            view_handles.append(
                view.Line(lines, 0.8, [0, 0, 0]))
        if 'R' == groupname[0]:
            dims = int(groupname[1:])
        if show_in_plt:
            fig = plt.figure(figsize=(16, 16))
            fig.suptitle(title)
            ax2d1 = fig.add_subplot(2, 2, 1)
            ax2d2 = fig.add_subplot(2, 2, 2)
            ax2d3 = fig.add_subplot(2, 2, 3)
            dim = int(groupname[1:])
            show_dims = dim
            if dim >= 3:
                ax3d = fig.add_subplot(2, 2, 4, projection="3d")
                show_dims = 3
            elif dim == 2:
                ax3d = fig.add_subplot(2, 2, 4)

            for i in range(dim):
                ax2d1.plot(t_set, pos[:, i], "o-",
                           label="%sth dim value" % (i+1), markersize=2)
            ax2d1.legend()
            ax2d1.set_title("Position")

            for i in range(dim):
                ax2d2.plot(t_set, velocities[:, i], "o-",
                           label="%sth dim value" % (i+1), markersize=2)
            ax2d2.plot(t_set, np.linalg.norm(velocities, axis=1),
                       "r.-", label="Composed velocity", lw=1, ms=2)
            ax2d2.legend()
            ax2d2.set_title("Velocity")

            for i in range(dim):
                ax2d3.plot(t_set, accelerations[:, i], "o-",
                           label="%sth dim value" % (i+1), markersize=2)
            ax2d3.legend()
            ax2d3.set_title("Acceleration")

            if dim >= 2:
                ax3d.plot(*pos[:, :show_dims].T, "ro-",
                          markersize=2, label="traj")
                if waypoints is not None and len(waypoints):
                    ax3d.plot(*waypoints[:, :show_dims].T,
                              'g*--', label="Way points")
                ax3d.legend()
                ax3d.set_title("Trajectory Path")
    else:
        logger.error("Group name is [%s], which is not supported" % groupname)
        return []

    if show_in_plt:
        if save_path is not None:
            os.makedirs(os.path.dirname(save_path), exist_ok=True)
            plt.savefig(save_path)
        plt.show()
        plt.clf()
        plt.close()

    view_handles.reverse()
    clear_view and view_handles and view.Delete(view_handles)

    if creat_view and view is not None:
        view.Clear()
        time.sleep(0.05)
        view.Close()
    return view_handles


def ShowTraj(traj, robot_vis, manipulator, show_mode=1, loop_times=1, robot_alpha=0.5, show_trail=False, kin_solver=None,
             trail_step=1, draw_traj=True, show_in_plt=False, clear_bodies=True, clear_handles=False, show_waypoints=True, **kwargs):
    """ Display a trajectory in vis and matplot
    :param traj: [Trajectory]
    :param robot_vis: [RobotVis]
    :param manipulator: [Manipulator]
    :param show_mode: [description], defaults to 1
    :param loop_times: [run trajectory how many times], defaults to 1
    :param robot_alpha: [robot model transparency], defaults to 0.5
    :param show_trail: [interpolate robot model within trajectory], defaults to True
    :param trail_step: [the step size that the robot models to be interpolated along trajectory], defaults to 1
    :param draw_traj: [Draw axes in vis], defaults to True
    :param show_in_plt: [Plot curves of position/velocity/acceleration in matplot], defaults to False
    :param clear_bodies: [clear robot models when exit], defaults to True
    :param clear_handles: [clear axes when exit], defaults to False
    :return: [robot models and axes handles]
    """
    if not traj.IsValid():
        logger.error("Trajectory is not valid")
        return [], []
    robot_models = []
    manis = []
    handles = []
    duration = traj.GetDuration()
    path = traj.GetPath()
    path_len = path.GetLength()
    number = math.ceil(max(path_len // 0.3, 3) // trail_step)

    original_robot_model = manipulator.GetRobotModel()
    origin_name = original_robot_model.GetName()
    robot_model = original_robot_model.Copy()
    name_idx = 1
    robot_model.SetName(origin_name + "_traj_show_copy_%i" % name_idx)
    name_idx += 1
    mani = robot_model.GetManipulator(manipulator.GetName())

    if kin_solver is None:
        kin_solver = GenericKinematics(mani)
        logger.warn(
            "Construct kinematics solver using GenericKinematics, please pass the argument kin_solver if there is different result")
    joint_seed = mani.GetDoFPositions()

    def PosToJoint(pos, joint_seed):
        if traj.GetLieGroupName() in set(["R3xSO3", "SE3", "Pose"]):
            _, ik_report, _ = kin_solver.GetNearestIK(pos,
                                                      joint_seed)
            if ik_report.success:
                for i in range(joint_seed.DoF()):
                    joint_seed[i] = ik_report[0][i]
                return ik_report[0]
            else:
                raise ValueError("IK failed: {}".format(pos))
                return None
        else:
            return pos

    if show_trail:
        for i, s in enumerate(np.linspace(0, path_len, number)):
            joint = PosToJoint(path.GetConfig(s), joint_seed)
            robot_model = original_robot_model.Copy()
            robot_model.SetName(origin_name + "_traj_show_copy_%i" % name_idx)
            name_idx += 1
            mani = robot_model.GetManipulator(manipulator.GetName())
            mani.SetDoFPositions(joint)
            robot_vis.AddBody(robot_model)
            robot_models.append(robot_model)
            # robot_alpha > 0 and robot_vis.SetTransparency(
            #     robot_model, robot_alpha)
            robot_vis.ChooseShowMode(robot_model, show_mode)

    if draw_traj:
        points_number = kwargs.get("points_number", 0)
        handles.extend(DrawTraj(traj, show_in_vis=True,
                                show_in_plt=show_in_plt, view=robot_vis.m_viewer, kin_solver=kin_solver,
                                clear_view=clear_handles, points_number=points_number, show_waypoints=show_waypoints))

    # execute trajectory
    if loop_times:
        if not robot_models:
            robot_vis.AddBody(robot_model)
            robot_alpha and robot_vis.SetTransparency(robot_model, robot_alpha)
            robot_vis.ChooseShowMode(robot_model, show_mode)
            robot_models.append(robot_model)
        robot_model = robot_models[0]
        mani = robot_model.GetManipulator(manipulator.GetName())
        # robot_vis.SetTransparency(robot_model, 0)
    for _ in range(loop_times):
        number = int(duration // 0.01)
        for t in np.linspace(0, duration, number):
            joint = PosToJoint(traj.GetPosition(t), joint_seed)
            mani.SetDoFPositions(joint)
            time.sleep(0.01)

    # clear bodies if defined
    if clear_bodies and robot_models:
        rm_number = len(robot_models)
        for i in range(rm_number):
            robot_vis.RemoveBody(robot_models.pop())

    return robot_models, handles


def DrawPath(path, enable_3d=False, **kwargs):
    """Show path with matplotlib
    position args:
    path : path object that need to be shown
    kwargs:
    enable_3d [bool] : show path in 3d
    same_axis_lim [bool] : use same lim for each axis
    figsize [tuple] : fig widthxheight, eg: (8, 8) 
    savepath [str] : if want to save the fig, give file path, eg: "/home/fig.png"
    xlim [tuple] : fig xlim, eg: (-8, 8)
    ylim [tuple] : fig ylim, eg: (-8, 8)
    zlim [tuple] : fig zlim, eg: (-8, 8)
    """
    try:
        # user may be pass path as other type
        if not path.IsValid():
            logger.error("Error: path is not valid")
            return
    except Exception as e:
        logger.error("Got Exception : ", e)
        return
    s_set = np.linspace(0, path.GetLength(), 1000)
    pos = np.array(list(map(lambda s: path.GetConfig(s).Coeffs(), s_set)))
    waypoints = np.array([p.Coeffs() for p in path.GetWaypoints()])
    dims = 2
    fig = plt.figure(figsize=kwargs.get("figsize", (6, 6)))
    if pos.shape[1] >= 3:
        if enable_3d:
            ax = fig.add_subplot(111, projection='3d')
            dims = 3
        else:
            ax = fig.add_subplot(111)
        ax.plot(*pos[:, :dims].T, "r.-", lw=1, ms=2)
        ax.plot(*waypoints[:, :dims].T, "g*--", lw=1, ms=2)
    elif pos.shape[1] == 2:
        ax = fig.add_subplot(111)
        ax.plot(*pos[:, :2].T, "r.-", lw=1, ms=2)
        ax.plot(*waypoints[:, :2].T, "g*--", lw=1, ms=2)

    if kwargs.get("same_axis_lim", True):
        lim_min = pos[:, :dims].min(axis=0)
        lim_max = pos[:, :dims].max(axis=0)
        lim_range_max = (lim_max - lim_min).max()*1.1
        lims = list(zip((lim_max + lim_min) / 2 - lim_range_max /
                        2, (lim_max + lim_min) / 2 + lim_range_max / 2))
        ax.set_xlim(*lims[0])
        ax.set_ylim(*lims[1])
        if dims == 3:
            ax.set_zlim(*lims[2])

    if kwargs.get("xlim", None):
        ax.set_xlim(*kwargs["xlim"])
    if kwargs.get("ylim", None):
        ax.set_ylim(*kwargs["ylim"])
    if kwargs.get("zlim", None):
        ax.set_zlim(*kwargs["zlim"])
    if kwargs.get("savepath", None):
        plt.savefig(kwargs["savepath"], bbox_inches='tight',
                    dpi=300, quality=100)
    plt.show()
