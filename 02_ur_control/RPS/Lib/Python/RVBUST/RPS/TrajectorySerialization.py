#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Copyright (c) RVBUST, Inc - All rights reserved.
import datetime
import json
import time
import os

from RVBUST.RCI import *

try:
    from .PyRPS import *
except ImportError:
    from RVBUST.RPS.PyRPS import *

logger = RichLogger(__name__)


def SaveTraj(traj, fp: str = GetDataPath() + "RVS/Robotics/Trajectory/TrajData.json", add_time_stamp: bool = False, **kwargs) -> str:
    """Save trajectory to a json file

    :param traj: trajectory to be saved
    :type traj: [type]
    :param fp: file path, defaults to GetDataPath()+"RVS/Robotics/Trajectory/TrajData.json"
    :type fp: str, optional
    :param add_time_stamp: automatically add time stamp to fp, eg: "~/trajdata_2019_08_09__12_11_11.json", defaults to False
    :type add_time_stamp: bool, optional
    :return: file path of the saved json file
    :rtype: str
    """
    try:
        timestamp = datetime.datetime.now().strftime("%Y_%m_%d__%H_%M_%S")
        if add_time_stamp:
            fp = "{0}_{2}{1}".format(*os.path.splitext(fp), timestamp)
        path = traj.GetPath()
        if isinstance(traj, TrajectoryBaseRn):
            groupname = "Rx"
        else:
            groupname = traj.GetLieGroupName()
        st = traj.GetPathParameter()
        coeffs2d = []
        for i in range(st.GetNumOfSegs()):
            coeffs2d.append(list(st[i].GetCoeffs()))

        traj_data = {"name": "Trajectory{}".format(groupname),
                     "group": "{}".format(groupname),
                     "timestamp": time.ctime(),
                     "path_type": int(path.GetType()),
                     "blend_tolerance": path.GetBlendTolerance(),
                     "path_parameter_coeffs": coeffs2d,
                     "path_parameter_knots": st.GetKnots()
                     }
        if path.GetType() != PathType_Composed:
            traj_data["waypoints"] = [p.Coeffs().tolist()
                                      for p in path.GetWaypoints()]
        else:
            seg_num = path.GetPathSegmentsNumber()
            seg_infos = []
            for i in range(seg_num):
                seg = path.GetPathSegmentByIndex(i)
                seg_infos.append({"seg_waypoints": [p.Coeffs().tolist() for p in seg.GetWaypoints()],
                                  "seg_type": str(seg.GetType()),
                                  "start_position": seg.GetStartParameter(),
                                  "seg_length": seg.GetLength()})
            traj_data["seg_infos"] = seg_infos
        os.makedirs(os.path.dirname(fp), exist_ok=True)
        with open(fp, "w") as f:
            json.dump(traj_data, f, indent=True, sort_keys=True)
        logger.info(f"Trajectory have been saved to file: {fp}")
        return fp
    except Exception as e:
        logger.error("Got exception {} when saving trajectory".format(e))
        return None


def LoadTraj(fp: str):
    """Load traj, restore a trajectory object from json data

    :param fp: file path
    :type fp: str
    :raises ValueError: [description]
    :return: trajectory object if success else None
    :rtype: [type]
    """
    try:
        traj_data = None
        with open(fp, "r") as f:
            traj_data = json.load(f)
        if traj_data is None:
            return None
        timestamp = traj_data['timestamp']
        logger.info(
            "Load trajectory from file: {}, which was save at {}".format(fp, timestamp))
        group = traj_data["group"]
        if group == "SE3":
            group = "Pose"
        blend_tolerance = traj_data["blend_tolerance"]
        path_type = PathType(traj_data["path_type"])

        if path_type != PathType_Composed:
            way_pts = [eval(f"{group}(wp)") for wp in traj_data["waypoints"]]
            code = f"CreatePath(way_pts, blend_tolerance, path_type)"
            path = eval(code)
        else:
            path = None
            seg_infos = traj_data['seg_infos']
            for seg_info in seg_infos:
                ms = [eval("{group}(p)".format(group=group))
                      for p in seg_info["seg_waypoints"]]
                seg_type = eval(seg_info["seg_type"])
                # restore a inscribed circle is not supported
                if seg_type == PathSegmentType_Circle_Inscribed:
                    raise ValueError(
                        "Restoring trajectory failed: restoring an inscribed circle segment is not supported, please use Bezier segment instead")
                else:
                    seg = eval(
                        "PathSegment{group}(*ms, seg_type=seg_type)".format(group=group))
                    seg.SetLength(seg_info["seg_length"])
                if path is None:
                    path = CreatePath(seg)
                else:
                    path.AppendPathSegment(seg)
        traj = CreateTrajectory(path)
        st = PSpline1d()
        coeffs2d = traj_data['path_parameter_coeffs']
        knots = traj_data['path_parameter_knots']
        for i, coeffs in enumerate(coeffs2d):
            poly = Polynomial1d(coeffs)
            st.PushBack(poly, knots[i+1]-knots[i])
        traj.SetPathParameter(st)
        return traj
    except Exception as e:
        logger.error("Got exception {} when loading trajectory".format(e))
        return None
