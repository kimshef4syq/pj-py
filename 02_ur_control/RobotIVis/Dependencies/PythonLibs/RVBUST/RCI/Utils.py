#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Copyright (c) RVBUST, Inc - All rights reserved.

import os
import re
import numpy as np


def TestNetworkDelay(ip='127.0.0.1'):
    """ Test the network delay to given host

    :param ip: [host ip address]
    :type ip: [str]
    :return: delay in ms if > 0 else error code
    :rtype: [str]
    """
    try:
        cmd = 'ping -c 1 ' + ip
        reply = os.popen(cmd).read()
        pattern_reachable = '.*(\d+)% packet loss.*avg.*(\d+.\d+)/(\d+.\d+)/(\d+.\d+)/(\d+.\d+).*'
        pattern_unreachable = '.*(\d+)% packet loss.*'
        m = re.search(pattern_unreachable, reply, flags=re.DOTALL)
        if m is None:
            return '-1.0'  # didn't match how many packet lost
        if m.group(1) == '0':  # no packet loss
            m2 = re.search(pattern_reachable, reply, flags=re.DOTALL)
        else:  # there is packet lost
            return '-2.0'
        if m2 is None:  # didn't match the average delay time
            return '-3.0'
        return m2.group(3)  # normal return the network delay in ms
    except Exception as e:
        return '-4.0'  # other exception


def SetNetWorkProcessPriority():
    """ Use this function to set the priority of system network related process to be highest
    :return: True if succees else Fasle
    """
    # Need sudo priority
    import os
    import re
    cmd = "ps -ef | grep enp"
    print("Exec cmd:\n{}".format(cmd))
    s = os.popen(cmd).read()
    print("result:\n{}".format(s))
    network_pids = re.findall(r".*root\s+(\d+)\s+\d+.*irq/\d+-enp\d+s\d+", s,
                              re.DOTALL)
    ret = "Error"
    for pid in network_pids:
        cmd = "sudo chrt --pid 99 %s" % pid
        print("Exec cmd:\n{}".format(cmd))
        ret = os.popen(cmd).read()
        print("result:\n{}".format(ret))
    reuslt = ret == ""
    print("SetNetWorkProcessPriority: {}".format(reuslt))
    return reuslt


def GetBoxFromVertices(bbox, show_in_vis=False):
    from sklearn.decomposition import PCA
    """GetBox pose and size from eight points

    :param bbox: point list, 8-vertices
    :type bbox: numpy.ndarray
    :param show_in_vis: whether show in vis.view, defaults to True
    :type show_in_vis: bool, optional
    :return: (pose, [length, width, height])
    :rtype: [type]
    """
    if len(bbox) != 8:
        return False, SE3(), [0, 0, 0]

    pca = PCA(3)
    pca.fit(bbox)

    position = pca.mean_
    ax, ay, az = pca.components_
    az = -az
    R = np.c_[ax, ay, az]

    bx = by = bz = 0

    for b in bbox:
        bx += np.abs(np.dot(b - position, ax))
        by += np.abs(np.dot(b - position, ay))
        bz += np.abs(np.dot(b - position, az))

    length = bx / 4
    width = by / 4
    height = bz / 4

    if show_in_vis:
        from RVBUST.Vis import View
        from RVBUST.RCI import SO3, SE3
        viewer = View("GetBox")
        viewer.Axes([0, 0, 0], [0, 0, 0, 1], .2, 2)
        ph = viewer.Point(bbox.flatten(), 4)
        viewer.Home()

        viewer.Point(position.flatten(), 10, [0, 0, 1])

        # vis Box's extents is half size of box size
        bh = viewer.Box(position, [length / 2, width / 2, height / 2])
        viewer.SetTransparency(bh, .8)
        viewer.SetTransform(bh, position, SO3(R).Coeffs())
        viewer.Clear()

    return True, SE3(position, R), [length, width, height]


def ConvertJointLimits2Arr(joint_limits):
    """Get max_vels, max_accs, max_jerks from list of joint limits. By default, acc=3.0*vel, jerk=3.0*acc.
    :param joint_limits: joint limits
    :type joint_limits: List[JointLimit]
    :return: (max_vels, max_accs, max_jerks)
    :rtype: tuple(np.ndarray[np.float, 1, dof],np.ndarray[np.float, 1, dof],np.ndarray[np.float, 1, dof])
    """
    max_vels = [
        jl.GetMaxVelocity() if jl.HasVelocityLimits() else 1.0
        for jl in joint_limits
    ]
    max_accs = [
        jl.GetMaxAcceleration()
        if jl.HasAccelerationLimits() else max_vels[i] * 3.0
        for i, jl in enumerate(joint_limits)
    ]
    max_jerks = [
        jl.GetMaxJerk() if jl.HasJerkLimits() else max_accs[i] * 3.0
        for i, jl in enumerate(joint_limits)
    ]
    return np.array([max_vels, max_accs, max_jerks])


def ConvertMeshUnitFromMillimetreToMeter(source_path, save_path):
    mesh_type_list = ["stl", "STL", "obj", "OBJ", "3ds", "3DS", "dae", "DAE"]
    if os.path.exists(source_path):
        if (os.path.isdir(source_path)):
            file_list = os.listdir(source_path)
            for mesh_file in file_list:
                if not os.path.isdir(mesh_file):
                    mesh_type = mesh_file.split(".")[-1]
                    if mesh_type in mesh_type_list:
                        if os.path.isdir(source_path + mesh_file) == False:
                            # mm -> m
                            mesh = Mesh(source_path + "/" + mesh_file,
                                        (0.001, 0.001, 0.001))
                            mesh.ExportSTL(os.path.join(save_path, mesh_file))
                    else:
                        logger.Error("Mesh type is : {}".format(mesh_type))
        if (os.path.isfile(source_path)):
            mesh_type = source_path.split(".")[-1]
            if mesh_type in mesh_type_list:
                # mm -> m
                mesh = Mesh(source_path + "/" + mesh_file,
                            (0.001, 0.001, 0.001))
                mesh.ExportSTL(os.path.join(save_path, mesh_file))
            else:
                logger.Error("Mesh type is : {}".format(mesh_type))
    else:
        logger.Error("The path is not exists!")


if __name__ == "__main__":
    ip = '127.0.0.1'
    delay = TestNetworkDelay()
    print(f"delay to {ip} is {delay}ms")
    SetNetWorkProcessPriority()
