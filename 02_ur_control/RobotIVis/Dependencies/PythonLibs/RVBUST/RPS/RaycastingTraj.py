#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Copyright (c) RVBUST, Inc - All rights reserved.

import math
import os
from itertools import chain

import numpy as np
import trimesh
import vedo
import vtk
from IPython import embed
from scipy.spatial.transform import Rotation

np.set_printoptions(precision=4, suppress=True)


def NpyToVtkMatrix(m):
    ma = vtk.vtkMatrix4x4()
    for i in range(4):
        for j in range(4):
            ma.SetElement(i, j, m[i, j])
    return ma


def DrawTrajInVtk(traj, pv, points_number=None, draw_axes=False, axis_length=0.1, color='g'):
    actors = []
    if not traj.IsValid():
        return actors
    if not points_number:
        points_number = math.ceil(traj.GetLength() / 0.01)
        points_number = max(points_number, 100)
    ts = np.linspace(0, traj.GetDuration(), points_number)
    poses = np.array([traj.GetPosition(t).Coeffs() for t in ts])
    # show origins
    lines = vedo.Lines(poses[:-1, :3], poses[1:, :3],
                       c=color, alpha=1, lw=1, dotted=False)
    pv += lines
    actors.append(lines)
    if draw_axes:
        # show rotations
        origins = poses[::10, :3]
        Rs = Rotation.from_quat(poses[::10, 3:7]).as_matrix()
        arrows_x = vedo.Arrows(
            origins, origins + Rs[:, :, 0], scale=axis_length, c='r')
        arrows_y = vedo.Arrows(
            origins, origins + Rs[:, :, 1], scale=axis_length, c='g')
        arrows_z = vedo.Arrows(
            origins, origins + Rs[:, :, 2], scale=axis_length, c='b')
        pv += arrows_x
        pv += arrows_y
        pv += arrows_z
        actors.extend([arrows_x, arrows_y, arrows_z])
    # show start direction
    direct = (poses[1, :3] - poses[0, :3]) / \
        np.linalg.norm(poses[1, :3] - poses[0, :3])
    arrow = vedo.Arrow(poses[0, :3], poses[0, :3] +
                       direct * axis_length * 2, c='k')
    pv += arrow
    actors.append(arrow)
    return actors


class MeshTraj:
    def __init__(self, mesh_file):
        self.mesh = trimesh.load_mesh(mesh_file)

    def GetAllWaypoints(self, direction, major_step, minor_step, extension, detect_hole=True, num_rays_between_holes=1, rotation_method=0, sort_method=0, normal_method=0):
        ray_origins, ray_dirs = self.GetParallelogramPlaneRays(
            *self.GetCorners(self.mesh, direction, 0.01), major_step, minor_step, sort_method)
        all_waypoints = self.IntersectWaypoints(
            ray_origins, ray_dirs, self.mesh, extension, rotation_method, detect_hole, num_rays_between_holes, normal_method)
        return all_waypoints

    @staticmethod
    def GetParallelogramPlaneRays(corner1: np.ndarray, corner2: np.ndarray, corner3: np.ndarray, major_step=None, minor_step=None, sort_method: int = 0, denser_at_ends=False):
        """Generate uniform-distributed rays in a rectangle plane. Ray casting direction is norm of the plane decided by corner1, corner2, corner3.

        Args:
            corner1 (np.ndarray): 1st corner point
            corner2 (np.ndarray): 2nd corner point
            corner3 (np.ndarray): 3rd corner point
            major_step: from corner1 to corner2, distance of rays
            minor_step: from corner2 to corner3, distance of each row of rays
            sort_method (int, optional): sequence of ray origins, 0: snake sequence； 1： zig-zag sequence. Defaults to 0. using the following code to show a difference visually:
            ```
            import matplotlib.pyplot as plt
            from mpl_toolkits.mplot3d import axes3d
            plt.subplot(projection='3d')
            plt.plot(*ray_origins.T, 'r-')
            plt.show()
            ```

        Returns:
            [type]: (ray_origins, ray_dirs)
        """
        Norm = np.linalg.norm
        major_dir = corner2 - corner1
        major_dir /= Norm(major_dir)
        minor_dir = corner3 - corner2
        minor_dir /= Norm(minor_dir)
        casting_dir = np.cross(major_dir, minor_dir)
        if not major_step:
            major_step = Norm(corner2 - corner1) / 3
        if not minor_step:
            minor_step = Norm(corner3 - corner2) / 3
        num_rays_per_row = math.ceil(
            (Norm(corner1 - corner2) + 0.51 * major_step) / major_step)

        num_rows = math.ceil(
            (Norm(corner3 - corner2) + 0.51 * minor_step) / minor_step)

        offset = (corner3-corner2) * minor_step / Norm(corner3-corner2)
        if denser_at_ends:
            corner1_approx = corner1 + 0.05 * (corner2 - corner1)
            corner2_approx = corner1 + 0.95 * (corner2 - corner1)
            row_origins1 = np.linspace(
                corner1, corner1_approx, max(num_rays_per_row // 10, 100), endpoint=False)
            row_origins2 = np.linspace(
                corner1_approx, corner2_approx, int(num_rays_per_row * 0.9), endpoint=False)
            row_origins3 = np.linspace(
                corner2_approx, corner2, max(num_rays_per_row // 10, 100))
            row_origins = np.r_[row_origins1, row_origins2, row_origins3]
        else:
            row_origins = np.linspace(corner1, corner2, num_rays_per_row)
        num_rays_per_row = row_origins.shape[0]

        ray_origins = []
        for i in range(num_rows):
            origins = row_origins.copy()
            if sort_method == 0:
                if i % 2 != 0:
                    origins = origins[::-1]
            ray_origins.append(origins)
            if Norm(offset) > Norm(corner3-row_origins[-1]):
                offset = corner3-row_origins[-1]
            row_origins += offset

        return np.vstack(ray_origins).reshape(num_rows, num_rays_per_row, 3), \
            np.tile(casting_dir, (num_rays_per_row * num_rows, 1)
                    ).reshape(num_rows, num_rays_per_row, 3)

    @staticmethod
    def DetectHoles(ray_indexs, num_rays_between_holes=1):
        idxs_diff = ray_indexs[1:] - ray_indexs[:-1]
        hole_idxs = ray_indexs[:-1][idxs_diff != 1]
        seg_idxs = [0]
        for i in range(ray_indexs.shape[0]-1):
            if ray_indexs[i+1] - ray_indexs[i] > num_rays_between_holes:
                seg_idxs.append(i+1)
        seg_idxs.append(len(ray_indexs))
        return seg_idxs

    @staticmethod
    def IntersectWaypoints(ray_origins, ray_dirs, mesh, extension=0.1, rotation_method=0, detect_hole=True, num_rays_between_holes=1, normal_method=0):
        """[summary]

        Args:
            ray_origins ([type]): (3,n,m)
            ray_dirs ([type]): (3,n,m)
            mesh ([type]): [description]
            extension (float, optional): [description]. Defaults to 0.1.
            rotation_method (int, optional): 0: use same x-rotation, 1: using forward direction as x-rotation. Defaults to 0.
            detect_hole (bool, optional): [description]. Defaults to True.
            num_rays_between_holes (int, optional): a judge to ignore small holes, how many rays go throught the hole at least. Defaults to 1.
            normal_method: 0: use intersected triangle normal; 1: use ray direction as normal

        Returns:
            [type]: [description]
        """
        Norm = np.linalg.norm
        all_waypoints = []
        for i in range(len(ray_origins)):
            row_locations, row_ray_indexs, row_triangle_indexs = mesh.ray.intersects_location(
                ray_origins[i], ray_dirs[i], multiple_hits=False)
            if row_locations.shape[0] < 2:
                continue
            ext_dir1 = row_locations[0] - row_locations[1]
            ext_dir2 = row_locations[-1] - row_locations[-2]
            seg_idxs = [0, len(row_ray_indexs)]
            if detect_hole:
                seg_idxs = __class__.DetectHoles(
                    row_ray_indexs, num_rays_between_holes)
            for k in range(len(seg_idxs)-1):
                locations = row_locations[seg_idxs[k]:seg_idxs[k+1]]
                locations_ext = np.zeros((locations.shape[0]+2, 3))
                locations_ext[1:-1] = locations
                p1st, p1st_l = locations[0], locations[-1]
                locations_ext[0] = p1st + extension * ext_dir1 / Norm(ext_dir1)
                locations_ext[-1] = p1st_l + \
                    extension * ext_dir2 / Norm(ext_dir2)

                if normal_method == 0:
                    norms = mesh.face_normals[row_triangle_indexs[seg_idxs[k]:seg_idxs[k+1]]]
                    dirs = ray_dirs[i][row_ray_indexs[seg_idxs[k]:seg_idxs[k+1]]]
                    # norm is close to ray direction
                    norms *= np.sign(np.sum(norms*dirs, axis=1)).reshape(-1, 1)
                else:
                    norms = ray_dirs[i][row_ray_indexs[seg_idxs[k]:seg_idxs[k+1]]]
                norms_ext = np.zeros((len(locations_ext), 3))
                norms_ext[1:-1] = norms
                norms_ext[0] = norms_ext[1]
                norms_ext[-1] = norms_ext[-2]

                xdirs = np.zeros_like(norms_ext)
                forward_dirs = locations_ext[1:] - locations_ext[:-1]
                forward_dirs /= np.linalg.norm(forward_dirs,
                                               axis=1).reshape(-1, 1)
                xdirs[1:] = forward_dirs
                xdirs[0] = xdirs[1]
                if rotation_method == 0 and i % 2 != 0:
                    xdirs = -xdirs

                ydirs = np.cross(norms_ext, xdirs)
                Rs = np.asarray([np.c_[xdirs[i], ydirs[i], norms_ext[i]]
                                for i in range(len(xdirs))])
                quats = Rotation.from_matrix(Rs).as_quat()
                all_waypoints.append(np.c_[locations_ext, quats])
        return all_waypoints

    @staticmethod
    def GetCorners(mesh, direction='+X/ZY', shrinkage=[[0.01, 0.01, 0.01], [0.01, 0.01, 0.01]], distance=0.1):
        """Get a face of bound box

        Args:
            mesh ([type]): [description]
            direction (str, optional): [description]. Defaults to '+X/ZY'.
            shrinkage (float, optional): to make the reactange facet a little smaller than real, so that the ray could be 
            intersects with the real mesh at the boundings. Defaults to 0.01.
            distance (float, optional): make facet a litter far from real mesh, so that ray origins with be outside of the mesh. Defaults to 0.1.

        Raises:
            ValueError: [description]

        Returns:
            [type]: [description]
        """
        direction = str(direction).upper()
        mesh_bounds = mesh.bounding_box.bounds.copy()
        mesh_bounds[0] += np.min([shrinkage[0], mesh.extents * 0.4], axis=0)
        mesh_bounds[1] -= np.min([shrinkage[1], mesh.extents * 0.4], axis=0)
        xmin, ymin, zmin = mesh_bounds[0]
        xmax, ymax, zmax = mesh_bounds[1]
        if direction == '+X/YZ':
            x = xmin - distance
            corner1 = np.array([x, ymin, zmin])
            corner2 = np.array([x, ymax, zmin])
            corner3 = np.array([x, ymax, zmax])
        elif direction == '+X/ZY':
            x = xmin - distance
            corner1 = np.array([x, ymin, zmax])
            corner2 = np.array([x, ymin, zmin])
            corner3 = np.array([x, ymax, zmin])
        elif direction == '-X/YZ':
            x = xmax + distance
            corner1 = np.array([x, ymax, zmin])
            corner2 = np.array([x, ymin, zmin])
            corner3 = np.array([x, ymin, zmax])
        elif direction == '-X/ZY':
            x = xmax + distance
            corner1 = np.array([x, ymin, zmin])
            corner2 = np.array([x, ymin, zmax])
            corner3 = np.array([x, ymax, zmax])
        elif direction == '+Y/XZ':
            y = ymin - distance
            corner1 = np.array([xmin, y, zmax])
            corner2 = np.array([xmax, y, zmax])
            corner3 = np.array([xmax, y, zmin])
        elif direction == '+Y/ZX':
            y = ymin - distance
            corner1 = np.array([xmin, y, zmin])
            corner2 = np.array([xmin, y, zmax])
            corner3 = np.array([xmax, y, zmax])
        elif direction == '-Y/XZ':
            y = ymax + distance
            corner1 = np.array([xmin, y, zmin])
            corner2 = np.array([xmax, y, zmin])
            corner3 = np.array([xmax, y, zmax])
        elif direction == '-Y/ZX':
            y = ymax + distance
            corner1 = np.array([xmin, y, zmax])
            corner2 = np.array([xmin, y, zmin])
            corner3 = np.array([xmax, y, zmin])
        elif direction == '+Z/XY':
            z = zmin - distance
            corner1 = np.array([xmin, ymin, z])
            corner2 = np.array([xmax, ymin, z])
            corner3 = np.array([xmax, ymax, z])
        elif direction == '+Z/YX':
            z = zmin - distance
            corner1 = np.array([xmin, ymax, z])
            corner2 = np.array([xmin, ymin, z])
            corner3 = np.array([xmax, ymin, z])
        elif direction == '-Z/XY':
            z = zmax + distance
            corner1 = np.array([xmax, ymin, z])
            corner2 = np.array([xmin, ymin, z])
            corner3 = np.array([xmin, ymax, z])
        elif direction == '-Z/YX':
            z = zmax + distance
            corner1 = np.array([xmin, ymin, z])
            corner2 = np.array([xmin, ymax, z])
            corner3 = np.array([xmax, ymax, z])
        else:
            raise ValueError(f"Unknown direction: {direction}")
        print(f"corner1: {corner1}\ncorner2: {corner2}\ncorner3: {corner3}")
        return corner1, corner2, corner3


if __name__ == "__main__":
    from RVBUST.RPS import *

    # vtk is used to show the scene, as Vis is too slow to load a large mesh
    cfd = os.path.dirname(__file__)
    mesh_file = os.path.join(cfd, "Example/BoxWithHole.ply")
    pv = vedo.Plotter(axes=1)
    pv.addHoverLegend()
    mesh = pv.load(mesh_file)

    mesh_traj = MeshTraj(mesh_file)
    trajs = []
    hs = []
    colors = 'gb'
    all_directions = ('+X/YZ', '+X/ZY', '-X/YZ', '-X/ZY', '+y/xz',
                      '+y/zx', '-y/xz', '-y/zx', '+z/xy', '+z/yx', '-z/xy', '-z/yx')
    for i, direction in enumerate(['+x/yz', '+x/zy', '+z/xy', '+z/yx']):
        # for i, direction in enumerate(all_directions):
        corners = mesh_traj.GetCorners(
            mesh_traj.mesh, direction, shrinkage=[[0.01, 0.01, 0.01], [0.01, 0.01, 0.01]], distance=0.1)
        ray_origins, ray_dirs = mesh_traj.GetParallelogramPlaneRays(
            *corners, major_step=0.01, minor_step=0.08, sort_method=0)
        all_waypoints = mesh_traj.IntersectWaypoints(
            ray_origins, ray_dirs, mesh_traj.mesh, extension=0.05, rotation_method=0, detect_hole=True, num_rays_between_holes=2, normal_method=0)
        # combine as one trajectory
        all_waypoints = [list(chain.from_iterable(all_waypoints))]
        for waypoints in all_waypoints:
            path = CreatePath(waypoints, 0.1, PathType_Bezier5thBlend)
            traj = CreateTrajectory(path)
            hs.extend(DrawTrajInVtk(
                traj, pv, color=colors[i % 2], points_number=4000, axis_length=0.02, draw_axes=True))
            trajs.append(traj)

        # plot corner points
        # corner_pts = vedo.Points(corners, c='r', r=10)
        # pv += corner_pts
        # plot rays
        # ray_origins = np.vstack(ray_origins)
        # ray_dirs = np.vstack(ray_dirs)
        # rays = vedo.Arrows(ray_origins, ray_origins +
        #                    ray_dirs, c='y', scale=0.1)
        # pv += rays

    pv.show()
    embed()
