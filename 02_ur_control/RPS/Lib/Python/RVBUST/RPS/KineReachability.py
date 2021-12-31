#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Copyright (c) RVBUST, Inc - All rights reserved.

import os

import h5py
import numpy as np
from RVBUST.RCI import *

try:
    from .PyRPS import *
except ImportError:
    from RVBUST.RPS.PyRPS import *

logger = Logger.GetConsoleLogger("RVS_CL")


class SpaceSampler(object):
    def __init__(self):
        self.m_face_indices = self.m_face_numr = self.m_face_nump = None

    def UniformlySampleSpace(self, radius, delta):
        nsteps = np.floor(radius/delta)
        logger.Info("space sample steps: {}".format(nsteps))
        grid_x, grid_y, grid_z = np.mgrid[-nsteps:nsteps, -
                                          nsteps:nsteps, -nsteps:nsteps]
        all_points = np.c_[grid_x.flat, grid_y.flat, grid_z.flat] * delta
        inside_inds = np.flatnonzero(np.sum(all_points**2, 1) < radius**2)
        return all_points, inside_inds, grid_x.shape

    def ComputeFaceIndices(self, num):
        if self.m_face_indices is None or len(self.m_face_indices[0]) < num:
            indices = np.arange(num**2)
            # separate the odd and even bits into odd,even
            maxiter = int(np.log2(len(indices)))
            oddbits = np.zeros(num**2, int)
            evenbits = np.zeros(num**2, int)
            mult = 1
            for i in range(maxiter):
                oddbits += (indices & 1)*mult
                evenbits = evenbits + mult*((indices & 2)/2)
                indices >>= 2
                mult *= 2
            self.m_face_indices = [oddbits+evenbits, oddbits-evenbits]
        if self.m_face_numr is None or len(self.m_face_numr) != num*12:
            self.m_face_numr = np.reshape(np.transpose(
                np.tile([2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4], (num, 1))), num*12)
            self.m_face_nump = np.reshape(np.transpose(
                np.tile([1, 3, 5, 7, 0, 2, 4, 6, 1, 3, 5, 7], (num, 1))), num*12)

    def SampleS2(self, level=0, angledelta=None):
        """Uniformly sample in S2 LieGroup, uses healpix algorithm with ordering from Yershova et. al. 2009 journal paper"""
        if angledelta is not None:
            # select the best sphere level matching angledelta;
            # level,delta:
            # [0, 1.0156751592381095]
            # [1, 0.5198842203445676]
            # [2, 0.25874144949351713]
            # [3, 0.13104214473149575]
            # [4, 0.085649339187184162]
            level = max(0, int(0.5-np.log2(angledelta)))
        n_side = 2**level
        n_side2 = n_side**2
        N = 12*n_side**2
        self.ComputeFaceIndices(n_side**2)
        # compute sphere z coordinate
        jr = self.m_face_numr*n_side - \
            np.tile(self.m_face_indices[0][0:n_side2], 12)-1
        nr = np.tile(n_side, N)
        z = 2*(2*n_side-jr)/(3.0*n_side)
        kshift = np.mod(jr-n_side, 2)
        # north pole test
        northpoleinds = np.flatnonzero(jr < n_side)
        nr[northpoleinds] = jr[northpoleinds]
        z[northpoleinds] = 1.0 - nr[northpoleinds]**2*(1.0/(3.0*n_side2))
        kshift[northpoleinds] = 0
        # south pole test
        southpoleinds = np.flatnonzero(jr > 3*n_side)
        nr[southpoleinds] = 4*n_side - jr[southpoleinds]
        z[southpoleinds] = -1.0 + nr[southpoleinds]**2*(1.0/(3.0*n_side2))
        kshift[southpoleinds] = 0
        # compute pfi
        facenump = np.reshape(np.transpose(
            np.tile([1, 3, 5, 7, 0, 2, 4, 6, 1, 3, 5, 7], (n_side2, 1))), N)
        jp = (self.m_face_nump*nr +
              np.tile(self.m_face_indices[1][0:n_side2], 12)+1+kshift)/2
        jp[jp > 4*n_side] -= 4*n_side
        jp[jp < 1] += 4*n_side
        return np.arccos(z), (jp-(kshift+1)*0.5)*((0.5*np.pi)/nr)

    def Hopf2quat(self, hopfarray):
        """convert hopf rotation coordinates to quaternion [x, y, z, w]"""
        half0 = hopfarray[:, 0]*0.5
        half2 = hopfarray[:, 2]*0.5
        c0 = np.cos(half0)
        c2 = np.cos(half2)
        s0 = np.sin(half0)
        s2 = np.sin(half2)
        # return np.c_[c0*c2,c0*s2,s0*np.cos(hopfarray[:,1]+half2),s0*np.sin(hopfarray[:,1]+half2)]
        # quaternion order is same as Pose.Quat()
        return np.c_[c0*s2, s0*np.cos(hopfarray[:, 1]+half2), s0*np.sin(hopfarray[:, 1]+half2), c0*c2]

    def SampleSO3(self, level=0, quatdelta=None):
        """Uniformly Sample 3D Rotations.
        If quatdelta is specified, will compute the best level aiming for that average quaternion distance.
        Algorithm From
        A. Yershova, S. Jain, S. LaValle, J. Mitchell "Generating Uniform Incremental Grids on SO(3) Using the Hopf Fibration", International Journal of Robotics Research, Nov 13, 2009.
        """
        if quatdelta is not None:
            # level=0, quatdist = 0.5160220
            # level=1: quatdist = 0.2523583
            # level=2: quatdist = 0.120735
            level = max(0, int(-0.5-np.log2(quatdelta)))
        s1samples, step = np.linspace(
            0.0, 2*np.pi, 6*(2**level), endpoint=False, retstep=True)
        s1samples += step*0.5
        theta, pfi = self.SampleS2(level)
        band = np.zeros((len(s1samples), 3))
        band[:, 2] = s1samples
        qarray = np.zeros((0, 4))
        for i in range(len(theta)):
            band[:, 0] = theta[i]
            band[:, 1] = pfi[i]
            qarray = np.r_[qarray, self.Hopf2quat(band)]
        return qarray

    def SampleSO2(self, level=0, quatdelta=None, axis=[0, 0, 1]):
        if quatdelta is not None:
            level = max(0, int(-0.5-np.log2(quatdelta)))
        s1samples, step = np.linspace(
            0.0, 2*np.pi, 6*(2**level), endpoint=False, retstep=True)
        s1samples += step*0.5
        qarray = []
        for theta in s1samples:
            qarray_temp = np.r_[
                np.array(axis) * np.sin(theta/2), np.cos(theta/2)]
            qarray.append(qarray_temp)
        return qarray


class Reachability(object):
    def __init__(self):
        self.m_file_name = None
        self.m_file_handle = None
        self.m_reachability_stats = None
        self.m_reachability3d = None
        # the minimal step of translation in sample space. (Unit: m), default is 0.04
        self.m_xyz_delta = 0.04
        # the minimal step of rotation in sample space. (Unit: nearly 2*rad), default is 0.5
        self.m_quat_delta = 0.5
        self.m_sample_type = None
        self.m_sample_info = None
        self.m_robot_model = None
        self.m_kin_solver = None
        self.m_show_handles = []
        self.m_robot_vis = None
        self.m_env = Environment()
        self.m_is_init = False

    def IsInit(self):
        return self.m_is_init

    def Init(self, robot_model, kin_solver):
        manipulator = robot_model.GetActiveManipulator()
        if not manipulator:
            logger.Error("Invalid robot_model")
            self.m_is_init = False
            return False

        self.m_robot_model = robot_model
        # default path
        self.m_robot_name = self.m_robot_model.GetName()
        self.m_manip_name = self.m_robot_model.GetActiveManipulator().GetName()
        self.m_file_name = GetDataPath() + "Database/RobotWorksapce/{0}_{1}_{2}_{3}.h5".format(
            self.m_robot_name, self.m_manip_name, self.m_xyz_delta, self.m_quat_delta)

        self.m_kin_solver = kin_solver
        self.m_is_init = True
        return True

    def SetFileName(self, file_name):
        if file_name is not None:
            self.m_file_name = file_name

    def SetSampleDelta(self, xyz_delta, quat_delta):
        if xyz_delta > 0:
            self.m_xyz_delta = xyz_delta
        if quat_delta > 0:
            self.m_quat_delta = quat_delta

        self.m_file_name = GetDataPath() + "Database/RobotWorksapce/{0}_{1}_{2}_{3}.h5".format(
            self.m_robot_name, self.m_manip_name, self.m_xyz_delta, self.m_quat_delta)

    def SaveHDF5(self):
        if not self.m_is_init:
            logger.Error("KineReachability has not been initialized")
            return False
        if len(self.m_file_name) == 0:
            logger.Warn("File name invalid.")
            return
        f = h5py.File(self.m_file_name, 'w')
        f['xyz_delta'] = self.m_xyz_delta
        f['quat_delta'] = self.m_quat_delta
        f['reachability_stats'] = self.m_reachability_stats
        f['reachability3d'] = self.m_reachability3d
        f['sample_type'] = self.m_sample_type
        f['sample_info'] = self.m_sample_info
        f.close()
        logger.Info("Save data ok.")

    def LoadHDF5(self):
        if os.path.exists(self.m_file_name) == False:
            logger.Warn("File dosen't exist.")
            return False
        if self.m_file_handle is not None:
            self.m_file_handle.close()
            self.m_file_handle = None

        f = h5py.File(self.m_file_name, 'r')
        self.m_xyz_delta = f['xyz_delta'].value
        self.m_quat_delta = f['quat_delta'].value
        self.m_reachability_stats = f['reachability_stats']
        self.m_reachability3d = f['reachability3d']
        self.m_sample_type = f['sample_type'].value
        self.m_sample_info = f['sample_info'].value
        self.m_file_handle = f
        f = None
        self.m_show_handles = []
        logger.Info("Load data ok.")
        logger.Info("xyz_delta: {}  quat_delta: {}".format(
            self.m_xyz_delta, self.m_quat_delta))
        logger.Info("sample type: {}  sample info: {}".format(
            self.m_sample_type, self.m_sample_info))
        return True

    def GenerateReachData(self, sample_type=0, axis=[0, 0, 1], posture=[0, 0, 0, 1]):
        """
        Generate reachability data.
        sample_type: sample type of posture. 0-Rotation, 1-SO2 based on fixed axis, 2-fixed posture. 
        axis: fixed screw axis when sample_type is 1.
        posture: fixed posture when sample_type is 2.
        """
        if not self.m_is_init:
            logger.Error("KineReachability has not been initialized")
            return False

        robot = self.m_robot_model
        kin_solver = self.m_kin_solver
        manip = robot.GetManipulator("Arm")
        joints_name = manip.GetDoFNames()

        max_radius = 0
        base_trans = robot.GetJoint(
            joints_name[0]).GetLocalPose().Translation()
        for joint in joints_name[:0:-1]:
            pose = robot.GetJoint(joint).GetLocalPose()
            trans = pose.Translation()
            max_radius += np.linalg.norm(trans)
        tool = manip.GetActiveEndEffector().GetTCP()
        max_radius += np.linalg.norm(tool.Translation())
        max_radius = 1.05 * max_radius
        logger.Info("space sample radius: {}m".format(max_radius))

        if self.m_xyz_delta is None:
            self.m_xyz_delta = np.round(max_radius/50, 3)  # 0.001m
        if self.m_quat_delta is None:
            self.m_quat_delta = 0.5

        sample = SpaceSampler()
        all_points, inside_inds, shape = sample.UniformlySampleSpace(
            max_radius, delta=self.m_xyz_delta)
        qarray = None
        if sample_type == 0:
            qarray = sample.SampleSO3(quatdelta=self.m_quat_delta)
            self.m_sample_info = self.m_quat_delta
        elif sample_type == 1:
            qarray = sample.SampleSO2(quatdelta=self.m_quat_delta, axis=axis)
            self.m_sample_info = axis
        elif sample_type == 2:
            qarray = np.array([posture])
            self.m_sample_info = posture
        else:
            logger.Warn("The type is not implemented yet")
            return
        self.m_sample_type = sample_type

        self.m_reachability3d = np.zeros(np.prod(shape))
        self.m_reachability_stats = []
        T = np.zeros(7)
        for i, ind in enumerate(inside_inds):
            trans = all_points[ind]+base_trans
            num_valid = 0
            num_rotvalid = 0
            T[0:3] = trans
            for quat in qarray:
                T[3:7] = quat
                pose = Pose(T)
                res, solutions = kin_solver.GetPositionIK(pose)
                if res == 0 and solutions.ik_number != 0:
                    num_valid += 1  # solutions.ik_number
                    num_rotvalid += 1
            self.m_reachability_stats.append(
                np.r_[trans, num_valid/float(len(qarray))])
            self.m_reachability3d[ind] = num_rotvalid/float(len(qarray))
            if i % np.floor(len(inside_inds)/10) == 0:
                logger.Info("current progress: {}%".format(
                    np.floor(100*i/len(inside_inds))))
        logger.Info("current progress: 100.0%. Computation is finished.")
        self.m_reachability3d = np.reshape(self.m_reachability3d, shape)
        self.m_reachability_stats = np.array(self.m_reachability_stats)

    def ShowReachData(self, show_type=0, plane=None, distance=0, env=None):
        """
        Show reachability data.
        show_type: the show type of points, 0-vis sphere, otherwise-Mayavi  
        plane: the coefficients [x0,y0,z0,a,b,c] of the plane that will display, (x0,y0,z0)  the point in the plane, (a,b,c) represents the normalvector,
               subject to a(x-x0) + b(y-y0) + c(z-z0) = 0. When plane is None, it will show all the worksapce reachability, or it will show the reachability in the plane. 
        """
        if not env == None:
            self.m_robot_vis = env
        elif self.m_robot_vis == None:
            self.m_robot_vis = RobotVis()
            self.m_robot_vis.LoadEnvironment(self.m_env)
            self.m_env.AddBody(self.m_robot_model)

        radius = self.m_xyz_delta/2
        if show_type == 0:  # Todo
            if plane is not None:  # to show workspace in plane
                n = plane[3:6]
                if np.linalg.norm(n) < 1e-2:
                    logger.Warn("the normal vector is invalid")
                    return
                for stats in self.m_reachability_stats:
                    point = stats[0:3]
                    point0 = plane[0:3]
                    n = plane[3:6]
                    # |a(x-x0)+b(y-y0)+c(z-z0)| / sqrt(a*a+b*b+c*c)
                    dist = np.abs((point-point0).dot(n)) / np.linalg.norm(n)
                    # if dist > radius:
                    if dist > max(radius, distance):
                        continue
                    ratio = stats[-1]
                    if stats[-1] > 1e-4:
                        handle = self.m_robot_vis.GetView().Sphere(
                            stats[0:3], radius, [0, 1, 0, ratio])
                        self.m_show_handles.append(handle)
            else:  # to show the whole workspace
                for stats in self.m_reachability_stats:
                    ratio = stats[-1]/10  # to show more clearly
                    if stats[-1] > 1e-4:
                        handle = self.m_robot_vis.GetView().Sphere(
                            stats[0:3], radius, [0, 1, 0, ratio])
                        self.m_show_handles.append(handle)
        else:
            self._ShowReachDataMayavi()

    def _ShowReachDataMayavi(self, contours=[0.01, 0.1, 0.2, 0.5, 0.8, 0.9, 0.99], opacity=None, figureid=1, xrange=None, options=None):
        try:
            mlab = __import__('mayavi.mlab', fromlist=['mlab'])
        except:
            print("Please install mayavi and PyQt to show result:")
            print("\tpip install mayavi")
            print("\tpip install PyQt5")
            return

        mlab.figure(figureid, fgcolor=(0, 0, 0),
                    bgcolor=(1, 1, 1), size=(1024, 768))
        mlab.clf()
        reachability3d = self.m_reachability3d
        reachability3d = np.minimum(reachability3d, 1.0)
        if xrange is None:
            src = mlab.pipeline.scalar_field(reachability3d)
        else:
            src = mlab.pipeline.scalar_field(np.r_[np.zeros(
                (1,)+reachability3d.shape[1:]), reachability3d[xrange, :, :], np.zeros((1,)+reachability3d.shape[1:])])
        for i, c in enumerate(contours):
            mlab.pipeline.iso_surface(src, contours=[c], opacity=min(
                1, 0.7*c if opacity is None else opacity[i]))
        mlab.show()

    def HideReachViewer(self):
        if len(self.m_show_handles) != 0:
            for handle in self.m_show_handles:
                self.m_robot_vis.GetView().Hide(handle)

    def ReshowReachViewer(self):
        if len(self.m_show_handles) != 0:
            for handle in self.m_show_handles:
                self.m_robot_vis.GetView().Show(handle)

    def DeleteReachViewer(self):
        if len(self.m_show_handles) != 0:
            self.m_robot_vis.GetView().Delete(self.m_show_handles)
        self.m_show_handles.clear()
