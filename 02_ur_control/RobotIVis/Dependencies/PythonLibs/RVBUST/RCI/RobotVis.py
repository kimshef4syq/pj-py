#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Copyright (c) RVBUST, Inc - All rights reserved.

import atexit
import random
import sys
import threading
import time

from typing import Tuple, List, Union
import numpy as np
from RVBUST import Vis

from .PyRCI import *
from .RichLogger import RichLogger


class LinkData(object):
    def __init__(self):
        self.m_axes_handle = None
        self.m_display_mode = 0b001
        self.m_collision_geometry_handle = None
        self.m_visual_geometry_handle = None
        self.m_lock = threading.Lock()


class RobotVis(object):
    """RobotVis is a robot 3D simulation environment with the help of Vis. 
    """

    def __init__(self, vis=None):
        """Construct RobotVis

        :param vis: View object, if None, a new View will be created, defaults to None
        :type vis: [Vis.View], optional
        """
        self.m_logger = RichLogger("RobotVis")
        self.m_logger.debug("Constructing RobotVis")

        if vis == None:
            self.m_view = Vis.View("RobotVis")
        else:
            self.m_view = vis
        self.m_world_axes = self.m_view.Axes([0, 0, 0], [0, 0, 0, 1], 0.2, 2)
        self.m_env = Environment()
        self.m_links_map = {}

        self.m_update_rate = 1/24
        self.is_update = True
        self.m_lock = threading.Lock()
        self.m_threads = threading.Thread(target=self._Update, daemon=True)
        self.m_threads.start()

        self.m_is_dragging = False
        self.m_drag_kin_solver = None
        self.m_drag_body = None
        self.m_display_scale = 0.2

        atexit.register(self.Exit)

    def GetView(self) -> Vis.View:
        """Get View instance

        :return: [description]
        :rtype: Vis.View
        """
        return self.m_view

    def LoadEnvironment(self, env: Environment) -> bool:
        """Load an environment

        :param env: environment object
        :type env: Environment
        :return: successful or not
        :rtype: bool
        """
        self.m_lock.acquire()
        self.m_env = Environment()
        if not self._EnvironmentSync():
            self.m_lock.release()
            return False
        self.m_env = env
        if not self._EnvironmentSync():
            self.m_lock.release()
            return False
        self.m_lock.release()
        return True

    def AddBody(self, body: Multibody) -> bool:
        """Add a multibody

        :param body: multibody to be loaded
        :type body: Multibody
        :return: successful or not
        :rtype: bool
        """
        if isinstance(body, GenericRobotController):
            return self.AddBody(body.GetRobotModel())
        if not self.m_env.AddBody(body):
            return False
        self.m_lock.acquire()
        if not self._EnvironmentSync():
            self.m_lock.release()
            return False
        self.m_lock.release()
        return True

    def RemoveBody(self, body: Multibody) -> bool:
        """Remove a multibody

        :param body: multibody to be removed
        :type body: Multibody
        :return: successful or not
        :rtype: bool
        """
        if not self.m_env.RemoveBody(body):
            return False
        self.m_lock.acquire()
        if not self._EnvironmentSync():
            self.m_lock.release()
            return False
        self.m_lock.release()
        return True

    def SetColor(self, obj: Union[Multibody, Vis.Handle], color: List[float] = [1.0, 0.0, 0.0, 1.0]) -> bool:
        """Set a Multibody's color

        :param obj: multibody or handle
        :type obj: Union[Multibody, Vis.Handle]
        :param color: desired color in rgba format, defaults to [1.0, 0.0, 0.0, 1.0]
        :type color: List[float], optional
        :return: success or failure
        :rtype: bool
        """
        if isinstance(obj, Vis.Handle):
            return self.m_view.SetColor(obj, color)
        elif isinstance(obj, Multibody):
            for link in obj.GetLinks():
                ret, link_data = self.GetLinkData(link)
                if ret:
                    h = link_data.m_visual_geometry_handle
                    if h is not None and not self.SetColor(h, color):
                        return False
            return True
        else:
            return False

    def AddSphere(self, radius: float, pose: Pose = Pose(), color: List[float] = [1.0, 0.0, 0.0, 1.0]) -> Multibody:
        """Add a sphere to environment

        :param radius: sphere radius
        :type radius: float
        :param pose: sphere position and rotation, defaults to Pose()
        :type pose: Pose, optional
        :param pose: sphere position and rotation, defaults to Pose()
        :type pose: Pose, optional
        :param color: desired color in rgba format, defaults to [1.0, 0.0, 0.0, 1.0]
        :type color: List[float], optional
        :return: sphere Multibody
        :rtype: Multibody
        """
        sphere = Multibody()
        sphere.InitFromSphere(radius)
        sphere.SetBaseTransformation(pose)
        if self.AddBody(sphere):
            self.SetColor(sphere, color)
            return sphere
        else:
            return None

    def AddCylinder(self, radius: float, height: float, pose: Pose = Pose(), color: List[float] = [1.0, 0.0, 0.0, 1.0]) -> Multibody:
        """Add a cylinder to environment

        :param radius: radius of cylinder
        :type radius: float
        :param height: height of cylinder
        :type height: float
        :param pose: position and rotation of cylinder, defaults to Pose()
        :type pose: Pose, optional
        :param color: desired color in rgba format, defaults to [1.0, 0.0, 0.0, 1.0]
        :type color: List[float], optional
        :return: cylinder Multibody
        :rtype: Multibody
        """
        cylinder = Multibody()
        cylinder.InitFromCylinder(radius, height)
        cylinder.SetBaseTransformation(pose)
        if self.AddBody(cylinder):
            self.SetColor(cylinder, color)
            return cylinder
        else:
            return None

    def AddBox(self, length: float, width: float, height: float, pose: Pose = Pose(), color: List[float] = [1.0, 0.0, 0.0, 1.0]) -> Multibody:
        """Add a box Multibody to environment

        :param length: box length
        :type length: float
        :param width: box width
        :type width: float
        :param height: box height
        :type height: float
        :param pose: box position and rotation, defaults to Pose()
        :type pose: Pose, optional
        :param color: desired color in rgba format, defaults to [1.0, 0.0, 0.0, 1.0]
        :type color: List[float], optional
        :return: Multibody of box
        :rtype: Multibody
        """
        box = Multibody()
        box.InitFromBox(length, width, height)
        box.SetBaseTransformation(pose)
        if self.AddBody(box):
            self.SetColor(box, color)
            return box
        else:
            return None

    def AddCone(self, radius: float, height: float, pose: Pose = Pose(), color: List[float] = [1., 0., 0., 1.]) -> Multibody:
        """Add a cone to environment

        :param radius: radius
        :type radius: float
        :param height: height
        :type height: float
        :param pose: position and roration, defaults to Pose()
        :type pose: Pose, optional
        :param color: color of rgba format, defaults to [1., 0., 0., 1.]
        :type color: List[float], optional
        :return: [description]
        :rtype: Multibody
        """
        cone = Multibody()
        cone.InitFromCone(height, radius)
        cone.SetBaseTransformation(pose)
        if self.AddBody(cone):
            self.SetColor(cone, color)
            return cone
        else:
            return None

    def AddEllipseCone(self, major_axis_top: float, minor_axis_top: float, major_axis_bottom: float,
                       minor_axis_bottom: float, height: float,
                       pose: Pose = Pose(), color: List[float] = [1., 0., 0., 1.]) -> Multibody:
        """Add a ellipse cone to environment

        :param major_axis_top: [description]
        :type major_axis_top: float
        :param minor_axis_top: [description]
        :type minor_axis_top: float
        :param major_axis_bottom: [description]
        :type major_axis_bottom: float
        :param minor_axis_bottom: [description]
        :type minor_axis_bottom: float
        :param height: [description]
        :type height: float
        :param pose: [description], defaults to Pose()
        :type pose: Pose, optional
        :param color: [description], defaults to [1., 0., 0., 1.]
        :type color: List[float], optional
        :return: [description]
        :rtype: Multibody
        """
        ellipse_cone = Multibody()
        ellipse_cone.InitFromEllipseCone(
            height, major_axis_top, minor_axis_top, major_axis_bottom, minor_axis_bottom)
        ellipse_cone.SetBaseTransformation(pose)
        if self.AddBody(ellipse_cone):
            self.SetColor(ellipse_cone, color)
            return ellipse_cone
        else:
            return None

    def GetLinkData(self, link: Link) -> Tuple[bool, LinkData]:
        """Get link data

        :param link: link object
        :type link: Link
        :return: success flag and LinkData
        :rtype: Tuple[bool, LinkData]
        """
        link_data = self.m_links_map.get(link)
        if link_data == None:
            return [False, LinkData()]
        else:
            return [True, link_data]

    def ChooseShowMode(self, body: Multibody = None, mode: int = 0b001) -> bool:
        """Choose show mode of body, visual or collision mode

        :param body: Multibody object, if None, set show mode to all bodies, defaults to None
        :type body: Multibody, optional
        :param mode: 0b001: show visual geometry; 0b010: show collision geometry; 0b100: show link axes. defaults to 0b001
        :type mode: int, optional
        :return: successful or not
        :rtype: bool
        """
        self.m_lock.acquire()
        if not self._EnvironmentSync():
            self.m_lock.release()
            return False
        self.m_lock.release()
        if body == None:
            for link in self.m_links_map:
                self.ChooseShowMode(link, mode)
        elif body.__class__ == RobotModel or body.__class__ == Multibody:
            for link in body.GetLinks():
                self.ChooseShowMode(link, mode)
        else:
            [result, link_data] = self.GetLinkData(body)
            if not result:
                self.m_logger.error("{} not in scene".format(body.GetName()))
                return
            link_data.m_display_mode = mode
            if link_data.m_visual_geometry_handle:
                if mode & 0b001:
                    self.m_view.Show(link_data.m_visual_geometry_handle)
                else:
                    self.m_view.Hide(link_data.m_visual_geometry_handle)
            if link_data.m_collision_geometry_handle:
                if mode & 0b010:
                    self.m_view.Show(link_data.m_collision_geometry_handle)
                else:
                    self.m_view.Hide(link_data.m_collision_geometry_handle)
            if mode & 0b100 and link_data.m_axes_handle:
                self.m_view.Show(link_data.m_axes_handle)
            else:
                self.m_view.Hide(link_data.m_axes_handle)

    def PlotFrame(self, pose: Pose, axis_len: float = 0.2, axis_size: float = 2) -> Vis.Handle:
        """Plot a frame

        :param pose: position and orientation of frame
        :type pose: Pose
        :param axis_len: length of axis, defaults to 0.2
        :type axis_len: float, optional
        :param axis_size: size of axis, defaults to 2
        :type axis_size: float, optional
        :return: Handle of frame
        :rtype: Vis.Handle
        """
        return self.m_view.Axes(pose.GetR3().Coeffs(), pose.GetSO3().Coeffs(), axis_len, axis_size)

    def Delete(self, h: Vis.Handle) -> bool:
        """Delete a handle or handles

        :param h: handle or handles to be deleted
        :type h: Vis.Handle
        :return: [description]
        :rtype: bool
        """
        return self.m_view.Delete(h)

    def PlotGeometry(self, geometry: Geometry, pose: Pose = Pose(), color: List[float] = [1, 0, 0]) -> Vis.Handle:
        """Plot a geometry

        :param geometry: shapes to be plotted
        :type geometry: Geometry
        :param pose: 3D pose, defaults to Pose.IdentityStatic()
        :type pose: Pose, optional
        :param color: (r,g,b) color, defaults to [1, 0, 0]
        :type color: List[float], optional
        :return: handle of plotted geometry
        :rtype: Vis.Handle
        """
        geometry_pose = pose*geometry.pose
        if geometry.type == GeometryType_Sphere:
            handle = self.m_view.Sphere(
                geometry_pose.GetR3().Coeffs(), geometry.radius, color)
            return handle
        elif geometry.type == GeometryType_Mesh:
            mesh = geometry.Copy()
            mesh.ApplyTransformation(geometry_pose)
            vertices_list = np.array(mesh.GetVertices()).flatten().tolist()
            triangles_list = np.array(mesh.GetTriangles()).flatten().tolist()
            cond1 = len(mesh.GetVerticesColor()) != 0
            cond2 = len(mesh.GetVerticesColor()) == len(mesh.GetVertices())
            if cond1 and cond2:
                vertices_color = np.asarray(mesh.GetVerticesColor())[:, :3]
                vertices_color = vertices_color.flatten().tolist()
                color = vertices_color
            handle = self.m_view.Mesh(
                vertices_list, triangles_list, color)
            return handle
        elif geometry.type == GeometryType_Box:
            handle = self.m_view.Box(
                geometry_pose.GetR3().Coeffs(), [geometry.length/2, geometry.width/2, geometry.height/2], color)
            self.m_view.SetTransform(
                handle, geometry_pose.GetR3().Coeffs(), geometry_pose.GetSO3().Coeffs())
            return handle
        elif geometry.type == GeometryType_Cylinder:
            handle = self.m_view.Cylinder(
                geometry_pose.GetR3().Coeffs(), geometry.radius, geometry.height, color)
            self.m_view.SetTransform(
                handle, geometry_pose.GetR3().Coeffs(), geometry_pose.GetSO3().Coeffs())
            return handle
        elif geometry.type == GeometryType_OcTree:
            pass

    def StartDragging(self, drag_body: Multibody) -> bool:
        """Start to drag a Multibody or Manipulator

        :param drag_body: if Multibody, drag applied to base transformation; if Manipulator, to its TCP pose
        :type drag_body: Multibody
        :return: setup successful or not
        :rtype: bool
        """
        if self.m_is_dragging:
            self.m_logger.error("{0} is under dragging mode, start drag mode for {1} failed".format(
                self.m_drag_body.GetName(), drag_body.GetName()))
            return False
        self.m_drag_body = drag_body
        if type(drag_body) == Manipulator:
            manipulator = drag_body
            robot_model = manipulator.GetRobotModel()
            if self.m_env.GetRobotModel(robot_model.GetUniqueName()) is not robot_model:
                self.m_logger.error("Manipulator {0} is not in this environment, start drag mode failed".format(
                    manipulator.GetName()))
                return False
            if manipulator.GetArmController() is None:
                self.m_drag_kin_solver = CreateKinSolver(manipulator)
            elif type(manipulator.GetArmController()) is not SimController:
                self.m_logger.error("Manipulator {0} is not controlled by SimController, start drag mode failed".format(
                    manipulator.GetName()))
                return False
            else:
                self.m_drag_kin_solver = manipulator.GetArmController().GetKinSolver()

            init_pose = robot_model.GetBaseTransformation() * manipulator.GetTipPose()
        elif (type(drag_body) == Multibody) or (type(drag_body) == RobotModel) or (type(drag_body) == EndEffector):
            if self.m_env.GetBody(drag_body.GetUniqueName()) is not drag_body:
                self.m_logger.error("Multibody {0} is not in this environment, start drag mode failed".format(
                    drag_body.GetName()))
                return False
            if drag_body.IsAttached():
                self.m_logger.error("EndEffector {0} is attached, start drag mode failed".format(
                    drag_body.GetName()))
                return False
            init_pose = drag_body.GetBaseTransformation()
        else:
            return False

        self.m_drag_handle = self.m_view.Axes(
            [init_pose[0], init_pose[1], init_pose[2]], [init_pose[3], init_pose[4], init_pose[5], init_pose[6]], 0.2, 2)
        self.m_view.EnableGizmo(self.m_drag_handle, 4)
        self.m_view.SetGizmoDisplayScale(self.m_display_scale)
        self.m_drag_thread = threading.Thread(
            target=self._UpdateDragging, daemon=True)
        self.m_is_dragging = True
        self.m_drag_thread.start()
        return True

    def StopDragging(self) -> bool:
        """Stop dragging mode

        :return: [description]
        :rtype: bool
        """
        if not self.m_is_dragging:
            return False
        self.m_is_dragging = False
        self.m_drag_kin_solver = None
        self.m_drag_body = None
        self.m_drag_thread.join()
        self.m_view.DisableGizmo()
        self.m_view.Delete(self.m_drag_handle)
        return True

    def _EnvironmentSync(self):
        for frame in self.m_env.GetSceneGraph().GetFrames():
            link = frame.GetUserData()
            if not link == None and not link in self.m_links_map:
                if not self._LoadLink(link):
                    self.m_logger.error(
                        "Load link {0} fail".format(link.GetName()))
                    return False
        remove_link_list = []
        for link in self.m_links_map:
            frame_name = link.GetMultibody().GetUniqueName()+"/"+link.GetName()
            if not self.m_env.GetSceneGraph().HasFrame(frame_name):
                remove_link_list.append(link)
        for link in remove_link_list:
            if not self._RemoveLink(link):
                self.m_logger.error(
                    "Remove link {0} fail".format(link.GetName()))
                return False
        return True

    def _LoadLink(self, link):
        link_data = LinkData()
        pose = link.GetPose()
        link_data.m_axes_handle = self.m_view.Axes(
            pose.GetR3().Coeffs(), pose.GetSO3().Coeffs(), 0.1, 5)
        [res, visual_handle] = self._UpdateLinkVisualHandle(link)
        if not res:
            return False
        link_data.m_visual_geometry_handle = visual_handle
        [res, collision_handle] = self._UpdateLinkCollisionHandle(link)
        if not res:
            return False
        link_data.m_collision_geometry_handle = collision_handle
        self.m_links_map[link] = link_data
        if link_data.m_visual_geometry_handle:
            self.m_view.Show(link_data.m_visual_geometry_handle)
        if link_data.m_collision_geometry_handle:
            self.m_view.Hide(link_data.m_collision_geometry_handle)
        if link_data.m_axes_handle:
            self.m_view.Hide(link_data.m_axes_handle)
        return True

    def _RemoveLink(self, link):
        # print(link.GetName())
        [res, link_data] = self.GetLinkData(link)
        if not res:
            return False
        if link_data.m_axes_handle:
            if self.m_view.IsAlive(link_data.m_axes_handle):
                self.m_view.Delete(link_data.m_axes_handle)
        if link_data.m_visual_geometry_handle:
            if self.m_view.IsAlive(link_data.m_visual_geometry_handle):
                self.m_view.Delete(link_data.m_visual_geometry_handle)
        if link_data.m_collision_geometry_handle:
            if self.m_view.IsAlive(link_data.m_collision_geometry_handle):
                self.m_view.Delete(link_data.m_collision_geometry_handle)
        self.m_links_map.pop(link, None)
        return True

    def _Show(self, handles):
        for i in range(len(handles)):
            if self.m_view.IsAlive(handles[i]):
                self.m_view.Show(handles[i])

    def _Hide(self, handles):
        for i in range(len(handles)):
            if self.m_view.IsAlive(handles[i]):
                self.m_view.Hide(handles[i])

    def _Update(self):
        while self.is_update:
            self.m_lock.acquire()
            if not self._EnvironmentSync():
                self.is_update = False
                break
            update_handles = []
            update_trans = []
            update_quat = []
            for link in self.m_links_map:
                [res, link_data] = self.GetLinkData(link)
                if not res:
                    continue
                link_pose = link.GetPose()
                if link_data.m_display_mode & 0b001 and link_data.m_visual_geometry_handle:
                    visual_geometry = link.GetVisualGeometry()
                    render_pose = link_pose * visual_geometry.pose
                    update_trans.append(
                        render_pose.GetR3().Coeffs())
                    update_quat.append(
                        render_pose.GetSO3().Coeffs())
                    update_handles = update_handles + \
                        [link_data.m_visual_geometry_handle]
                if link_data.m_display_mode & 0b010 and link_data.m_collision_geometry_handle:
                    collision_gemetry = link.GetCollisionGeometry()
                    collision_pose = link_pose * collision_gemetry.pose
                    update_trans.append(
                        collision_pose.GetR3().Coeffs())
                    update_quat.append(
                        collision_pose.GetSO3().Coeffs())
                    update_handles = update_handles + \
                        [link_data.m_collision_geometry_handle]
                if link_data.m_display_mode & 0b100 and link_data.m_axes_handle:
                    update_trans.append(
                        link_pose.GetR3().Coeffs())
                    update_quat.append(
                        link_pose.GetSO3().Coeffs())
                    update_handles = update_handles+[link_data.m_axes_handle]
            if len(update_handles) > 0:
                if not self.m_view.SetTransforms(update_handles, update_trans, update_quat):
                    self.m_logger.error(
                        "viewer SetTransforms fail because handles are not exist")
            self.m_lock.release()
            time.sleep(self.m_update_rate)

    def _UpdateLinkCollisionHandle(self, link):
        collision_gemetry = link.GetCollisionGeometry()
        collision_pose = link.GetPose() * collision_gemetry.pose
        link_collision_handle = None
        if(collision_gemetry.type == GeometryType_Mesh):
            vertices_list = np.array(
                collision_gemetry.GetVertices()).flatten().tolist()
            triangles_list = np.array(
                collision_gemetry.GetTriangles()).flatten().tolist()
            if(len(collision_gemetry.material.color) == 3 or len(collision_gemetry.material.color) == 4):
                color = collision_gemetry.material.color
            else:
                color = [1, 1, 1, 1]
            link_collision_handle = self.m_view.Mesh(
                vertices_list, triangles_list, color)
            self.m_view.SetTransform(link_collision_handle, collision_pose.GetR3(
            ).Coeffs(), collision_pose.GetSO3().Coeffs())
        elif(collision_gemetry.type == GeometryType_Box):
            link_collision_handle = self.m_view.Box(collision_pose.GetR3().Coeffs(), [
                collision_gemetry.length/2, collision_gemetry.width/2, collision_gemetry.height/2], [1, 0, 0])
        elif(collision_gemetry.type == GeometryType_Cylinder):
            link_collision_handle = self.m_view.Cylinder(collision_pose.GetR3().Coeffs(
            ), collision_gemetry.radius, collision_gemetry.height, [1, 0, 0])
        elif(collision_gemetry.type == GeometryType_Sphere):
            link_collision_handle = self.m_view.Sphere(collision_pose.GetR3(
            ).Coeffs(), collision_gemetry.radius, [1, 0, 0])
        elif(collision_gemetry.type == GeometryType_OcTree):
            link_collision_handle = self.m_view.Point(
                collision_gemetry.point_cloud, 1, [1, 0, 0])
        return [True, link_collision_handle]

    def _UpdateLinkVisualHandle(self, link):
        visual_geometry = link.GetVisualGeometry()
        link_visual_handle = None
        render_pose = link.GetPose() * visual_geometry.pose
        if(visual_geometry.type == GeometryType_Mesh):
            temp_vertex_list = visual_geometry.GetVertices()
            vertex_list = np.array(temp_vertex_list).flatten().tolist()
            triangle_list = np.array(
                visual_geometry.GetTriangles()).flatten().tolist()
            temp_color_list = visual_geometry.GetVerticesColor()
            vertex_color_list = np.array(temp_color_list)
            if len(vertex_color_list) != 0:
                vertex_color_list = vertex_color_list[:, :3].flatten().tolist()
            color = []
            if len(temp_vertex_list) == len(temp_color_list):
                color = vertex_color_list
            color_from_rvdf = visual_geometry.material.color
            color_from_rvdf = np.asarray(color_from_rvdf)
            if color_from_rvdf.size != 0:
                if len(temp_color_list) != 0:
                    if temp_color_list.count(temp_color_list[0]) == len(temp_color_list):
                        color = color_from_rvdf
            if len(color) == 0:
                color = [1, 1, 1, 1]
            link_visual_handle = self.m_view.Mesh(
                vertex_list, triangle_list, color)
            self.m_view.SetTransform(
                link_visual_handle, render_pose.GetR3().Coeffs(), render_pose.GetSO3().Coeffs())

        elif(visual_geometry.type == GeometryType_Box):
            link_visual_handle = self.m_view.Box(render_pose.GetR3().Coeffs(), [
                visual_geometry.length/2, visual_geometry.width/2, visual_geometry.height/2], [1, 0, 0])
        elif(visual_geometry.type == GeometryType_Cylinder):
            link_visual_handle = self.m_view.Cylinder(render_pose.GetR3().Coeffs(
            ), visual_geometry.radius, visual_geometry.height, [1, 0, 0])
        elif(visual_geometry.type == GeometryType_Sphere):
            link_visual_handle = self.m_view.Sphere(render_pose.GetR3().Coeffs(),
                                                    visual_geometry.radius, [1, 0, 0])
        elif(visual_geometry.type == GeometryType_OcTree):
            link_visual_handle = self.m_view.Point(
                visual_geometry.point_cloud, 1, [1, 0, 0])
        return [True, link_visual_handle]

    def _ToList(self, handles):
        l = []
        for h in handles:
            l += h
        return l

    def _UpdateDragging(self):
        while self.m_is_dragging:
            self.m_lock.acquire()
            [ret, trans, quat] = self.m_view.GetTransform(self.m_drag_handle)
            if ret:
                new_pose = Pose(trans[0], trans[1], trans[2],
                                quat[0], quat[1], quat[2], quat[3])
                if type(self.m_drag_body) == Manipulator:
                    target_pose = self.m_drag_body.GetRobotModel(
                    ).GetBaseTransformation().Inverse() * new_pose

                    current_q = self.m_drag_body.GetDoFPositions()
                    res, ik_result, dist = self.m_drag_kin_solver.GetNearestIK(
                        target_pose, current_q)

                    if res == RVSReturn_Success:
                        self.m_drag_body.SetDoFPositions(ik_result[0])
                    else:
                        old_pose = self.m_drag_body.GetRobotModel().GetBaseTransformation() * \
                            self.m_drag_body.GetTipPose()
                        self.m_view.SetTransform(
                            self.m_drag_handle, old_pose.GetR3().Coeffs(), old_pose.GetSO3().Coeffs())
                elif (type(self.m_drag_body) == Multibody) or (type(self.m_drag_body) == RobotModel) or (type(self.m_drag_body) == EndEffector):
                    if not self.m_drag_body.IsAttached():
                        self.m_drag_body.SetBaseTransformation(new_pose)
                else:
                    self.m_logger.trace("Invalid drag body type")

            self.m_lock.release()
            time.sleep(self.m_update_rate)

    def Exit(self):
        self.is_update = False
        self.m_threads.join()
        if not self.m_view.IsClosed():
            self.m_view.Close()
