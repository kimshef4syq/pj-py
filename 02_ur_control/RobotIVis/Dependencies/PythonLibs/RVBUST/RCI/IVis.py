#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Copyright (c) RVBUST, Inc - All rights reserved.
import pathlib
import sys
import time
from threading import Thread
from typing import Dict, List, Union

import numpy as np
from IPython import embed
from pivy import coin
from pivy.quarter import QuarterWidget
from PySide2.QtWidgets import QApplication
from RVBUST import RCI

try:
    from IViewer import IViewer
except ImportError:
    from .IViewer import IViewer

logger = RCI.RichLogger("IVis")
logger.setLevelForConsole("INFO")
np.set_printoptions(precision=4, suppress=True)


def SoTransformToPose(transform: coin.SoTransform) -> RCI.Pose:
    trans = list(transform.translation.getValue())
    quat = list(transform.rotation.getValue().getValue())
    return RCI.Pose(trans, quat)


def PoseToSoTransform(pose: RCI.Pose, transform: Union[None, coin.SoTransform] = None) -> coin.SoTransform:
    """Convert RCI.Pose to coin.SoTransform, if transform is not None, update it in place, else create a new instance of SoTransform

    :param pose: [description]
    :type pose: RCI.Pose
    :param transform: [description], defaults to None
    :type transform: Union[None, coin.SoTransform], optional
    :return: [description]
    :rtype: coin.SoTransform
    """
    if transform is None:
        transform = coin.SoTransform()
    transform.translation.setValue(pose.Translation())
    transform.rotation.setValue(pose.Quat())
    return transform


class IVis:
    """Inventor based Vis
    """

    def __init__(self,  background="white", run_in_ipython: bool = False):
        self.iviewer = IViewer(background, title="IVis")

        # add event callbacks
        self.iviewer.evt_node.addEventCallback(
            coin.SoKeyboardEvent.getClassTypeId(), self.__MyKeyBoardEventCB, self.iviewer)

        # add a time sensor
        self.iviewer.timer_sensor = coin.SoTimerSensor(self.__MyTimerCB, None)
        self.iviewer.timer_sensor.setInterval(1/20)
        self.iviewer.timer_sensor.setBaseTime(coin.SbTime(2))
        self.iviewer.timer_sensor.schedule()

        self.env = RCI.Environment()
        self.links_map: Dict[RCI.Link, coin.SoShapeKit] = {}  # link: shape

        self.dragger = None

    ########### Basic shape drawing ###############################
    def SetTransform(self, shape: coin.SoShapeKit, position: Union[RCI.Pose, np.ndarray] = np.array([0., 0., 0.]),
                     quat: np.ndarray = np.array([0., 0., 0., 1.])) -> bool:
        """Set a shape transformation, paramter could be position and quat, or just pose

        :param shape: [description]
        :type shape: coin.SoShapeKit
        :param position: [description], defaults to np.array([0., 0., 0.])
        :type position: np.ndarray, optional
        :param quat: [description], defaults to np.array([0., 0., 0., 1.])
        :type quat: np.ndarray, optional
        :return: [description]
        :rtype: bool
        """
        if not isinstance(shape, coin.SoShapeKit):
            logger.error("input pramater shape is not SoShapeKit")
            return False

        if isinstance(position, RCI.Pose):
            transform: coin.SoTransform = shape.getPart("transform", True)
            transform.translation.setValue(*position.Translation())
            transform.rotation.setValue(*position.Quat())
        else:
            transform: coin.SoTransform = shape.getPart("transform", True)
            transform.translation.setValue(*position)
            transform.rotation.setValue(*quat)
        return True

    def SetColor(self, body: Union[RCI.Multibody, coin.SoShapeKit], color: List[float]) -> bool:
        if isinstance(body, coin.SoShapeKit):
            shape = body
            real_shape = shape.getPart("shape", True)
            if isinstance(real_shape, coin.SoIndexedTriangleStripSet):
                vert_prop = real_shape.vertexProperty.getValue()
                vert_prop.orderedRGBA.set1Value(
                    0, coin.SbColor(color[:3]).getPackedValue())
                vert_prop.materialBinding = coin.SoMaterialBinding.OVERALL
            else:
                shape.set("material {diffuseColor %s %s %s}" %
                          (color[0], color[1], color[2]))
                if len(color) >= 4:
                    shape.set("material {transparency %s}" % color[3])
            return True
        else:
            links = body.GetLinks()
            res = False
            for link in links:
                if link in self.links_map:
                    shape = self.links_map[link]
                    res = self.SetColor(shape, color)
            return res

    def AddBody(self, body: RCI.Multibody) -> bool:
        if not self.env.AddBody(body):
            logger.error("failed to add body to environment")
            return False
        links = body.GetLinks()
        body_name = body.GetName()
        for link in links:
            self.__LoadLink(link, name=body_name+"_"+link.GetName())
        return True

    def RemoveBody(self, body: RCI.Multibody) -> bool:
        if not self.env.RemoveBody(body):
            return False
        links = body.GetLinks()
        for link in links:
            self.__RemoveLink(link)
        return True

    def PlotFrame(self, pose: RCI.Pose = RCI.Pose(), length: float = 0.1, line_width: float = 1) -> coin.SoShapeKit:
        return self.iviewer.Axes(positions=pose.Translation(), quats=pose.Quat(), length=length, line_width=line_width)

    def PlotGeometry(self, geometry: RCI.Geometry, pose: RCI.Pose = RCI.Pose(), color: List[float] = [1, 0, 0], name: str = None) -> coin.SoShapeKit:
        """Plot a geometry

        :param geometry: shapes to be plotted
        :type geometry: Geometry
        :param pose: 3D pose, defaults to Pose.IdentityStatic()
        :type pose: Pose, optional
        :param color: (r,g,b) color, defaults to [1, 0, 0]
        :type color: List[float], optional
        :return: handle of plotted geometry
        :rtype: coin.SoShapeKit
        """
        shape = None
        geometry_pose = pose*geometry.pose
        if geometry.type == RCI.GeometryType_Mesh:
            verts = np.asarray(geometry.GetVertices())
            indicies = np.asarray(geometry.GetTriangles())
            colors = np.asarray(geometry.GetVerticesColor())
            user_defined_colors = geometry.material.color
            if len(user_defined_colors) > 0:
                colors = user_defined_colors
                colors[-1] = 1 - colors[-1]
            #  temporarily set to grey to load UR10
            else:
                colors = [0.5, 0.5, 0.5, 0]
            shape = self.iviewer.TriMesh(verts, indicies, colors, name=name)
            self.SetTransform(shape, pose.Translation(), pose.Quat())
            if len(geometry.material.diffuse) >= 3:
                shape.set("material {diffuseColor %s %s %s}" % (
                    geometry.material.diffuse[0], geometry.material.diffuse[1], geometry.material.diffuse[2]))
            if len(geometry.material.ambient) >= 3:
                shape.set("material {ambientColor %s %s %s}" % (
                    geometry.material.ambient[0], geometry.material.ambient[1], geometry.material.ambient[2]))
            shape.set("material {shininess %s}" % geometry.material.shininess)
            self.SetTransform(shape, geometry_pose)
        elif geometry.type == RCI.GeometryType_Sphere:
            logger.info(f"Plot sphere geometry")
            shape = self.iviewer.Sphere(
                geometry_pose.Translation(), geometry.radius, color, name=name)
        elif geometry.type == RCI.GeometryType_Box:
            logger.info(f"Plot box geometry")
            shape = self.iviewer.Box(positions=geometry_pose.Translation(), quats=geometry_pose.Quat(),
                                     length=geometry.length, width=geometry.width, height=geometry.height, colors=color, name=name)
        elif geometry.type == RCI.GeometryType_Cylinder:
            logger.info(f"Plot cylinder geometry")
            shape = self.iviewer.Cylinder(geometry_pose.Translation(), bottom_radius=geometry.radius,
                                          height=geometry.height, colors=color, name=name)
        elif geometry.type == RCI.GeometryType_OcTree:
            logger.info(f"Plot OcTree geometry")
            shape = self.iviewer.Point(points=np.asarray(geometry.point_cloud).reshape(-1, 3),
                                       colors=color, point_size=2, name=name)
        else:
            logger.warning(f"unknown geometry type: {geometry.type}")
        return shape

    def AddSphere(self, radius: float, pose: RCI.Pose = RCI.Pose(), color: List[float] = [1.0, 0.0, 0.0, 1.0]) -> RCI.Multibody:
        """Add a sphere to environment

        :param radius: sphere radius
        :type radius: float
        :param pose: sphere position and rotation, defaults toRCI.Pose()
        :type pose: Pose, optional
        :param pose: sphere position and rotation, defaults toRCI.Pose()
        :type pose: Pose, optional
        :param color: desired color in rgba format, defaults to [1.0, 0.0, 0.0, 1.0]
        :type color: List[float], optional
        :return: sphereRCI.Multibody
        :rtype:RCI.Multibody
        """
        sphere = RCI.Multibody()
        sphere.InitFromSphere(radius)
        sphere.SetBaseTransformation(pose)
        if self.AddBody(sphere):
            self.SetColor(sphere, color)
            return sphere
        else:
            return None

    def AddCylinder(self, radius: float, height: float, pose: RCI.Pose = RCI.Pose(), color: List[float] = [1.0, 0.0, 0.0, 1.0]) -> RCI.Multibody:
        """Add a cylinder to environment

        :param radius: radius of cylinder
        :type radius: float
        :param height: height of cylinder
        :type height: float
        :param pose: position and rotation of cylinder, defaults toRCI.Pose()
        :type pose: Pose, optional
        :param color: desired color in rgba format, defaults to [1.0, 0.0, 0.0, 1.0]
        :type color: List[float], optional
        :return: cylinderRCI.Multibody
        :rtype:RCI.Multibody
        """
        cylinder = RCI.Multibody()
        cylinder.InitFromCylinder(radius, height)
        cylinder.SetBaseTransformation(pose)
        if self.AddBody(cylinder):
            self.SetColor(cylinder, color)
            return cylinder
        else:
            return None

    def AddBox(self, length: float, width: float, height: float, pose: RCI.Pose = RCI.Pose(), color: List[float] = [1.0, 0.0, 0.0, 1.0]) -> RCI.Multibody:
        """Add a boxRCI.Multibody to environment

        :param length: box length
        :type length: float
        :param width: box width
        :type width: float
        :param height: box height
        :type height: float
        :param pose: box position and rotation, defaults toRCI.Pose()
        :type pose: Pose, optional
        :param color: desired color in rgba format, defaults to [1.0, 0.0, 0.0, 1.0]
        :type color: List[float], optional
        :return:RCI.Multibody of box
        :rtype: RCI.Multibody
        """
        box = RCI.Multibody()
        box.InitFromBox(length, width, height)
        box.SetBaseTransformation(pose)
        if self.AddBody(box):
            self.SetColor(box, color)
            return box
        else:
            return None

    def AddCone(self, radius: float, height: float, pose: RCI.Pose = RCI.Pose(), color: List[float] = [1., 0., 0., 1.]) -> RCI.Multibody:
        """Add a cone to environment

        :param radius: radius
        :type radius: float
        :param height: height
        :type height: float
        :param pose: position and roration, defaults toRCI.Pose()
        :type pose: Pose, optional
        :param color: color of rgba format, defaults to [1., 0., 0., 1.]
        :type color: List[float], optional
        :return: [description]
        :rtype: RCI.Multibody
        """
        cone = RCI.Multibody()
        cone.InitFromCone(height, radius)
        cone.SetBaseTransformation(pose)
        if self.AddBody(cone):
            self.SetColor(cone, color)
            return cone
        else:
            return None

    def AddEllipseCone(self, major_axis_top: float, minor_axis_top: float, major_axis_bottom: float,
                       minor_axis_bottom: float, height: float,
                       pose: RCI.Pose = RCI.Pose(), color: List[float] = [1., 0., 0., 1.]) -> RCI.Multibody:
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
        :param pose: [description], defaults toRCI.Pose()
        :type pose: Pose, optional
        :param color: [description], defaults to [1., 0., 0., 1.]
        :type color: List[float], optional
        :return: [description]
        :rtype:RCI.Multibody
        """
        ellipse_cone = RCI.Multibody()
        ellipse_cone.InitFromEllipseCone(
            height, major_axis_top, minor_axis_top, major_axis_bottom, minor_axis_bottom)
        ellipse_cone.SetBaseTransformation(pose)
        if self.AddBody(ellipse_cone):
            self.SetColor(ellipse_cone, color)
            return ellipse_cone
        else:
            return None

    def DrawTraj(self, traj: Union[RCI.TrajectoryBaseRn, RCI.TrajectoryBaseSE3],
                 kin_solver: RCI.KinematicsBase = None, length: float = 0.05, line_width: int = 1, points_num: int = 100) -> coin.SoSeparator:
        if traj is None:
            return
        if not traj.IsValid():
            logger.error(f"traj [{traj}] is not valid!")
            return None
        d = traj.GetDuration()
        ts = np.linspace(0, d, points_num)
        positions = np.asarray([traj.GetPosition(t).Coeffs() for t in ts])
        if isinstance(traj, RCI.TrajectoryBaseRn):
            positions = np.asarray([kin_solver.GetPositionFK(
                q)[1].Coeffs() for q in positions])
        axes = self.iviewer.Axes(positions=positions[:, :3], quats=positions[:, 3:7],
                                 length=length, line_width=line_width, name="TrajectoryAxes")
        self.iviewer.root.removeChild(axes)
        lines = self.iviewer.Lines(positions[:, :3], colors=(
            0, 0, 0), line_width=line_width, name="TrajectoryLines")
        self.iviewer.root.removeChild(lines)
        shape = coin.SoSeparator()
        shape.addChild(lines)
        shape.addChild(axes)
        self.iviewer.root.addChild(shape)
        return shape

    #### Draw geometry interfaces########

    def StartDragger(self, dragger_body: RCI.Multibody):
        if self.dragger is not None:
            logger.info("already in dragger mode!")
            return
        if type(dragger_body) == RCI.Manipulator:
            manipulator = dragger_body
            robot_model = manipulator.GetRobotModel()
            if self.env.GetRobotModel(robot_model.GetUniqueName()) is not robot_model:
                logger.error("Manipulator {0} is not in this environment, start drag mode failed".format(
                    manipulator.GetName()))
                return False
            if manipulator.GetArmController() is None:
                self.dragger_kin_solver = RCI.CreateKinSolver(manipulator)
            elif type(manipulator.GetArmController()) is not RCI.SimController:
                logger.error("Manipulator {0} is not controlled by SimController, start drag mode failed".format(
                    manipulator.GetName()))
                return False
            else:
                self.dragger_kin_solver = manipulator.GetArmController().GetKinSolver()
            init_pose = robot_model.GetBaseTransformation() * manipulator.GetTipPose()

        elif (type(dragger_body) == RCI.Multibody) or (type(dragger_body) == RCI.RobotModel) or (type(dragger_body) == RCI.EndEffector):
            if self.env.GetBody(dragger_body.GetUniqueName()) is not dragger_body:
                logger.error("Multibody {0} is not in this environment, start drag mode failed".format(
                    dragger_body.GetName()))
                return False
            if dragger_body.IsAttached():
                logger.error("EndEffector {0} is attached, start drag mode failed".format(
                    dragger_body.GetName()))
                return False
            init_pose = dragger_body.GetBaseTransformation()
        elif isinstance(dragger_body, coin.SoShapeKit):
            _transform: coin.SoTransform = dragger_body.getPart(
                "transform", True)
            init_pose = SoTransformToPose(_transform)
        else:
            return False
        self.dragger_body = dragger_body
        # self.path = node.createPathToPart("transform", True)
        # there are many draggers, but none of them is satisfied
        # self.dragger = coin.SoTrackballManip()
        # self.dragger = coin.SoTransformerManip()
        # self.dragger = coin.SoTransformBoxManip()
        # self.dragger = coin.SoJackManip()
        # self.dragger = coin.SoTabBoxManip()
        # self.dragger = coin.SoHandleBoxManip()
        self.dragger = coin.SoCenterballManip()
        self.dragger_node = self.iviewer.Lines(points=np.asarray([(-0.1, 0, 0), (0.1, 0, 0), (0, -0.1, 0), (0, 0.1, 0), (0, 0, -0.1), (0, 0, 0.1)]),
                                               colors=np.asarray([(1, 0, 0), (0, 1, 0), (0, 0, 1)]), line_width=5, num_vertices=(2, 2, 2))
        self.SetTransform(self.dragger_node, init_pose)
        self.path = self.dragger_node.createPathToPart("transform", True)
        # self.dragger.scaleFactor.setValue(3, 3, 3)
        self.dragger.replaceNode(self.path)
        self.is_dragging = True

        self.dragging_thread = Thread(target=self.__UpdateDragging)
        self.dragging_thread.start()

    def StopDragger(self):
        if self.dragger is not None:
            fullpath = coin.SoFullPath.fromSoPath(self.path)
            self.dragger.replaceManip(fullpath, None)
            self.iviewer.DeleteShape(self.dragger_node)
            self.dragger = None
            self.dragger_body = None
            self.is_dragging = False

    def SyncRobotLinkPose(self) -> bool:
        """The function should be called periodically
        """
        logger.debug("start sync")
        for link, shape in self.links_map.items():
            pose = link.GetPose()
            transform: coin.SoTransform = shape.getPart("transform", True)
            transform.translation.setValue(*pose.Translation())
            transform.rotation.setValue(*pose.Quat())
        logger.debug("finish sync")
        return True

    def LoadEnvironment(self, env: RCI.Environment) -> bool:
        links = list(self.links_map.keys())
        for link in links:
            self.__RemoveLink(link)

        self.env = env
        for frame in self.env.GetSceneGraph().GetFrames():
            link = frame.GetUserData()
            if link is not None and not link in self.links_map:
                if not self.__LoadLink(link):
                    logger.warning("Load link {0} fail".format(link.GetName()))
        remove_link_list = []
        for link in self.links_map:
            logger.info(f"check link {link.GetName()}")
            body = link.GetMultibody()
            if body:
                frame_name = body.GetUniqueName()+"/"+link.GetName()
                if not self.env.GetSceneGraph().HasFrame(frame_name):
                    remove_link_list.append(link)
            else:
                remove_link_list.append(link)
        for link in remove_link_list:
            if not self.__RemoveLink(link):
                logger.error("Remove link {0} fail".format(link.GetName()))
        return True

    def GetSelectedShape(self):
        return self.iviewer.selected_shape

    def GetSelectedBody(self):
        return self.GetShapeMultibody(self.GetSelectedShape())

    def GetShapeLink(self, shape: coin.SoShapeKit) -> Union[None, RCI.Link]:
        """Get the link which the shape belongs to

        :param shape: [description]
        :type shape: coin.SoShapeKit
        :return: [description]
        :rtype: Union[None, RCI.Link]
        """
        if shape is not None:
            for link, link_shape in self.links_map.items():
                if link_shape.getName() == shape.getName():
                    return link
        return None

    def GetShapeMultibody(self, shape: coin.SoShapeKit) -> Union[None, RCI.Multibody]:
        """Get theRCI.Multibody which the shape belongs to

        :param shape: [description]
        :type shape: coin.SoShapeKit
        :return: [description]
        :rtype: Union[None, RCI.Multibody]
        """
        link = self.GetShapeLink(shape)
        if link is not None:
            return link.GetMultibody()
        else:
            return None

    def __LoadLink(self, link: RCI.Link, name: str = "Link") -> coin.SoShapeKit:
        logger.info(f"load link {link.GetName()}")
        shape = None
        vg = link.GetVisualGeometry()
        body_name = link.GetMultibody().GetName()
        link_name = link.GetName()
        name = f"{body_name}_{link_name}"
        shape = self.PlotGeometry(
            geometry=vg, pose=link.GetPose(), name=name)
        if shape is not None:
            self.links_map[link] = shape
        return shape

    def __RemoveLink(self, link: RCI.Link) -> bool:
        if link in self.links_map:
            self.iviewer.DeleteShape(self.links_map[link])
            del self.links_map[link]
            return True
        else:
            return False

    def __UpdateDragging(self):
        while self.is_dragging:
            get_matrix_action = coin.SoGetMatrixAction(
                self.iviewer.getSoRenderManager().getViewportRegion())
            get_matrix_action.apply(self.path)
            mat: coin.SbMatrix = get_matrix_action.getMatrix().getValue()
            mat = np.array(mat)
            mat = mat.T
            logger.debug(f"dragger box matrix: {mat}")
            new_pose = RCI.Pose(mat)
            if type(self.dragger_body) == RCI.Manipulator:
                target_pose = self.dragger_body.GetRobotModel(
                ).GetBaseTransformation().Inverse() * new_pose

                current_q = self.dragger_body.GetDoFPositions()
                res, ik_result, dist = self.dragger_kin_solver.GetNearestIK(
                    target_pose, current_q)

                if res == RCI.RVSReturn_Success:
                    self.dragger_body.SetDoFPositions(ik_result[0])
                else:
                    old_pose = self.dragger_body.GetRobotModel().GetBaseTransformation() * \
                        self.dragger_body.GetTipPose()
                    self.SetTransform(
                        self.dragger_node, old_pose.GetR3().Coeffs(), old_pose.GetSO3().Coeffs())
            elif (type(self.dragger_body) == RCI.Multibody) or (type(self.dragger_body) == RCI.RobotModel) or (type(self.dragger_body) == RCI.EndEffector):
                if not self.dragger_body.IsAttached():
                    self.dragger_body.SetBaseTransformation(new_pose)
            elif isinstance(self.dragger_body, coin.SoShapeKit):
                self.SetTransform(self.dragger_body, new_pose)
            else:
                logger.debug("Invalid drag body type")
            time.sleep(0.05)

    ################# Call backs ###################################################

    def __MyKeyBoardEventCB(self, quarter: QuarterWidget, evt_node: coin.SoEventCallback):
        evt: coin.SoKeyboardEvent = evt_node.getEvent()
        # press R to view all scene
        if evt.getKey() == coin.SoKeyboardEvent.R:
            quarter.viewAll()
            evt_node.setHandled()
        # press space to reset to default camera view
        elif evt.getKey() == coin.SoKeyboardEvent.SPACE:
            self.iviewer.HomeView()
            evt_node.setHandled()
        elif evt.getKey() == coin.SoKeyboardEvent.M:
            body = self.GetShapeMultibody(self.selected_node)
            if body:
                self.StartDragger(body)
            else:
                self.StartDragger(self.selected_node)
            evt_node.setHandled()
        elif evt.getKey() == coin.SoKeyboardEvent.N:
            self.StopDragger()
            evt_node.setHandled()
        elif evt.wasCtrlDown() and evt.getKey() == coin.SoKeyboardEvent.EQUAL:
            if self.dragger is not None:
                scale = list(self.dragger.scaleFactor.getValue())
                scale = np.asarray(scale) * 1.1
                self.dragger.scaleFactor.setValue(*scale)
        elif evt.wasCtrlDown() and evt.getKey() == coin.SoKeyboardEvent.MINUS:
            if self.dragger is not None:
                scale = list(self.dragger.scaleFactor.getValue())
                scale = np.asarray(scale) * 0.9
                self.dragger.scaleFactor.setValue(*scale)

    def __MyTimerCB(self, user_data, timer_sensor: coin.SoTimerSensor):
        self.SyncRobotLinkPose()


if __name__ == "__main__":
    DATADIR = pathlib.Path(__file__).parent / "../../Data/"
    run_in_ipython = False
    if len(sys.argv) > 1:
        run_in_ipython = int(sys.argv[1])
    app = QApplication.instance()
    if app is None:
        app = QApplication([])
    ivis = IVis()
    controller = RCI.SimController.Create("Motoman_GP7")
    controller.Connect()
    controller.EnableRobot()
    _, q0 = controller.GetJointPosition()
    q1 = q0 + RCI.RxTangent([0.2, 0.5, -0.6, 0.1, -0.9, 0])
    path = RCI.CreatePath([q0, q1])
    traj = RCI.CreateTrajectory(
        path, RCI.ConvertJointLimits2Arr(controller.GetJointLimits())*0.1)
    robot_model = controller.GetRobotModel()
    ivis.AddBody(robot_model)
    ivis.iviewer.viewAll()
    ivis.iviewer.show()
    if run_in_ipython:
        embed()  # %gui qt
        # controller.ExecuteTrajectory(traj, False)
    else:
        app.exec_()
