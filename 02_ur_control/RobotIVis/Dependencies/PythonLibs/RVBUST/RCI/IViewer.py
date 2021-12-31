#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Copyright (c) RVBUST, Inc - All rights reserved.
import re
import os
import logging
from collections.abc import Iterable
from typing import Dict, List, Union
from scipy.spatial import transform

import numpy as np
from IPython import embed
from pivy import coin
from pivy.quarter import QuarterWidget
from PySide2 import QtGui, QtWidgets

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("IViewer")

np.set_printoptions(precision=4, suppress=True)


class IViewer(QuarterWidget):
    def __init__(self, background="white", title="IViewer", run_in_ipython=False, *args, **kwargs):
        QuarterWidget.__init__(self, *args, **kwargs)
        self.run_in_ipython = run_in_ipython
        self.setBackgroundColor(QtGui.QColor(background))
        self.setWindowTitle("IVis")

        # selection setting
        self.selection_root = coin.SoSelection()
        self.selection_root.policy.setValue(coin.SoSelection.SHIFT)
        self.selection_root.addFinishCallback(self.__MyFinishSelectionCB)

        # camera setting
        self.camera: coin.SoCamera = coin.SoOrthographicCamera()
        self.camera.position.setValue(1, 1, 1)
        self.camera.pointAt(coin.SbVec3f(0, 0, 0), coin.SbVec3f(0, 0, 1))
        self.selection_root.addChild(self.camera)

        # event callback setting
        self.evt_node = coin.SoEventCallback()
        self.evt_node.addEventCallback(coin.SoMouseButtonEvent.getClassTypeId(),
                                       self.__MyMouseEventCB)
        self.selection_root.addChild(self.evt_node)

        # shape scene graph
        self.root = coin.SoSeparator()
        self.root.setName('SceneRoot')
        self.selection_root.addChild(self.root)

        # hight light setting
        self.hightlight_action = coin.SoLineHighlightRenderAction()
        self.hightlight_action.setColor(coin.SbColor(1, 0, 1))
        self.hightlight_action.setLineWidth(3)
        self.getSoRenderManager().setGLRenderAction(self.hightlight_action)

        # Draw a global axes
        self.Axes(positions=(0, 0, 0), quats=(0., 0., 0., 1.),
                  length=0.5, line_width=2, name="Axes_World")

        self.setSceneGraph(self.selection_root)

        self.selected_shape = None
        self.picked_point = None
        self.picked_normal = None

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        self.setVisible(False)
        if self.run_in_ipython:
            logger.debug(
                "running in IPython, close event is replaced with hide event")
            event.ignore()
        else:
            event.accept()

    def HomeView(self):
        camera: coin.SoCamera = self.getSoRenderManager().getCamera()
        camera.position.setValue(1, 1, 1)
        camera.pointAt(coin.SbVec3f(0, 0, 0), coin.SbVec3f(0, 0, 1))
        if camera.getTypeId() == coin.SoPerspectiveCamera.getClassTypeId():
            camera.heightAngle.setValue(np.pi/4)
        elif camera.getTypeId() == coin.SoOrthographicCamera.getClassTypeId():
            camera.height.setValue(2.)
        self.viewAll()

    def FrontView(self):
        camera: coin.SoCamera = self.getSoRenderManager().getCamera()
        camera.position.setValue(1, 0, 0)
        camera.pointAt(coin.SbVec3f(0, 0, 0), coin.SbVec3f(0, 0, 1))
        self.viewAll()

    def BackView(self):
        camera: coin.SoCamera = self.getSoRenderManager().getCamera()
        camera.position.setValue(-1, 0, 0)
        camera.pointAt(coin.SbVec3f(0, 0, 0), coin.SbVec3f(0, 0, 1))
        self.viewAll()

    def LeftView(self):
        camera: coin.SoCamera = self.getSoRenderManager().getCamera()
        camera.position.setValue(0, -1, 0)
        camera.pointAt(coin.SbVec3f(0, 0, 0), coin.SbVec3f(0, 0, 1))
        self.viewAll()

    def RightView(self):
        camera: coin.SoCamera = self.getSoRenderManager().getCamera()
        camera.position.setValue(0, 1, 0)
        camera.pointAt(coin.SbVec3f(0, 0, 0), coin.SbVec3f(0, 0, 1))
        self.viewAll()

    def TopView(self):
        camera: coin.SoCamera = self.getSoRenderManager().getCamera()
        camera.position.setValue(0, 0, 1)
        camera.pointAt(coin.SbVec3f(0, 0, 0), coin.SbVec3f(0, 1, 0))
        self.viewAll()

    def BottomView(self):
        camera: coin.SoCamera = self.getSoRenderManager().getCamera()
        camera.position.setValue(0, 0, -1)
        camera.pointAt(coin.SbVec3f(0, 0, 0), coin.SbVec3f(0, 1, 0))
        self.viewAll()

    def PerspectiveView(self):
        camera: coin.SoCamera = self.getSoEventManager().getCamera()
        if camera.getTypeId() == coin.SoPerspectiveCamera.getClassTypeId():
            return
        camera_persp: coin.SoPerspectiveCamera = coin.SoPerspectiveCamera()
        camera_persp.position.setValue(camera.position.getValue())
        camera_persp.orientation.setValue(camera.orientation.getValue())
        self.getSoEventManager().setCamera(camera_persp)
        self.getSoRenderManager().setCamera(camera_persp)
        self.selection_root.replaceChild(self.camera, camera_persp)
        self.camera = camera_persp

    def OrthogonalView(self):
        camera: coin.SoCamera = self.getSoEventManager().getCamera()
        if camera.getTypeId() == coin.SoOrthographicCamera.getClassTypeId():
            return
        camera_persp: coin.SoOrthographicCamera = coin.SoOrthographicCamera()
        camera_persp.position.setValue(camera.position.getValue())
        camera_persp.orientation.setValue(camera.orientation.getValue())
        self.getSoEventManager().setCamera(camera_persp)
        self.getSoRenderManager().setCamera(camera_persp)
        self.selection_root.replaceChild(self.camera, camera_persp)
        self.camera = camera_persp

    def Zoom(self, ratio: float = 0.95):
        camera: coin.SoCamera = self.getSoEventManager().getCamera()
        if camera.getTypeId() == coin.SoPerspectiveCamera.getClassTypeId():
            angle = camera.heightAngle.getValue()
            camera.heightAngle.setValue(max(0.0, angle*ratio))
        elif camera.getTypeId() == coin.SoOrthographicCamera.getClassTypeId():
            height = camera.height.getValue()
            camera.height.setValue(height*ratio)

    def ZoomIn(self):
        self.Zoom(0.95)

    def ZoomOut(self):
        self.Zoom(1.05)

    def GetSceneDescription(self):
        return "\n".join(self.__GetNodeDescription(self.root, 0))

    def __GetNodeDescription(self, node: coin.SoShapeKit, indent: int = 0) -> List[str]:
        has_child_shape_kit = False
        children_desp = []
        children = node.getChildren()
        if children is not None:
            for child in children:
                if isinstance(child, coin.SoShapeKit):
                    has_child_shape_kit = True
                    name = child.getName()
                    name = re.sub("_\d+$", "", str(name))
                    grand_desp = self.__GetNodeDescription(child, indent+1)
                    children_desp.extend(grand_desp)
        name = node.getName()
        name = re.sub("_\d+$", "", str(name))
        if has_child_shape_kit:
            desp = [f"{'    '*indent}+ {name}"]
        else:
            desp = [f"{'    '*indent}- {name}"]
        children_desp.sort()
        desp.extend(children_desp)
        return desp

    def __MyFinishSelectionCB(self, user_data, selection_node: coin.SoSelection):
        """Redraw should be called after selection changes.

        :param render_manager: [description]
        :type render_manager: SoRenderManager
        """
        event_manager = self.getSoEventManager()
        handle_event_action = event_manager.getHandleEventAction()
        picked_point: coin.SoPickedPoint = handle_event_action.getPickedPoint()
        if picked_point:
            self.picked_point = np.array(list(picked_point.getPoint()))
            self.picked_normal = np.array(list(picked_point.getNormal()))
        else:
            self.picked_point = None
            self.picked_normal = None

        selection_path_list = selection_node.getList()
        if selection_path_list.getLength() > 0:
            selection_path = selection_path_list.get(0)
            self.selected_shape = selection_path.getTail()
        else:
            self.selected_shape = None

    def TriMesh(self, vertices: np.ndarray, indicies: np.ndarray,
                colors: np.ndarray = (0.3, 0.2, 0.5),
                transparency: float = 0, name: str = "TriMesh") -> coin.SoShapeKit:
        shape = coin.SoShapeKit()
        vert_prop = coin.SoVertexProperty()
        vert_prop.vertex.setValues(0, len(vertices), vertices)
        if isinstance(colors[0], Iterable):
            vert_prop.orderedRGBA.setValues(self.__PackColors(colors))
            if len(colors) == len(vertices):
                vert_prop.materialBinding = coin.SoMaterialBinding.PER_VERTEX
            else:
                vert_prop.materialBinding = coin.SoMaterialBinding.PER_FACE
        else:
            colors[-1] = 1 - colors[-1]  # transparency to alpha
            vert_prop.orderedRGBA.set1Value(
                0, coin.SbColor4f(colors).getPackedValue())
            vert_prop.materialBinding = coin.SoMaterialBinding.OVERALL
        facets = coin.SoIndexedTriangleStripSet()
        facets.vertexProperty = vert_prop
        indicies = np.c_[indicies, np.ones(len(indicies), dtype=int) * -1]
        facets.coordIndex.setValues(0, len(indicies.ravel()), indicies.ravel())
        shape.setName(f"{name}_{id(shape)}")
        shape.setPart("shape", facets)
        # shape.set("material {transparency %s}" % transparency)
        # shape.set("material {diffuseColor 1 0 0}")
        # set shape hints to compute the normal directions as we expected
        shape.set("shapeHints {vertexOrdering CLOCKWISE}")  # COUNTERCLOCKWISE
        self.root.addChild(shape)
        return shape

    def Cylinder(self, positions: List[float] = (0, 0, 0), quats: List[float] = (0, 0, 0, 1),
                 bottom_radius: float = 0.1, height: float = 0.2,
                 colors: List[float] = (1, 0, 0), transparency: float = 0.0, name: str = "Cylinder") -> coin.SoShapeKit:
        shape = coin.SoShapeKit()
        shape.setName(f"{name}_{id(shape)}")
        shape.setPart("shape", coin.SoCylinder())
        shape.set("shape {radius %s height %s}" %
                  (bottom_radius, height))
        transform: coin.SoTransform = shape.getPart("transform", True)
        transform.translation.setValue(positions)
        transform.rotation.setValue(quats)
        material: coin.SoMaterial = shape.getPart("material", True)
        material.diffuseColor.setValue(*colors)
        material.transparency.setValue(transparency)
        self.root.addChild(shape)
        return shape

    def Cone(self, positions: List[float] = (0, 0, 0), quats: List[float] = (0, 0, 0, 1),
             bottom_radius: float = 0.1, height: float = 0.2,
             colors: List[float] = (1, 0, 0), transparency: float = 0.0, name: str = "Cone") -> coin.SoShapeKit:
        shape = coin.SoShapeKit()
        shape.setName(f"{name}_{id(shape)}")
        shape.setPart("shape", coin.SoCone())
        shape.set("shape {bottomRadius %s height %s}" %
                  (bottom_radius, height))
        transform: coin.SoTransform = shape.getPart("transform", True)
        transform.translation.setValue(positions)
        transform.rotation.setValue(quats)
        material: coin.SoMaterial = shape.getPart("material", True)
        material.diffuseColor.setValue(*colors)
        material.transparency.setValue(transparency)
        self.root.addChild(shape)
        return shape

    def Box(self, positions: List[float] = (0, 0, 0), quats: List[float] = (0, 0, 0, 1),
            length: float = 0.1, width: float = 0.1, height: float = 0.1,
            colors: List[float] = (1, 0, 0), transparency: float = 0.0, name: str = "Box") -> coin.SoShapeKit:
        shape = coin.SoShapeKit()
        shape.setName(f"{name}_{id(shape)}")
        shape.setPart("shape", coin.SoCube())
        shape.set("shape {width %s height %s depth %s}" %
                  (length, width, height))
        transform: coin.SoTransform = shape.getPart("transform", True)
        transform.translation.setValue(positions)
        transform.rotation.setValue(quats)
        material: coin.SoMaterial = shape.getPart("material", True)
        material.diffuseColor.setValue(*colors)
        material.transparency.setValue(transparency)
        self.root.addChild(shape)
        return shape

    def Sphere(self, positions: List[float] = np.zeros(3), radius: float = 0.01,
               colors: List[float] = (1, 0, 0), transparency: float = 0.0, name: str = "Sphere") -> coin.SoShapeKit:
        """Draw a sphere

        :param positions: sphere center positions
        :type positions: float, optional
        :param radius: sphere radius, defaults to 0.01
        :type radius: float, optional
        :param color: sphere color, defaults to (1, 0, 0)
        :type color: Tuple[float], optional
        :param transparency: transparency of sphere, defaults to 0.0
        :type transparency: float, optional
        :return: shape
        :rtype: SoShapeKit
        """
        sphere = coin.SoShapeKit()
        sphere.setName(f"{name}_{id(sphere)}")
        sphere.setPart("shape", coin.SoSphere())
        sphere.set("shape {radius %s}" % radius)
        material: coin.SoMaterial = sphere.getPart("material", True)
        material.diffuseColor.setValue(*colors)
        material.transparency.setValue(transparency)
        transform: coin.SoTransform = sphere.getPart("transform", True)
        transform.translation.setValue(positions)
        self.root.addChild(sphere)
        return sphere

    def Point(self, points: np.ndarray, colors: List[float] = (1., 0., 0.), point_size: float = 2.0, name: str = "Points") -> coin.SoShapeKit:
        vert_prop = coin.SoVertexProperty()
        vert_prop.vertex.setValues(0, len(points), points)
        if isinstance(colors[0], Iterable):
            vert_prop.orderedRGBA.setValues(
                0, len(colors), self.__PackColors(colors))
            vert_prop.materialBinding = coin.SoMaterialBinding.PER_PART
        else:
            vert_prop.orderedRGBA.set1Value(
                0, coin.SbColor(colors).getPackedValue())
            vert_prop.materialBinding = coin.SoMaterialBinding.OVERALL
        pcd = coin.SoPointSet()
        pcd.vertexProperty = vert_prop
        shape = coin.SoShapeKit()
        shape.setName(f"{name}_{id(shape)}")
        shape.setPart("shape", pcd)
        shape.set("drawStyle {pointSize %s}" % point_size)
        self.root.addChild(shape)
        return shape

    def Axes(self, positions: Union[List[float], List[List[float]]] = (0, 0, 0),
             quats: Union[List[float], List[List[float]]] = (0, 0, 0, 1),
             length: float = 0.1, line_width: int = 3, name: str = "Axes") -> coin.SoShapeKit:
        """Draw one or many axes

        :param positions: [description], defaults to (0, 0, 0)
        :type positions: Union[List[float], List[List[float]]], optional
        :param quats: [description], defaults to (0, 0, 0, 1)
        :type quats: Union[List[float], List[List[float]]], optional
        :param length: [description], defaults to 0.1
        :type length: float, optional
        :param line_width: [description], defaults to 3
        :type line_width: int, optional
        :return: [description]
        :rtype: SoShapeKit
        """
        points = np.c_[np.zeros((3, 3)), np.eye(3)*length].reshape(-1, 3)
        colors = np.eye(3)
        num_vertices = (2, 2, 2)
        if isinstance(positions[0], Iterable):
            num_axes = len(positions)
            points = np.tile(points, (num_axes, 1))
            # apply rotations
            points[1::6] = transform.Rotation.from_quat(
                quats).apply(points[1::6])
            points[3::6] = transform.Rotation.from_quat(
                quats).apply(points[3::6])
            points[5::6] = transform.Rotation.from_quat(
                quats).apply(points[5::6])
            # apply translations
            points = points.reshape(num_axes, -1, 3) + \
                np.asarray(positions).reshape(num_axes, -1, 3)
            points.shape = (-1, 3)
            num_vertices = np.repeat(2, num_axes*3)
            colors = np.tile(np.eye(3), (num_axes, 1))
        else:
            points = transform.Rotation.from_quat(quats).apply(points)
            points += positions
        shape = self.Lines(points=points,
                           colors=colors,
                           line_width=line_width,
                           num_vertices=num_vertices)
        shape.setName(f"{name}_{id(shape)}")
        return shape

    def Lines(self, points: List[List[float]] = ((0., 0., 0.), (1., 1., 1.)),
              colors: Union[List[float], List[List[float]]] = (1, 0, 0),
              line_width: int = 2, num_vertices: List[int] = None, name: str = "Lines") -> coin.SoShapeKit:
        """Draw one line segment or many line segments, with the color of each segment could be different

        :param points: [description], defaults to ((0., 0., 0.), (1., 1., 1.))
        :type points: List[List[float]], optional
        :param colors: [description], defaults to (1, 0, 0)
        :type colors: Union[List[float], List[List[float]]], optional
        :param line_width: [description], defaults to 2
        :type line_width: int, optional
        :param num_vertices: define the number of points of each segment,sum of num_vertices should be no more than number of points, defaults to None
        :type num_vertices: List[int], optional
        :return: [description]
        :rtype: SoShapeKit
        """
        vert_prop = coin.SoVertexProperty()
        vert_prop.vertex.setValues(0, len(points), points)
        if isinstance(colors[0], Iterable):
            vert_prop.orderedRGBA.setValues(
                0, len(colors), self.__PackColors(colors))
            vert_prop.materialBinding = coin.SoMaterialBinding.PER_PART
        else:
            vert_prop.orderedRGBA.set1Value(
                0, coin.SbColor(colors).getPackedValue())
            vert_prop.materialBinding = coin.SoMaterialBinding.OVERALL
        lines = coin.SoLineSet()
        lines.vertexProperty = vert_prop
        if num_vertices is not None:
            lines.numVertices.setValues(0, len(num_vertices), num_vertices)
        shape = coin.SoShapeKit()
        shape.setName(f"{name}_{id(shape)}")
        shape.setPart("shape", lines)
        shape.set("drawStyle {lineWidth %s}" % line_width)
        self.root.addChild(shape)
        return shape

    def Save(self, fname: str):
        if not fname.endswith(".iv"):
            fname = os.path.join(os.path.splitext(fname)[0], ".iv")
        if os.path.dirname(fname):
            os.makedirs(os.path.dirname(fname), exist_ok=True)
        out = coin.SoOutput()
        out.openFile(fname)
        writer = coin.SoWriteAction(out)
        writer.apply(self.root)
        out.closeFile()

    def __PackColors(self, colors: np.ndarray):
        if colors.shape[1] == 3:
            return np.frombuffer((np.c_[colors, np.ones(len(colors))]*255).astype(np.uint8), np.dtype(">u4"))
        else:
            # colors[:, -1] = 1 - colors[:, -1]
            colors[:, -1] = 1
            return np.frombuffer((colors*255).astype(np.uint8), np.dtype(">u4"))

    def __MyMouseEventCB(self, user_data, evt_node: coin.SoEventCallback):
        evt: coin.SoMouseButtonEvent = evt_node.getEvent()
        if evt.getButton() == evt.BUTTON4:
            self.ZoomIn()
            evt_node.setHandled()
        elif evt.getButton() == evt.BUTTON5:
            self.ZoomOut()
            evt_node.setHandled()

    def DeleteShape(self, shape: coin.SoNode):
        self.root.removeChild(shape)

    def LoadModel(self, filepath, positions: List[float] = (0, 0, 0), quats: List[float] = (0, 0, 0, 1),
                  colors: List[float] = (0.4, 0.6, 0.0), transparency: float = 0) -> coin.SoWrapperKit:
        shape = coin.SoWrapperKit()
        in_ = coin.SoInput()
        if not in_.openFile(filepath):
            return None
        mesh = coin.SoDB.readAll(in_)
        shape.setPart("contents", mesh)
        transform: coin.SoTransform = shape.getPart("transform", True)
        transform.translation.setValue(positions)
        transform.rotation.setValue(quats)
        if mesh:
            mesh[0].set("material {diffuseColor %s %s %s transparency %s}" % (
                colors[0], colors[1], colors[2], transparency))
        self.root.addChild(shape)
        return shape


if __name__ == "__main__":
    import pathlib
    import sys
    DATADIR = pathlib.Path(__file__).parent / "../../Data/"
    run_in_ipython = False
    if len(sys.argv) > 1:
        run_in_ipython = int(sys.argv[1])
    app = QtWidgets.QApplication.instance()
    if app is None:
        app = QtWidgets.QApplication([])
    iviewer = IViewer()
    # iviewer.Point(points=np.random.uniform(0.5, 1.5, (10, 3)),
    #            colors=(0., 1., 1.), point_size=3, name="TestPoints")
    # iviewer.Axes(positions=np.random.uniform(0, 1, (10, 3)), name="TestAxes")
    # iviewer.Box(positions=(0.5, 0.5, 0.), length=0.3,
    #          width=0.2, height=0.1, colors=(0.1, 0.3, 0.6), transparency=0.5, name="TestBox")
    # iviewer.Sphere(radius=0.05, name="TestSphere")
    # iviewer.Cone(positions=(0, 0, 0.5), name="TestCone")
    # iviewer.Cylinder(positions=(0.2, 0, 0), colors=(0, 1, 0),
    #               transparency=0.5, name="TestCylinder")

    # iviewer.Load(str(DATADIR / "Cube.stl"),
    #           positions=(0.2, 0.8, 0.0), colors=(0.1, 0.8, 0.1))
    # vertex_positions = (
    #     (0., 0., 0.),
    #     (0.2, 0., 0.),
    #     (0.0, 0.2, 0.),
    #     (0., 0.0, 0.2),
    # )
    # indices = (
    #     (0, 1, 2),
    #     (0, 1, 3),
    #     (0, 2, 3),
    #     (1, 2, 3),
    # )
    # colors = np.array([
    #     (1., 0., 0., 0.),
    #     (0., 1., 0., 0.),
    #     (0., 0., 1., 0.),
    #     (1., 1., 0., 0.),
    # ])
    # iviewer.TriMesh(np.array(vertex_positions),
    #              np.array(indices), np.array(colors), name="TestTriMesh")
