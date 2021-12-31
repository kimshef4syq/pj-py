#!/usr/bin/python3
import os

import numpy as np
import transforms3d
import vedo
import vtk
from PySide2.QtCore import Qt
from PySide2.QtWidgets import (QApplication, QDoubleSpinBox, QFrame,
                               QHBoxLayout, QLabel, QMainWindow, QPushButton,
                               QSlider, QSplitter, QVBoxLayout, QWidget)
from scipy.spatial.transform import Rotation
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor


def get_program_parameters():
    import argparse
    description = 'algorithms to simplify mesh'
    epilogue = '''
    (a) Decimation of mesh
    (b) Smoothing mesh
    (c) Transformation of mesh, including x,y,z,r,p,y and scale
    (d) Subdividing mesh
   '''
    parser = argparse.ArgumentParser(description=description, epilog=epilogue,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('filename', help='path_to_original_mesh, stl/ply/obj')
    parser.add_argument('--decimation-ratio', '-d', type=float, default=0,
                        help='decimation ratio')
    parser.add_argument('--laplacian-smooth-ratio', '-l', type=float, default=0,
                        help='laplacian smooth ratio')
    parser.add_argument('-x', type=float, default=0.0,
                        help='transformation, x')
    parser.add_argument('-y', type=float, default=0.0,
                        help='transformation, y')
    parser.add_argument('-z', type=float, default=0.0,
                        help='transformation, z')
    parser.add_argument('--Rx', type=float, default=0.0,
                        help='transformation, roation about x, unit degree')
    parser.add_argument('--Ry', type=float, default=0.0,
                        help='transformation, roation about y, unit degree')
    parser.add_argument('--Rz', type=float, default=0.0,
                        help='transformation, roation about z, unit degree')
    parser.add_argument('--scale', '-s', type=int, choices=(0, 1, 2),
                        default=1, help='transformation, scale, could be used to convert unit, 0: 0.001, 1: 1.0, 2: 1000.0')
    parser.add_argument('--subdivision-num', '-t', type=int, default=0,
                        help='subdivide each mesh triangle to 4 new triangles, donot give a value larger than 4, which will cause a lot of time')
    parser.add_argument('--out', '-o', type=str,
                        default="", help='output filename, deafut $input_name$_compressed.stl, extention should be ply or stl')
    args = parser.parse_args()
    print(args)
    return args.filename, args.decimation_ratio, args.laplacian_smooth_ratio, args.x, args.y, args.z, args.Rx, args.Ry, args.Rz, args.scale, args.subdivision_num, args.out


vedo.embedWindow(backend=None)
app = QApplication.instance()
if not app:
    app = QApplication([])


file_name, decimation_ratio, smooth_ratio, x, y, z, rx, ry, rz, scale, subdivision_num, file_name_save = get_program_parameters()


def VtkMatrixToNpy(m):
    return np.array([m.GetElement(i, j) for i in range(4) for j in range(4)]).reshape(4, 4)


def VtkTransformToNpy(t):
    return VtkMatrixToNpy(t.GetMatrix())


def NpyToVtkMatrix(m):
    ma = vtk.vtkMatrix4x4()
    for i in range(4):
        for j in range(4):
            ma.SetElement(i, j, m[i, j])
    return ma


def VtkTransformToEuler(t):
    angle, x, y, z = t.GetOrientationWXYZ()
    return Rotation.from_rotvec(np.deg2rad(angle) * np.array([x, y, z])).as_euler('xyz', degrees=True)


class MeshViewer(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle('MeshViewer')
        self.hsplitter = QSplitter(Qt.Orientation.Horizontal)
        self.sider = QFrame()
        self.vl = QVBoxLayout(self.hsplitter)
        self.vl.addLayout(self.add_spin_box(
            [x, rx], [-100, -180], [100, 180], [0.01, 1], ['X', 'RX'], [self.translate_x, self.rotate_x]))
        self.vl.addLayout(self.add_spin_box(
            [y, ry], [-100, -180], [100, 180], [0.01, 1], ['Y', 'RY'], [self.translate_y, self.rotate_y]))
        self.vl.addLayout(self.add_spin_box(
            [z, rz], [-100, -180], [100, 180], [0.01, 1], ['Z', 'RZ'], [self.translate_z, self.rotate_z]))
        self.vl.addLayout(self.add_slider([scale], [0], [2], [1], [
                          'Scale Ratio'], [self.scale_ratio_slider_call_back]))
        self.vl.addLayout(self.add_slider([decimation_ratio], [0], [100], [1], [
                          'Decimation Ratio'], [self.decimation_ratio_slider_call_back]))
        self.vl.addLayout(self.add_slider([smooth_ratio], [0], [100], [1], [
                          'Smoothness Ratio'], [self.smooth_ratio_slider_call_back]))
        self.vl.addLayout(self.add_slider([subdivision_num], [0], [5], [1], [
                          'Subdivision number'], [self.subdivision_slider_call_back]))

        hl = QHBoxLayout()
        self.crop_btn = QPushButton('Crop')
        self.crop_btn.setCheckable(True)
        self.crop_btn.clicked.connect(self.Crop)
        self.save_btn = QPushButton('Save')
        self.save_btn.clicked.connect(self.Save)
        hl.addWidget(self.crop_btn)
        hl.addWidget(self.save_btn)
        hl.addWidget(self.save_btn)
        self.vl.addLayout(hl)
        self.vl.addStretch(1)

        self.sider.setLayout(self.vl)
        self.hsplitter.addWidget(self.sider)

        self.vtkwidget = QVTKRenderWindowInteractor(self.hsplitter)
        self.hsplitter.addWidget(self.vtkwidget)
        self.hsplitter.setStretchFactor(0, 3)
        self.hsplitter.setStretchFactor(1, 7)
        self.setCentralWidget(self.hsplitter)

        self.vp = vedo.Plotter(axes=8, qtWidget=self.vtkwidget)
        self.LoadActor()
        self.vp.show(interactorStyle=0)

        self.iren = self.vtkwidget.GetRenderWindow().GetInteractor()
        self.iren.AddObserver("LeftButtonPressEvent", self.vp._mouseleft)
        self.iren.AddObserver("RightButtonPressEvent", self.vp._mouseright)
        self.iren.AddObserver("MiddleButtonPressEvent", self.vp._mousemiddle)

        def keypress(obj, e):
            self.vp._keypress(obj, e)
            print(self.iren.GetKeySym())
            if self.iren.GetKeySym() in ["q"]:
                self.iren.ExitCallback()
                exit()
            elif self.iren.GetKeySym() in ["space"]:
                self.vp.resetCamera()
                self.iren.Render()
        self.iren.AddObserver("KeyPressEvent", keypress)

        self.boxWidget = None

    def add_slider(self, vals, minvals, maxvals, steps, titles, call_backs):
        hl = QHBoxLayout()
        for val, minval, maxval, step, title, call_back in zip(vals, minvals, maxvals, steps, titles, call_backs):
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setMinimum(minval)
            slider.setMaximum(maxval)
            slider.setSingleStep(step)
            slider.setValue(val)
            slider.valueChanged.connect(call_back)
            label = QLabel(title)
            label.setMinimumWidth(110)
            hl.addWidget(label)
            hl.addWidget(slider)
        return hl

    def add_spin_box(self, vals, minvals, maxvals, steps, titles, call_backs):
        hl = QHBoxLayout()
        for val, minval, maxval, step, title, call_back in zip(vals, minvals, maxvals, steps, titles, call_backs):
            spin_box = QDoubleSpinBox()
            spin_box.setMinimum(minval)
            spin_box.setMaximum(maxval)
            spin_box.setSingleStep(step)
            spin_box.setValue(val)
            spin_box.valueChanged.connect(call_back)
            label = QLabel(title)
            label.setMinimumWidth(20)
            hl.addWidget(label)
            hl.addWidget(spin_box)
        hl.addStretch(1)
        return hl

    def keypress(self, obj, e):
        self._keypress(obj, e)
        if self.iren.GetKeySym() in ["space"]:
            self.renderer.ResetCamera()
        elif self.iren.GetKeySym() in ["q", "escape"]:
            self.iren.ExitCallback()
            # app.quit()

    def decimation_ratio_slider_call_back(self, value):
        ratio = value / 100
        self.decimation_mesh.SetTargetReduction(ratio)
        poly0 = self.origin_mesh_reader.GetOutput()
        poly1 = self.smoother_filter.GetOutput()
        print(
            f"original size, cells  : {poly0.GetNumberOfCells()}, vertices: {poly0.GetNumberOfPoints()}")
        print(
            f"simplified size, cells: {poly1.GetNumberOfCells()}, vertices: {poly1.GetNumberOfPoints()}")
        self.vtkwidget.Render()

    def smooth_ratio_slider_call_back(self, value):
        ratio = value / 100
        print(f"smooth ratio: {ratio}")
        self.smoother_filter.SetRelaxationFactor(ratio)
        self.vtkwidget.Render()

    def scale_ratio_slider_call_back(self, value):
        if value == 0:
            value = 0.001
        elif value == 2:
            value = 1000.0
        else:
            value = 1.0
        m = VtkTransformToNpy(self.transform)
        T, R, Z, S = transforms3d.affines.decompose44(m)
        Z[:] = value
        m = transforms3d.affines.compose(T, R, Z, S)
        self.transform.SetMatrix(NpyToVtkMatrix(m))
        self.transform_filter.SetTransform(self.transform)
        self.vp.remove(self.vp.axes_instances)
        self.vp.axes_instances[0] = None
        self.vp.addGlobalAxes(axtype=8)
        self.vp.resetCamera()
        self.iren.Render()

    def subdivision_slider_call_back(self, value):
        self.subdivision_filter.SetNumberOfSubdivisions(value)
        self.iren.Render()

    def translate_x(self, v):
        m = VtkTransformToNpy(self.transform)
        T, R, Z, S = transforms3d.affines.decompose44(m)
        T[0] = v
        m = transforms3d.affines.compose(T, R, Z, S)
        self.transform.SetMatrix(NpyToVtkMatrix(m))
        self.transform_filter.SetTransform(self.transform)
        self.iren.Render()

    def translate_y(self, v):
        m = VtkTransformToNpy(self.transform)
        T, R, Z, S = transforms3d.affines.decompose44(m)
        T[1] = v
        m = transforms3d.affines.compose(T, R, Z, S)
        self.transform.SetMatrix(NpyToVtkMatrix(m))
        self.transform_filter.SetTransform(self.transform)
        self.iren.Render()

    def translate_z(self, v):
        m = VtkTransformToNpy(self.transform)
        T, R, Z, S = transforms3d.affines.decompose44(m)
        T[2] = v
        m = transforms3d.affines.compose(T, R, Z, S)
        self.transform.SetMatrix(NpyToVtkMatrix(m))
        self.transform_filter.SetTransform(self.transform)
        self.iren.Render()

    def rotate_x(self, v):
        global rx
        rx = v
        m = VtkTransformToNpy(self.transform)
        T, R, Z, S = transforms3d.affines.decompose44(m)
        R = transforms3d.euler.euler2mat(*np.deg2rad([rx, ry, rz]), 'sxyz')
        m = transforms3d.affines.compose(T, R, Z, S)
        self.transform.SetMatrix(NpyToVtkMatrix(m))
        self.transform_filter.SetTransform(self.transform)
        self.iren.Render()

    def rotate_y(self, v):
        global ry
        ry = v
        m = VtkTransformToNpy(self.transform)
        T, R, Z, S = transforms3d.affines.decompose44(m)
        R = transforms3d.euler.euler2mat(*np.deg2rad([rx, ry, rz]), 'sxyz')
        m = transforms3d.affines.compose(T, R, Z, S)
        self.transform.SetMatrix(NpyToVtkMatrix(m))
        self.transform_filter.SetTransform(self.transform)
        self.iren.Render()

    def rotate_z(self, v):
        global rz
        rz = v
        m = VtkTransformToNpy(self.transform)
        T, R, Z, S = transforms3d.affines.decompose44(m)
        R = transforms3d.euler.euler2mat(*np.deg2rad([rx, ry, rz]), 'sxyz')
        m = transforms3d.affines.compose(T, R, Z, S)
        self.transform.SetMatrix(NpyToVtkMatrix(m))
        self.transform_filter.SetTransform(self.transform)
        self.iren.Render()

    def LoadActor(self):
        if file_name.endswith(".stl") or file_name.endswith(".STL"):
            self.origin_mesh_reader = vtk.vtkSTLReader()
        elif file_name.endswith(".ply"):
            self.origin_mesh_reader = vtk.vtkPLYReader()
        elif file_name.endswith(".obj"):
            self.origin_mesh_reader = vtk.vtkOBJReader()
        else:
            print("only stl/ply/obj format is supported!")
            exit(-1)
        self.file_name = file_name

        colors = vtk.vtkNamedColors()
        self.origin_mesh_reader.SetFileName(file_name)

        self.transform_filter = vtk.vtkTransformPolyDataFilter()
        self.transform_filter.SetInputConnection(
            self.origin_mesh_reader.GetOutputPort())
        self.transform = vtk.vtkTransform()
        s = [0.001, 1.0, 1000.0][scale]
        self.transform.Scale(s, s, s)
        self.transform.Translate(x, y, z)
        self.transform.RotateX(rx)
        self.transform.RotateY(ry)
        self.transform.RotateZ(rz)
        self.transform_filter.SetTransform(self.transform)

        self.decimation_mesh = vtk.vtkDecimatePro()
        self.decimation_mesh.SetInputConnection(
            self.transform_filter.GetOutputPort())
        self.decimation_mesh.SetTargetReduction(0)
        self.decimation_mesh.PreserveTopologyOn()

        self.decimation_normals = vtk.vtkPolyDataNormals()
        self.decimation_normals.SetInputConnection(
            self.decimation_mesh.GetOutputPort())
        self.decimation_normals.FlipNormalsOn()
        self.decimation_normals.SetFeatureAngle(60)

        self.smoother_filter = vtk.vtkSmoothPolyDataFilter()
        self.smoother_filter.SetInputConnection(
            self.decimation_normals.GetOutputPort())
        self.smoother_filter.SetRelaxationFactor(0)
        self.smoother_filter.FeatureEdgeSmoothingOn()
        self.smoother_filter.SetNumberOfIterations(50)

        self.subdivision_filter = vtk.vtkLinearSubdivisionFilter()
        self.subdivision_filter.SetNumberOfSubdivisions(subdivision_num)
        self.subdivision_filter.SetInputConnection(
            self.smoother_filter.GetOutputPort())

        self.decimation_mapper = vtk.vtkPolyDataMapper()
        self.decimation_mapper.SetInputConnection(
            self.subdivision_filter.GetOutputPort())

        self.actor = vtk.vtkActor()
        self.actor.SetMapper(self.decimation_mapper)
        self.actor.GetProperty().SetOpacity(1.0)
        self.actor.GetProperty().SetAmbient(.5)
        self.actor.GetProperty().SetDiffuse(.5)
        self.actor.GetProperty().SetColor(colors.GetColor3d("Yellow"))

        self.origin_mapper = vtk.vtkPolyDataMapper()
        self.origin_mapper.SetInputConnection(
            self.origin_mesh_reader.GetOutputPort())
        self.actor_origin = vtk.vtkActor()
        self.actor_origin.SetMapper(self.origin_mapper)
        self.actor_origin.GetProperty().SetOpacity(.2)
        self.actor_origin.GetProperty().SetAmbient(.5)
        self.actor_origin.GetProperty().SetDiffuse(.5)
        self.actor_origin.GetProperty().SetColor(colors.GetColor3d("Red"))

        self.vp += self.actor
        # self.vp += self.actor_origin

    def onClose(self):
        # Disable the interactor before closing to prevent it
        # from trying to act on already deleted items
        # self.Save()
        self.vtkwidget.close()

    def Save(self):
        global file_name_save
        if not file_name_save:
            file_name_save = os.path.splitext(self.file_name)[
                0] + "_compressed.stl"
        if file_name_save.endswith('.stl'):
            writer = vtk.vtkSTLWriter()
        else:
            writer = vtk.vtkPLYWriter()
        writer.SetFileTypeToBinary()
        writer.SetFileName(file_name_save)
        writer.SetInputConnection(self.smoother_filter.GetOutputPort())
        writer.Write()
        print(f"saved to {file_name_save}")

    def Crop(self, checked):
        if not checked:
            if self.boxWidget:
                self.clipper.Update()
                trans_f = vtk.vtkTransformPolyDataFilter()
                trans = vtk.vtkTransform()
                trans.SetInverse(self.transform)
                trans_f.SetTransform(trans)
                trans_f.SetInputConnection(self.clipper.GetOutputPort())
                trans_f.Update()
                cpoly = trans_f.GetOutput()
                poly = vtk.vtkPolyData()
                poly.DeepCopy(cpoly)
                self.transform_filter.SetInputData(poly)
                self.decimation_mesh.SetInputConnection(
                    self.transform_filter.GetOutputPort())
                self.boxWidget.Off()
                self.vp.remove(self.act1)
                self.iren.Render()
            return
        apd = self.transform_filter.GetOutput()
        planes = vtk.vtkPlanes()
        planes.SetBounds(apd.GetBounds())

        self.clipper = vtk.vtkClipPolyData()
        self.clipper.GenerateClipScalarsOff()
        self.clipper.SetInputConnection(
            self.transform_filter.GetOutputPort())
        self.clipper.SetClipFunction(planes)
        self.clipper.SetInsideOut(True)
        self.clipper.GenerateClippedOutputOn()
        self.clipper.Update()
        self.decimation_mesh.SetInputConnection(self.clipper.GetOutputPort())

        self.act1 = vedo.Mesh()
        self.act1.mapper().SetInputConnection(
            self.clipper.GetClippedOutputPort())  # needs OutputPort
        self.act1.alpha(0.04).color((0.5, 0.5, 0.5)).wireframe()
        self.vp.add(self.act1)

        def selectPolygons(vobj, event):
            vobj.GetPlanes(planes)

        boxWidget = vtk.vtkBoxWidget()
        boxWidget.OutlineCursorWiresOn()
        boxWidget.GetSelectedOutlineProperty().SetColor(1, 0, 1)
        boxWidget.GetOutlineProperty().SetColor(0.2, 0.2, 0.2)
        boxWidget.GetOutlineProperty().SetOpacity(0.8)
        boxWidget.SetPlaceFactor(1.025)
        boxWidget.SetInteractor(self.vp.interactor)
        boxWidget.SetCurrentRenderer(self.vp.renderer)
        boxWidget.SetInputData(apd)
        boxWidget.PlaceWidget()
        boxWidget.AddObserver("InteractionEvent", selectPolygons)
        boxWidget.On()
        self.vp.widgets.append(boxWidget)

        self.vp.cutterWidget = boxWidget
        self.vp.clickedActor = self.actor

        self.boxWidget = boxWidget
        self.iren.Render()


if __name__ == '__main__':
    mv = MeshViewer()
    mv.show()
    app.aboutToQuit.connect(mv.onClose)  # <-- connect the onClose event
    app.exec_()
    # in terminal, run "python3 MeshSimplification.py -h" to see the help
