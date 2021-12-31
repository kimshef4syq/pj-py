#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Copyright (c) RVBUST, Inc - All rights reserved.

from __future__ import annotations

import json
import os
import typing
from typing import Any, Dict, List, Union

import numpy as np
import PySide2
from IPython import embed
from pivy import coin
from PySide2 import QtGui, QtWidgets
from PySide2.QtCore import Qt
from RVBUST import RCI, RPS

try:
    from IVis import *
except ImportError:
    from .IVis import *

RCI.Logger.SetLevelForAll(RCI.LoggerLevel_Error)

logger = RCI.RichLogger("App")

DATADIR = RCI.GetDataPath()
DATADIR = "/home/rvbust/Rvbust/Data/Multibody/RobotModels/UniversalRobots/UR10"
RESOURCES_DIR = pathlib.Path(__file__).parent / "Resources"


def Read(fname):
    if os.path.exists(fname):
        with open(fname, "r") as f:
            return f.read()
    else:
        logger.warning(f"{fname} is not exist!")
        return ""


def GetHorizontalLine():
    line = QtWidgets.QFrame()
    line.setFrameShape(QtWidgets.QFrame.HLine)
    line.setFrameShadow(QtWidgets.QFrame.Sunken)
    return line


class PoseWidget:
    pose_result = None

    @classmethod
    def GetPoseWidget(cls, init_pose: RCI.Pose = RCI.Pose(), title: str = "设置Pose", parent=None, confirm_cb=None):
        cls.pose_result = None
        dialog = QtWidgets.QDialog(parent=parent)
        dialog.setWindowTitle(title)
        dialog.setMinimumWidth(300)
        vbox = QtWidgets.QVBoxLayout()

        rotation_type = QtWidgets.QComboBox()
        rotation_type.addItems(["欧拉角(RPY)", "四元数"])
        hbox = QtWidgets.QHBoxLayout()
        hbox.addWidget(QtWidgets.QLabel("旋转类型: "))
        hbox.addWidget(rotation_type)
        vbox.addLayout(hbox)

        layout = QtWidgets.QFormLayout()
        cart_editors = []
        cart_labels = ("X", "Y", "Z")
        for i, label in enumerate(cart_labels):
            editor = QtWidgets.QDoubleSpinBox()
            editor.setMinimum(-10)
            editor.setMaximum(10)
            editor.setSingleStep(0.1)
            editor.setDecimals(3)
            editor.setValue(init_pose[i])
            layout.addRow(label, editor)
            cart_editors.append(editor)

        rot_editors = []

        def RotationTypeChangeCB(idx):
            if len(rot_editors):
                rot = RCI.Rotation(*[w.value() for w in rot_editors])
            else:
                rot = init_pose.GetSO3()
            for _ in range(len(rot_editors)):
                layout.removeRow(3)
            rot_editors.clear()
            if idx == 0:
                rpy = rot.RPY()
                rot_labels = ("Roll", "Pitch", "Yaw")
                for i, label in enumerate(rot_labels, 3):
                    editor = QtWidgets.QDoubleSpinBox()
                    editor.setMinimum(-np.pi)
                    editor.setMaximum(np.pi)
                    editor.setSingleStep(0.1)
                    editor.setDecimals(3)
                    editor.setValue(rpy[i-3])
                    layout.addRow(label, editor)
                    rot_editors.append(editor)
            else:
                rot_labels = ("OX", "OY", "OZ", "OW")
                quat = rot.Quat()
                for i, label in enumerate(rot_labels, 6):
                    editor = QtWidgets.QDoubleSpinBox()
                    editor.setMinimum(-np.pi)
                    editor.setMaximum(np.pi)
                    editor.setSingleStep(0.1)
                    editor.setDecimals(3)
                    editor.setValue(quat[i-6])
                    layout.addRow(label, editor)
                    rot_editors.append(editor)
        rotation_type.currentIndexChanged.connect(RotationTypeChangeCB)
        RotationTypeChangeCB(0)
        vbox.addLayout(layout)

        def ConfirmCB():
            translation = [w.value() for w in cart_editors]
            rot = [w.value() for w in rot_editors]
            if confirm_cb is not None:
                confirm_cb(RCI.Pose(translation, rot))
            cls.pose_result = RCI.Pose(translation, rot)
        confirm_btn = QtWidgets.QPushButton("确定")
        confirm_btn.clicked.connect(ConfirmCB)

        def CancelCB():
            cls.pose_result = None
            dialog.close()
        cancel_btn = QtWidgets.QPushButton("取消")
        cancel_btn.clicked.connect(CancelCB)

        hbox = QtWidgets.QHBoxLayout()
        hbox.addWidget(confirm_btn)
        hbox.addWidget(cancel_btn)
        vbox.addLayout(hbox)

        dialog.setLayout(vbox)
        dialog.show()
        dialog.exec_()
        return cls.pose_result


class RobotDockPanel(QtWidgets.QDockWidget):
    def __init__(self, main_win: MainWindow, parent: typing.Optional[PySide2.QtWidgets.QWidget] = None) -> None:
        super().__init__(parent=parent)
        self.setStyleSheet(Read(RESOURCES_DIR/"Qss/RobotControlPanel.qss"))
        self.setWindowTitle(u"机器人管理")

        self.app_data = main_win.app_data
        self.status_bar = main_win.statusBar()
        self.robot_status = main_win.status_bar_robot_status
        self.robot_speed_ratio = main_win.status_bar_robot_speed_ratio
        self.ivis = main_win.ivis
        self.setObjectName("RobotControlPanel")

        controller = self.app_data.get('controller')

        main_layout = QtWidgets.QVBoxLayout()

        group_box = QtWidgets.QGroupBox("基本设置")
        group_box_grid = QtWidgets.QGridLayout(group_box)
        main_layout.addWidget(group_box)

        form = QtWidgets.QFormLayout()
        self.robot_ip_editor = QtWidgets.QLineEdit()
        self.robot_ip_editor.setText("127.0.0.1")
        self.robot_ip_editor.setValidator(
            QtGui.QRegExpValidator("\d+\.\d+\.\d+\.\d+"))
        form.addRow("机器人IP", self.robot_ip_editor)
        self.speed_ratio_slider = QtWidgets.QSlider()
        self.speed_ratio_slider.setOrientation(Qt.Orientation.Horizontal)
        self.speed_ratio_slider.setMinimum(0)
        self.speed_ratio_slider.setMaximum(100)
        self.speed_ratio_slider.valueChanged.connect(self.__SetSpeedRatio)
        self.speed_ratio_slider.setValue(controller.GetSpeedRatio()*100)
        form.addRow("速率", self.speed_ratio_slider)
        group_box_grid.addLayout(form, 0, 0, 1, 2)

        self.sim_radio_btn = QtWidgets.QRadioButton("仿真机器人")
        self.sim_radio_btn.setChecked(True)
        self.real_radio_btn = QtWidgets.QRadioButton("实际机器人")
        self.sim_radio_btn.toggled.connect(self.__SwitchRealSimController)
        self.real_radio_btn.toggled.connect(self.__SwitchRealSimController)
        group_box_grid.addWidget(self.sim_radio_btn, 1, 0)
        group_box_grid.addWidget(self.real_radio_btn, 1, 1)

        self.connect_btn = QtWidgets.QPushButton("连接")
        self.connect_btn.clicked.connect(self.__Connect)
        self.disconnect_btn = QtWidgets.QPushButton("断开")
        self.disconnect_btn.clicked.connect(self.__Disconnect)
        group_box_grid.addWidget(self.connect_btn, 2, 0)
        group_box_grid.addWidget(self.disconnect_btn, 2, 1)

        self.enable_btn = QtWidgets.QPushButton("使能")
        self.enable_btn.clicked.connect(self.__EnableRobot)
        self.disable_btn = QtWidgets.QPushButton("下电")
        self.disable_btn.clicked.connect(self.__DisableRobot)
        group_box_grid.addWidget(self.enable_btn, 3, 0)
        group_box_grid.addWidget(self.disable_btn, 3, 1)

        self.update_tcp_btn = QtWidgets.QPushButton("更新TCP")
        self.update_tcp_btn.clicked.connect(self.__UpdateTCP)
        self.is_show_tcp_axes = QtWidgets.QCheckBox("显示TCP")
        group_box_grid.addWidget(self.update_tcp_btn, 4, 0)
        group_box_grid.addWidget(self.is_show_tcp_axes, 4, 1)

        group_box = QtWidgets.QGroupBox("运动控制")
        group_box_grid = QtWidgets.QGridLayout(group_box)
        main_layout.addWidget(group_box)
        self.update_home_btn = QtWidgets.QPushButton("更新Home点")
        self.move_home_btn = QtWidgets.QPushButton("回到Home点")
        self.update_home_btn.clicked.connect(self.__UpdateRobotHomePosition)
        self.move_home_btn.clicked.connect(self.__MoveToRobotHomePosition)
        group_box_grid.addWidget(self.update_home_btn, 0, 0)
        group_box_grid.addWidget(self.move_home_btn, 0, 1)

        self.start_drag_btn = QtWidgets.QPushButton("开始拖动")
        self.stop_drag_btn = QtWidgets.QPushButton("停止拖动")
        self.start_drag_btn.clicked.connect(self.__StartDragRobot)
        self.stop_drag_btn.clicked.connect(self.__StopDragRobot)
        group_box_grid.addWidget(self.start_drag_btn, 1, 0)
        group_box_grid.addWidget(self.stop_drag_btn, 1, 1)

        # jogger panel
        self.jogger_btns = []
        self.start_jogger_btn = QtWidgets.QPushButton("开始点动")
        self.start_jogger_btn.clicked.connect(self.__StartJogRobot)
        self.stop_jogger_btn = QtWidgets.QPushButton("停止点动")
        self.stop_jogger_btn.clicked.connect(self.__StopJogRobot)
        self.is_base_frame_check_box = QtWidgets.QCheckBox("基坐标")
        self.is_base_frame_check_box.setChecked(True)
        self.is_step_mode_check_box = QtWidgets.QCheckBox("步进模式")
        self.is_step_mode_check_box.setChecked(True)
        self.is_step_mode_check_box.stateChanged.connect(
            lambda v: self.step_spin_box.setEnabled(v))
        group_box_grid.addWidget(self.start_jogger_btn, 2, 0)
        group_box_grid.addWidget(self.stop_jogger_btn, 2, 1)
        group_box_grid.addWidget(self.is_base_frame_check_box, 3, 0)
        group_box_grid.addWidget(self.is_step_mode_check_box, 3, 1)

        form = QtWidgets.QFormLayout()
        self.step_spin_box = QtWidgets.QDoubleSpinBox()
        self.step_spin_box.setMinimum(0.01)
        self.step_spin_box.setMaximum(0.5)
        self.step_spin_box.setSingleStep(0.01)
        self.step_spin_box.setDecimals(4)
        form.addRow("步长: ", self.step_spin_box)
        group_box_grid.addLayout(form, 7, 0, 1, 2)

        # joint jogger
        dof = controller.GetDoF()
        self.jog_joint_group = QtWidgets.QGroupBox("关节点动")
        self.jog_joint_group.setObjectName("JogPanel")
        jog_joint_grid = QtWidgets.QGridLayout(self.jog_joint_group)
        main_layout.addWidget(self.jog_joint_group)
        for i in range(dof):
            jogger_btn_minus = QtWidgets.QPushButton(f"J{i+1}-")
            jogger_btn_minus.pressed.connect(
                lambda k=i: self.__JogRobotAlongDirection(k, False, True))
            jogger_btn_minus.released.connect(self.__StopJogMove)
            jogger_btn_plus = QtWidgets.QPushButton(f"J{i+1}+")
            jogger_btn_plus.pressed.connect(
                lambda k=i: self.__JogRobotAlongDirection(k, True, True))
            jogger_btn_plus.released.connect(self.__StopJogMove)
            jog_joint_grid.addWidget(jogger_btn_minus, i, 0)
            jog_joint_grid.addWidget(jogger_btn_plus, i, 1)
            self.jogger_btns.append(jogger_btn_plus)
            self.jogger_btns.append(jogger_btn_minus)
        # cartesian jogger
        self.jog_cart_group = QtWidgets.QGroupBox("笛卡尔点动")
        self.jog_cart_group.setObjectName("JogPanel")
        jog_cart_grid = QtWidgets.QGridLayout(self.jog_cart_group)
        main_layout.addWidget(self.jog_cart_group)
        labels = ("X", "Y", "Z", "Rx", "Ry", "Rz")
        for i in range(6):
            jogger_btn_minus = QtWidgets.QPushButton(f"{labels[i]}-")
            jogger_btn_minus.pressed.connect(
                lambda k=i: self.__JogRobotAlongDirection(k, False, False))
            jogger_btn_minus.released.connect(self.__StopJogMove)
            jogger_btn_plus = QtWidgets.QPushButton(f"{labels[i]}+")
            jogger_btn_plus.pressed.connect(
                lambda k=i: self.__JogRobotAlongDirection(k, True, False))
            jogger_btn_plus.released.connect(self.__StopJogMove)
            jog_cart_grid.addWidget(jogger_btn_minus, i, 0)
            jog_cart_grid.addWidget(jogger_btn_plus, i, 1)
            self.jogger_btns.append(jogger_btn_plus)
            self.jogger_btns.append(jogger_btn_minus)
        self.__StopJogRobot()

        main_layout.addStretch(1)
        self.status_group = QtWidgets.QGroupBox("机器人当前位置")
        main_layout.addWidget(self.status_group)
        self.joint_position_label = QtWidgets.QLabel(f"{RCI.Rx(dof)}")
        self.cart_position_label = QtWidgets.QLabel(f"{RCI.Pose()}") # tcp position value
        vbox = QtWidgets.QVBoxLayout()
        vbox.addWidget(self.joint_position_label)
        vbox.addWidget(self.cart_position_label)
        self.status_group.setLayout(vbox)
        self.status_group.setObjectName("status_box_group")

        widget = QtWidgets.QWidget()
        widget.setLayout(main_layout)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidget(widget)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.setWidget(scroll)

        self.startTimer(100, Qt.CoarseTimer)

    def timerEvent(self, event: PySide2.QtCore.QTimerEvent) -> None:
        self.__TimerCB()
        return super().timerEvent(event)

    def __UpdateRobotHomePosition(self):
        btn = QtWidgets.QMessageBox.question(
            self, "更新Home点", "是否将当前点更新为home点？")
        if btn == QtWidgets.QMessageBox.Yes:
            controller = self.app_data.get('controller')
            manipulator = controller.GetManipulator()
            ret, q = controller.GetJointPosition()
            if ret == RCI.RVSReturn_Success:
                manipulator.SetHomePosition(q)
                self.status_bar.showMessage(f"设置Home点为: {q}")
        else:
            logger.info("取消更新!")

    def __MoveToRobotHomePosition(self):
        controller = self.app_data.get('controller')
        manipulator = controller.GetManipulator()
        ret, q0 = controller.GetJointPosition()
        if ret == RCI.RVSReturn_Success:
            q1 = manipulator.GetHomePosition()
            path = RCI.CreatePath([q0, q1])
            traj = RCI.CreateTrajectory(path, controller.GetJointLimits())
            traj_shape = self.ivis.DrawTraj(traj, controller.GetKinSolver())
            btn = QtWidgets.QMessageBox.question(
                self, "回Home点轨迹检查", "确定执行该轨迹吗？")

            def __ExecTraj():
                if btn == QtWidgets.QMessageBox.Yes:
                    controller.MoveJoints([q0, q1])
                else:
                    logger.info("取消执行轨迹!")
                self.ivis.iviewer.DeleteShape(traj_shape)
            Thread(target=__ExecTraj, daemon=True).start()

    def __SwitchRealSimController(self):
        if self.sim_radio_btn.isChecked():
            logger.info("using simulation controller")
            self.app_data['controller'] = self.app_data['controller_sim']
        else:
            if self.app_data['controller_real'] is not None:
                logger.info("using real controller")
                self.app_data['controller'] = self.app_data['controller_real']
            else:
                logger.info("real controller is not created!")
        controller = self.app_data.get('controller')
        manipulator = controller.GetManipulator()
        manipulator.SetArmController(controller)

    def __JogRobotAlongDirection(self, dof_idx: int, positive_direction: bool, is_joint_space: bool):
        controller = self.app_data.get('controller')
        controller_jogger = self.app_data.get('controller_jogger')
        controller_state = controller.GetControllerState()
        if self.is_step_mode_check_box.isChecked():  # setp 模式 确定
            controller_jogger.StopJog()
            if not controller_state.IsMoving():
                step = self.step_spin_box.value()
                step = step if positive_direction else -step
                if is_joint_space:
                    _, q = controller.GetJointPosition()
                    q[dof_idx] += step
                    controller.MoveJoints(q, False)
                else:
                    _, pose = controller.GetPose()
                    offset = RCI.SE3Tangent(np.eye(6)[dof_idx]) * step
                    if self.is_base_frame_check_box.isChecked():
                        pose = offset + pose
                    else:
                        pose = pose + offset
                    controller.MoveLinear(pose, False)
        else:
            controller_jogger.StartJog()
            dof = controller.GetDoF()
            directions = np.eye(dof)
            if not positive_direction:
                directions *= -1.0
            if is_joint_space:
                controller_jogger.UpdateDirection(
                    RCI.RxTangent(directions[dof_idx]))
            else:
                controller_jogger.UpdateDirection(
                    RCI.SE3Tangent(directions[dof_idx]))

    def __StopJogMove(self):
        controller_jogger = self.app_data.get('controller_jogger')
        if controller_jogger is not None:
            controller_jogger.StopMove()

    def __TimerCB(self, *args) -> None:


        controller = self.app_data.get('controller')
        if controller is not None:
            controller_state: RCI.ControllerState = controller.GetControllerState()
        else:
            return
        if controller_state.IsStateServerConnected():
            _, joints = controller.GetJointPosition()
            _, pose = controller.GetPose()
            joints_text = "关节: " + \
                ",".join(("%5.3f" % v for v in joints.Coeffs()))
            self.joint_position_label.setText(joints_text)
            pose_text = "笛卡尔: " + \
                ",".join(("%5.3f" % v for v in pose.Coeffs()))
            self.cart_position_label.setText(pose_text)
            if self.is_show_tcp_axes.isChecked():
                _, pose = controller.GetPose()
                if self.app_data.get('tcp_axes') is not None:
                    self.ivis.SetTransform(self.app_data['tcp_axes'], pose)
                else:
                    self.app_data['tcp_axes'] = self.ivis.PlotFrame(
                        length=0.1, line_width=2)
                    self.ivis.SetTransform(self.app_data['tcp_axes'], pose)
            else:
                if self.app_data.get('tcp_axes') is not None:
                    self.ivis.iviewer.DeleteShape(self.app_data['tcp_axes'])
                    self.app_data['tcp_axes'] = None
        if controller_state.IsRobotEnabled():
            self.robot_status.setText(
                '<font color="red">已使能</font>')
        elif controller_state.IsStateServerConnected():
            self.robot_status.setText(
                '<font color="green">已连接</font>')
        else:
            self.robot_status.setText(
                '<font color="blue">未连接!</font>')

    def __SetEnabledButtonsWhenJogging(self, enabled: bool):
        self.connect_btn.setEnabled(enabled)
        self.disconnect_btn.setEnabled(enabled)
        self.enable_btn.setEnabled(enabled)
        self.disable_btn.setEnabled(enabled)
        self.start_drag_btn.setEnabled(enabled)
        self.stop_drag_btn.setEnabled(enabled)
        self.start_jogger_btn.setEnabled(enabled)
        self.move_home_btn.setEnabled(enabled)

    def __SetEnableButtonsWhenDragging(self, enabled: bool):
        self.connect_btn.setEnabled(enabled)
        self.disconnect_btn.setEnabled(enabled)
        self.enable_btn.setEnabled(enabled)
        self.disable_btn.setEnabled(enabled)
        self.start_jogger_btn.setEnabled(enabled)
        self.stop_jogger_btn.setEnabled(enabled)
        self.start_drag_btn.setEnabled(enabled)
        self.move_home_btn.setEnabled(enabled)

    def __StartJogRobot(self):
        controller = self.app_data.get('controller')
        if controller is None:
            self.status_bar.showMessage("机器人未初始化!")
            return
        controller_jogger = RPS.ControllerJogger(controller)
        if controller_jogger.StartJog():
            controller_jogger.UpdateSpeedRatio(
                self.speed_ratio_slider.value()/100)
            self.jog_joint_group.setEnabled(True)
            self.jog_cart_group.setEnabled(True)
            self.__SetEnabledButtonsWhenJogging(False)
            self.app_data['controller_jogger'] = controller_jogger
            self.status_bar.showMessage("点动模式开启成功")
        else:
            self.status_bar.showMessage("点动模式开启失败!")

    def __StopJogRobot(self):
        self.jog_joint_group.setEnabled(False)
        self.jog_cart_group.setEnabled(False)
        self.__SetEnabledButtonsWhenJogging(True)
        controller_jogger = self.app_data.get('controller_jogger')
        if controller_jogger is not None:
            controller_jogger.StopJog()
            del self.app_data['controller_jogger']
        self.status_bar.showMessage("点动模式已关闭")

    def __StartDragRobot(self):
        controller = self.app_data.get('controller')
        if controller is not None:
            self.ivis.StartDragger(controller.GetManipulator())
            self.__SetEnableButtonsWhenDragging(False)

    def __StopDragRobot(self):
        self.ivis.StopDragger()
        self.__SetEnableButtonsWhenDragging(True)

    def __Connect(self):
        controller = self.app_data.get('controller')
        if controller is not None:
            if controller.Connect(self.robot_ip_editor.text()) == RCI.RVSReturn_Success:
                self.status_bar.showMessage("连接成功!")
            else:
                self.status_bar.showMessage("连接失败!")

    def __Disconnect(self):
        controller = self.app_data.get('controller')
        if controller is not None:
            if controller.Disconnect() == RCI.RVSReturn_Success:
                self.status_bar.showMessage("断开连接成功!")
            else:
                self.status_bar.showMessage("断开连接失败!")

    def __EnableRobot(self):
        controller = self.app_data.get('controller')
        if controller is not None:
            if controller.EnableRobot() == RCI.RVSReturn_Success:
                self.status_bar.showMessage("使能成功!")
            else:
                self.status_bar.showMessage("使能失败!")

    def __DisableRobot(self):
        controller = self.app_data.get('controller')
        if controller is not None:
            if controller.DisableRobot() == RCI.RVSReturn_Success:
                self.status_bar.showMessage("下电成功!")
            else:
                self.status_bar.showMessage("下电失败!")

    def __SetSpeedRatio(self, v: int):
        controller = self.app_data.get('controller')
        controller.SetSpeedRatio(v/100)
        controller_jogger = self.app_data.get('controller_jogger')
        if controller_jogger is not None:
            controller_jogger.UpdateSpeedRatio(v/100)
        self.robot_speed_ratio.setText(f"{v}%")

    def __UpdateTCP(self):
        controller = self.app_data.get('controller')
        if controller is not None:
            tcp_init = controller.GetTCP()
            PoseWidget.GetPoseWidget(tcp_init, title="Set TCP",
                                     parent=self, confirm_cb=controller.SetTCP)

    def closeEvent(self, event: PySide2.QtGui.QCloseEvent) -> None:
        logger.debug("Robot Control Widget is closed!")
        self.__StopJogRobot()
        self.__StopDragRobot()
        return super().closeEvent(event)


class ProgramDockPanel(QtWidgets.QDockWidget):
    waypoint_seq = 1

    NODE_TYPE_SCRIPT_CONTAINER = 1  # root item
    NODE_TYPE_TRAJ_DATA = RPS.UICmdType_Traj
    NODE_TYPE_WAYPOINT = 100

    def __init__(self, main_win: MainWindow,  parent: typing.Optional[PySide2.QtWidgets.QWidget] = None) -> None:
        super().__init__(parent=parent)
        self.setObjectName("ProgramPanel")
        self.setStyleSheet(Read(RESOURCES_DIR/"Qss/ProgramPanel.qss"))

        self.setWindowTitle("程序")
        self.app_data = main_win.app_data
        self.status_bar = main_win.statusBar()
        self.ivis = main_win.ivis

        # 程序创建、导入、保存
        self.main_layout = QtWidgets.QVBoxLayout()
        hbox = QtWidgets.QHBoxLayout()
        self.new_prog_btn = QtWidgets.QPushButton("新建程序")
        self.new_prog_btn.clicked.connect(self.__NewProg)
        self.load_prog_btn = QtWidgets.QPushButton("加载程序")
        self.load_prog_btn.clicked.connect(self.LoadProgFromFile)
        self.save_prog_btn = QtWidgets.QPushButton("保存程序")
        self.save_prog_btn.clicked.connect(self.__SaveProg)
        hbox.addWidget(self.new_prog_btn)
        hbox.addWidget(self.load_prog_btn)
        hbox.addWidget(self.save_prog_btn)
        self.main_layout.addLayout(hbox)

        # 不同的一组命令组成一个页签
        self.prog_cmd_tab_widget = QtWidgets.QTabWidget()
        self.main_layout.addWidget(self.prog_cmd_tab_widget)
        commonly_used_tab = QtWidgets.QWidget()
        sub_layout = QtWidgets.QGridLayout()
        self.traj_edit_btn = QtWidgets.QPushButton("轨迹")
        self.traj_edit_btn.clicked.connect(self.__AddTrajBlock)
        self.wait_btn = QtWidgets.QPushButton("等待")
        self.wait_btn.clicked.connect(self.__AddWaitBlock)
        sub_layout.addWidget(self.traj_edit_btn, 0, 0)
        sub_layout.addWidget(self.wait_btn, 0, 1)
        commonly_used_tab.setLayout(sub_layout)
        self.prog_cmd_tab_widget.addTab(commonly_used_tab, "常用")
        self.prog_cmd_tab_widget.setEnabled(False)

        ########## Program Show ########
        self.prog_tree_widget = QtWidgets.QTreeWidget()
        self.prog_tree_widget.setObjectName("ProgramTree")
        self.prog_tree_widget.setHeaderLabel("程序")
        self.prog_tree_widget.currentItemChanged.connect(
            self.__ShowSelectedBlockParameters)
        self.prog_tree_widget.setContextMenuPolicy(Qt.CustomContextMenu)
        self.prog_tree_widget.customContextMenuRequested.connect(
            self.__ShowProgTreeContextMenu)
        self.prog_tree_widget.setEditTriggers(
            QtWidgets.QTreeWidget.EditTrigger.DoubleClicked)
        self.prog_tree_widget.itemChanged.connect(
            self.__UpdateProgTreeItemData)
        self.main_layout.addWidget(self.prog_tree_widget)

        ############## Program command paramters ########
        self.parameter_widget = QtWidgets.QGroupBox("程序命令参数设置")
        self.main_layout.addWidget(self.parameter_widget)

        self.main_layout.addStretch(1)
        #####################
        widget = QtWidgets.QWidget()
        widget.setLayout(self.main_layout)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidget(widget)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.setWidget(scroll)

        self.app_data['traj_visuals'] = {}
        self.app_data['waypoint_visuals'] = {}

    def __NewProg(self):
        prog_name = f"程序{len(self.app_data.get('programs'))}"
        self.prog: RPS.ScriptContainer = RPS.ScriptContainer.Create(prog_name)
        self.prog_top_level_item = QtWidgets.QTreeWidgetItem(
            self.prog_tree_widget, self.NODE_TYPE_SCRIPT_CONTAINER)
        self.prog_top_level_item.setText(0, prog_name)
        self.prog_top_level_item.setToolTip(0, "双击编辑名称")
        self.prog_top_level_item.setData(0, Qt.UserRole, self.prog)
        self.prog_top_level_item.setIcon(0, QtGui.QIcon(
            str(RESOURCES_DIR/"Icons/Program.svg")))
        self.prog_top_level_item.setFlags(
            self.prog_top_level_item.flags() | Qt.ItemIsEditable)
        self.prog_tree_widget.setCurrentItem(self.prog_top_level_item)
        self.prog_top_level_item.setExpanded(True)
        self.app_data['programs'].append(self.prog)
        self.prog_cmd_tab_widget.setEnabled(True)

    def LoadProgFromScriptContainer(self, prog: RPS.ScriptContainer):
        self.__NewProg()
        blocks = prog.GetScriptBlocks()
        ref_block = None
        for block in blocks:
            block_copy = block.Copy()
            self.__InsertBlock(block_copy, ref_block)
            if block.cmd_type == RPS.UICmdType_Traj:
                parent = self.prog_tree_widget.currentItem()
                traj_data = block_copy.traj_data
                waypoints = traj_data.GetWaypoints()
                self.waypoint_seq += len(waypoints)
                for waypoint in waypoints:
                    logger.info(f"load waypoint {waypoint.name}")
                    item = QtWidgets.QTreeWidgetItem(
                        parent, self.NODE_TYPE_WAYPOINT)
                    item.setFlags(item.flags() | Qt.ItemIsEditable)
                    item.setText(0, waypoint.name)
                    item.setIcon(0, QtGui.QIcon(
                        str(RESOURCES_DIR / "Icons/Waypoint.svg")))
                    item.setData(0, Qt.UserRole, waypoint)
            ref_block = block_copy
        self.prog_top_level_item.setText(0, prog.GetName())
        logger.info(f"loaded program:\n====\n{self.prog.Repr()}\n====")
        self.prog_cmd_tab_widget.setEnabled(True)

    def LoadProgFromFile(self, fname: str = ""):
        if not fname:
            fname, _ = QtWidgets.QFileDialog.getOpenFileName(
                caption="选择一个文件", dir=str(DATADIR),
                filter="程序(*.json)",
                options=QtWidgets.QFileDialog.DontUseNativeDialog)
        if os.path.exists(fname):
            logger.info(f"loading program from {fname}")
            prog_name = pathlib.Path(fname).stem
            prog: RPS.ScriptContainer = RPS.ScriptContainer.Create(prog_name)
            with open(fname, "r") as f:
                prog_str = f.read()
                prog.FromJson(prog_str)
            self.LoadProgFromScriptContainer(prog)

    def __SaveProg(self):
        fname, _ = QtWidgets.QFileDialog.getSaveFileName(
            caption="Select a file", dir=str(DATADIR),
            filter="程序(*.json)",
            options=QtWidgets.QFileDialog.DontUseNativeDialog)
        if fname:
            fname = str(pathlib.Path(fname).with_suffix(".json"))
            with open(fname, "w") as f:
                prog_str = self.prog.ToJson()
                f.write(prog_str)

    def __ShowSelectedBlockParameters(self):
        item = self.prog_tree_widget.currentItem()
        if item is None:  # show empty panel
            widget = QtWidgets.QGroupBox("程序命令参数设置")
            self.main_layout.replaceWidget(self.parameter_widget, widget)
            self.parameter_widget = widget
        elif item.type() == self.NODE_TYPE_SCRIPT_CONTAINER:
            self.__ShowMainProgParameters()
            self.prog = item.data(0, Qt.UserRole)
        elif item.type() == self.NODE_TYPE_WAYPOINT:
            self.__ShowTrajBlockParameters()
        elif item.type() == RPS.UICmdType_Traj:
            self.__ShowTrajBlockParameters()
        elif item.type() == RPS.UICmdType_Wait:
            self.__ShowWaitBlockParameters()
        else:
            logger.info(
                f"block {block.Repr()} is not support to be edited!")

    def __GetBlockIconFile(self, block: RPS.ScriptBlock) -> str:
        if block.cmd_type == RPS.UICmdType_Traj:
            return RESOURCES_DIR / "Icons/Trajectory.svg"
        elif block.cmd_type == RPS.UICmdType_Wait:
            return RESOURCES_DIR / "Icons/Wait.svg"
        else:
            return ""

    def __InsertBlock(self, block: RPS.ScriptBlock, ref_block: RPS.ScriptBlock = None):
        parent = self.prog_tree_widget.currentItem()
        if parent is None:
            parent = self.prog_top_level_item
        while parent is not None and parent.type() != self.NODE_TYPE_SCRIPT_CONTAINER:
            parent = parent.parent()
        if parent is not None and parent.type() == self.NODE_TYPE_SCRIPT_CONTAINER:
            if ref_block is None:
                ref_block = self.__GetSelectedScriptBlock()
            logger.info(f"insert block {block}, ref_block is {ref_block}")
            container: RPS.ScriptContainer = parent.data(0, Qt.UserRole)
            container.InsertScriptBlock(block, True, ref_block)
            item = QtWidgets.QTreeWidgetItem(None, block.cmd_type)
            item.setText(0, block.Repr())
            item.setData(0, Qt.UserRole, block)
            item.setFlags(item.flags() | Qt.ItemIsEditable)
            icon_file = self.__GetBlockIconFile(block)
            if icon_file:
                item.setIcon(0, QtGui.QIcon(str(icon_file)))

            current_tree_item = self.__GetSelectedScriptBlockTreeItem()
            if current_tree_item:
                idx = parent.indexOfChild(current_tree_item)
                parent.insertChild(idx+1, item)
            else:
                parent.addChild(item)

            logger.info(f"Add {block.Repr()} to program")
            self.prog_tree_widget.setCurrentItem(item)

    def __AddWaitBlock(self):
        block: RPS.ScriptBlock = RPS.ScriptBlock.CreateBlockLogicWait(1)
        self.__InsertBlock(block)

    def __AddTrajBlock(self):
        block = RPS.ScriptBlock.CreateBlockTraj()
        self.__InsertBlock(block)

    def __InsertWaypoint(self):
        controller = self.app_data.get('controller')
        ret1, q = controller.GetJointPosition()
        ret2, p = controller.GetPose()
        if ret1 != RCI.RVSReturn_Success or ret2 != RCI.RVSReturn_Success:
            logger.error(f"Failed to get robot position! Maybe not connect!")
            self.status_bar.showMessage("插入路径点失败！不能获取机器人位置，检查机器人连接是否正常！")
            return

        index = 0
        parent = self.prog_tree_widget.currentItem()
        if parent.type() == self.NODE_TYPE_WAYPOINT:
            child = parent
            parent = child.parent()
            index = parent.indexOfChild(child) + 1  # after current waypoint
        block: RPS.ScriptBlock = parent.data(0, Qt.UserRole)
        if block.cmd_type == RPS.UICmdType_Traj:
            traj_data = block.traj_data
            waypoint = RPS.Waypoint(p, q)
            waypoint.name = self.__GetUniqueWaypointName()
            waypoint = traj_data.InsertWaypoint(index, waypoint)
            item = QtWidgets.QTreeWidgetItem(None, self.NODE_TYPE_WAYPOINT)
            item.setFlags(item.flags() | Qt.ItemIsEditable)
            item.setText(0, waypoint.name)
            item.setData(0, Qt.UserRole, waypoint)
            item.setIcon(0, QtGui.QIcon(
                str(RESOURCES_DIR / "Icons/Waypoint.svg")))
            parent.insertChild(index, item)
            self.prog_tree_widget.setCurrentItem(item)

    def __UpdateProgTreeItemData(self, item: QtWidgets.QTreeWidgetItem, column: int):
        if item.type() == self.NODE_TYPE_WAYPOINT and column == 0:
            waypoint: RPS.Waypoint = item.data(0, Qt.UserRole)
            if waypoint:
                new_name = item.text(column)
                logger.info(f"update waypoint {waypoint.name} to {new_name}")
                waypoint.name = new_name
        elif item.type() == RPS.UICmdType_Traj:
            block = self.__GetSelectedScriptBlock()
            if block:
                traj_data = block.traj_data
                if traj_data:
                    logger.info(
                        f"update trajectory {traj_data.GetName()} name to {item.text(0)}")
                    traj_data.SetName(item.text(0))
        elif item.type() == self.NODE_TYPE_SCRIPT_CONTAINER:
            self.prog.SetName(item.text(0))

    def __ShowMainProgParameters(self):
        item = self.prog_tree_widget.currentItem()
        if item.type() == self.NODE_TYPE_SCRIPT_CONTAINER:
            prog: RPS.ScriptContainer = item.data(0, Qt.UserRole)

            layout = QtWidgets.QGridLayout()
            self.exec_prog_btn = QtWidgets.QPushButton("运行")
            self.exec_prog_btn.clicked.connect(self.__ExecProg)
            self.pause_prog_btn = QtWidgets.QPushButton("暂停")
            self.pause_prog_btn.clicked.connect(self.__StopExecProg)
            self.continue_prog_btn = QtWidgets.QPushButton("继续")
            self.continue_prog_btn.clicked.connect(self.__ContinueExecProg)
            layout.addWidget(self.exec_prog_btn, 0, 0)
            layout.addWidget(self.pause_prog_btn, 0, 1)
            layout.addWidget(self.continue_prog_btn, 0, 2)

            group = QtWidgets.QGroupBox("程序命令参数设置")
            group.setLayout(layout)
            self.main_layout.replaceWidget(self.parameter_widget, group)
            self.parameter_widget.deleteLater()
            self.parameter_widget = group

    def __ShowTrajBlockParameters(self):
        item = self.prog_tree_widget.currentItem()
        if item.type() != RPS.UICmdType_Traj and item.type() != self.NODE_TYPE_WAYPOINT:
            return

        block = self.__GetSelectedScriptBlock()
        traj_data: RPS.TrajectoryData = block.traj_data

        grid = QtWidgets.QGridLayout()
        self.insert_waypoint_btn = QtWidgets.QPushButton("插入路径点")
        self.insert_waypoint_btn.clicked.connect(self.__InsertWaypoint)

        self.plan_traj_btn = QtWidgets.QPushButton("规划并显示")
        self.plan_traj_btn.clicked.connect(self.__PlanAndShowTraj)

        self.verify_traj_btn = QtWidgets.QPushButton("验证轨迹")
        self.verify_traj_btn.clicked.connect(self.__VerifyTraj)

        # TODO: change to jog trajectory later
        self.exec_traj_btn = QtWidgets.QPushButton("执行轨迹")
        self.exec_traj_btn.clicked.connect(self.__ExecTraj)

        self.clear_traj_visual_btn = QtWidgets.QPushButton("清除显示")
        self.clear_traj_visual_btn.clicked.connect(self.__ClearTrajVisual)

        grid.addWidget(self.insert_waypoint_btn, 0, 0)
        grid.addWidget(self.plan_traj_btn, 1, 0)
        grid.addWidget(self.verify_traj_btn, 1, 1)
        grid.addWidget(self.exec_traj_btn, 2, 0)
        grid.addWidget(self.clear_traj_visual_btn, 2, 1)

        # 运动类型
        form = QtWidgets.QFormLayout()
        self.blend_tolerance_input = QtWidgets.QDoubleSpinBox()
        self.blend_tolerance_input.setMinimum(0.0)
        self.blend_tolerance_input.setMaximum(float('inf'))
        self.blend_tolerance_input.setDecimals(3)
        self.blend_tolerance_input.setSingleStep(0.1)
        self.blend_tolerance_input.setValue(traj_data.GetBlendTolerance())
        self.blend_tolerance_input.valueChanged.connect(
            self.__UpdateTrajBlockBlendTolerance)
        form.addRow("过度圆角半径", self.blend_tolerance_input)
        self.motion_types_com_box = QtWidgets.QComboBox()
        self.motion_types_com_box.addItems(["MoveJ", "MoveL", "MoveC"])
        self.motion_types_com_box.setCurrentIndex(
            traj_data.GetMotionType())
        self.motion_types_com_box.currentIndexChanged.connect(
            self.__UpdateTrajBlockMotionType)
        form.addRow("运动类型", self.motion_types_com_box)
        # 轨迹类型
        self.traj_types_com_box = QtWidgets.QComboBox()
        self.traj_types_com_box.addItems(
            ["Trapezoidal", "Totp", "DoubleS", "Toppra", "Totp3"])
        self.traj_types_com_box.setCurrentIndex(
            traj_data.GetTrajType())
        self.traj_types_com_box.currentIndexChanged.connect(
            self.__UpdateTrajBlockTrajType)
        form.addRow("运动类型", self.traj_types_com_box)

        # 是否匀速
        hbox = QtWidgets.QHBoxLayout()
        self.is_constant_vel_check_box = QtWidgets.QCheckBox()
        hbox.addWidget(QtWidgets.QLabel("是否匀速"))
        hbox.addWidget(self.is_constant_vel_check_box)
        # 是否整圆
        self.is_full_circle_check_box = QtWidgets.QCheckBox()
        self.is_full_circle_check_box.setChecked(traj_data.GetIsFullCircle())
        self.is_full_circle_check_box.stateChanged.connect(
            self.__UpdateTrajBlockIsFullCircle)
        hbox.addWidget(QtWidgets.QLabel("是否整圆"))
        hbox.addWidget(self.is_full_circle_check_box)
        form.addRow(hbox)

        # 匀速速度
        self.velocity_spin_box = QtWidgets.QDoubleSpinBox()
        self.velocity_spin_box.setMinimum(0.01)
        self.velocity_spin_box.setMaximum(float('inf'))
        self.velocity_spin_box.setDecimals(3)
        self.velocity_spin_box.setSingleStep(0.1)
        self.velocity_spin_box.setValue(traj_data.GetVelocity())
        self.velocity_spin_box.valueChanged.connect(
            self.__UpdateTrajBlockVelocity)
        self.velocity_spin_box.setEnabled(traj_data.GetIsConstantVelocity())
        form.addRow("速度(m/s, rad/s)", self.velocity_spin_box)

        self.is_constant_vel_check_box.stateChanged.connect(
            self.velocity_spin_box.setEnabled)
        self.is_constant_vel_check_box.setChecked(
            traj_data.GetIsConstantVelocity())
        self.is_constant_vel_check_box.stateChanged.connect(
            self.__UpdateTrajBlockIsConstantVel)

        if item.type() == self.NODE_TYPE_WAYPOINT:
            waypoint = item.data(0, Qt.UserRole)
            # 路径点空间类型
            self.waypoint_type_com_box = QtWidgets.QComboBox()
            self.waypoint_type_com_box.addItems(['关节空间', '笛卡尔空间'])
            form.addRow("路径点类型", self.waypoint_type_com_box)

            # 路径点参考坐标系
            hbox = QtWidgets.QHBoxLayout()
            self.update_frame_list_btn = QtWidgets.QPushButton("更新列表")
            self.waypoint_frame_name_combo_box = QtWidgets.QComboBox()
            self.waypoint_frame_name_combo_box.setMaximumWidth(100)
            hbox.addWidget(QtWidgets.QLabel("参考坐标系"))
            hbox.addWidget(self.waypoint_frame_name_combo_box)
            hbox.addWidget(self.update_frame_list_btn)
            form.addRow(hbox)

            # lookat function
            hbox = QtWidgets.QHBoxLayout()
            self.lookat_btn = QtWidgets.QPushButton("LookAt")
            self.lookat_btn.released.connect(self.__MakeWaypointLookAt)
            self.update_lookat_object_list_btn = QtWidgets.QPushButton("更新列表")
            self.lookat_object_combo_box = QtWidgets.QComboBox()

            self.lookat_reverse_x_check_box = QtWidgets.QCheckBox("反转x")
            self.lookat_reverse_x_check_box.stateChanged.connect(
                self.__UpdateLookAtReverseX)
            self.lookat_reverse_x_check_box.setChecked(
                self.app_data.get('lookat_reverse_x', False))

            hbox.addWidget(self.lookat_btn)
            hbox.addWidget(self.lookat_object_combo_box)
            hbox.addWidget(self.update_lookat_object_list_btn)
            hbox.addWidget(self.lookat_reverse_x_check_box)
            form.addRow(hbox)

            form.addRow(GetHorizontalLine())
            q_text = ",".join(
                ["%.2f" % v for v in waypoint.joint.Coeffs()])
            self.waypoint_joint_label = QtWidgets.QLabel(q_text)
            self.waypoint_joint_label.setObjectName("WaypointJointValueLabel")
            p_text = ",".join(
                ["%.2f" % v for v in waypoint.pose.Coeffs()])
            self.waypoint_cart_label = QtWidgets.QLabel(p_text)
            self.waypoint_cart_label.setObjectName("WaypointPoseValueLabel")
            form.addRow("路径点关节值", self.waypoint_joint_label)
            form.addRow("路径点Pose值", self.waypoint_cart_label)

            # 更新Waypoint Type Frame 显示
            self.update_lookat_object_list_btn.clicked.connect(
                self.__UpdateLookAtObjectComboBox)
            self.update_frame_list_btn.clicked.connect(
                self.__UpdateRefFrameComboBox)
            self.update_lookat_object_list_btn.click()
            self.update_frame_list_btn.click()
            self.waypoint_frame_name_combo_box.setCurrentText(
                waypoint.frame_name)
            self.waypoint_type_com_box.setCurrentIndex(waypoint.type)
            self.waypoint_type_com_box.currentIndexChanged.connect(
                self.__UpdateWaypointType)
            self.waypoint_frame_name_combo_box.currentTextChanged.connect(
                self.__UpdateWaypointFrameName)

        grid.addLayout(form, grid.rowCount(), 0, 1, 2)
        group = QtWidgets.QGroupBox("程序命令参数设置")
        group.setLayout(grid)
        self.main_layout.replaceWidget(self.parameter_widget, group)
        self.parameter_widget.deleteLater()  # delete old widget
        self.parameter_widget = group

    def __UpdateLookAtObjectComboBox(self):
        self.lookat_object_combo_box.clear()
        bodies = self.ivis.env.GetBodies()
        for body in bodies:
            if type(body) == RCI.Multibody:  # filter out Manipulator and RobotModel
                self.lookat_object_combo_box.addItem(body.GetName())
                self.lookat_object_combo_box.setItemData(0, body, Qt.UserRole)
                logger.debug(f'add {body.GetName()} to object combo box list')

    def __UpdateRefFrameComboBox(self):
        text = self.waypoint_frame_name_combo_box.currentText()
        self.waypoint_frame_name_combo_box.clear()
        bodies = self.ivis.env.GetBodies()
        self.waypoint_frame_name_combo_box.addItem("env_world")
        for body in bodies:
            if type(body) == RCI.Multibody:
                self.waypoint_frame_name_combo_box.addItem(
                    body.GetUniqueName())
        self.waypoint_frame_name_combo_box.setCurrentText(text)

    def __UpdateLookAtReverseX(self, v: bool):
        self.app_data['lookat_reverse_x'] = v

    def __MakeWaypointLookAt(self, waypoint: RPS.Waypoint = None):
        if waypoint is None:
            item = self.prog_tree_widget.currentItem()
            if not item:
                return
            waypoint: RPS.Waypoint = item.data(0, Qt.UserRole)
        if waypoint is None:
            return
        controller = self.app_data.get('controller')
        kin_solver = controller.GetKinSolver()

        body = self.lookat_object_combo_box.itemData(0, Qt.UserRole)
        if not body:
            return
        body_pose = body.GetBaseTransformation()
        body_center = body_pose.Translation()
        ret, waypoint_pose = waypoint.GetPose(kin_solver, self.ivis.env)
        if ret != RCI.RVSReturn_Success:
            logger.info(f"Failed to get waypoint {waypoint.name} pose!")
            return
        waypoint_center = waypoint_pose.Translation()
        rz = body_center - waypoint_center
        rz /= np.linalg.norm(rz)
        # TODO: handle when rz is same with rx
        # rx = np.array([1., 0., 0.])
        rx = [rz[0], rz[1], rz[2] + np.sqrt(2)]
        rx /= np.linalg.norm(rx)
        if self.app_data.get('lookat_reverse_x', False):
            rx *= -1.0
        ry = np.cross(rz, rx)
        rx = np.cross(ry, rz)
        rot = RCI.Rotation(np.c_[rx, ry, rz])
        waypoint.pose = body.GetBaseTransformation().Inverse() * \
            RCI.Pose(waypoint_center, rot.Coeffs())
        waypoint.type = RPS.WaypointType_Cartesian
        waypoint.frame_name = body.GetUniqueName()
        logger.info(f"make waypoint {waypoint.name} look at {body}")
        self.__UpdateWaypointInfoDisplay()

    def __UpdateTrajBlockBlendTolerance(self, v: float):
        block = self.__GetSelectedScriptBlock()
        traj_data: RPS.TrajectoryData = block.traj_data
        if traj_data:
            traj_data.SetBlendTolerance(v)

    def __UpdateTrajBlockVelocity(self, v: float):
        block = self.__GetSelectedScriptBlock()
        traj_data: RPS.TrajectoryData = block.traj_data
        if traj_data:
            traj_data.SetVelocity(v)

    def __UpdateTrajBlockIsConstantVel(self, v: bool):
        block = self.__GetSelectedScriptBlock()
        traj_data: RPS.TrajectoryData = block.traj_data
        if traj_data:
            traj_data.SetIsConstantVelocity(v)

    def __UpdateTrajBlockIsFullCircle(self, v: bool):
        block = self.__GetSelectedScriptBlock()
        traj_data: RPS.TrajectoryData = block.traj_data
        if traj_data:
            traj_data.SetIsFullCircle(v)

    def __UpdateTrajBlockTrajType(self, idx):
        block = self.__GetSelectedScriptBlock()
        traj_data: RPS.TrajectoryData = block.traj_data
        if traj_data:
            traj_data.SetTrajType(RCI.TrajType(idx))

    def __UpdateTrajBlockMotionType(self, idx):
        block = self.__GetSelectedScriptBlock()
        traj_data: RPS.TrajectoryData = block.traj_data
        if traj_data:
            traj_data.SetMotionType(RPS.MotionType(idx))

    def __UpdateWaypointType(self, idx):
        item = self.prog_tree_widget.currentItem()
        if item.type() == self.NODE_TYPE_WAYPOINT:
            waypoint: RPS.Waypoint = item.data(0, Qt.UserRole)
            waypoint.type = RPS.WaypointType(idx)

    def __UpdateWaypointFrameName(self, frame_name):
        if not frame_name:
            frame_name = "env_world"
        item = self.prog_tree_widget.currentItem()
        if item.type() == self.NODE_TYPE_WAYPOINT:
            waypoint: RPS.Waypoint = item.data(0, Qt.UserRole)
            if frame_name == waypoint.frame_name:
                return
            if frame_name == 'env_world':
                ref_pose = RCI.Pose()
            else:
                body = self.ivis.env.GetBody(frame_name)
                ref_pose = body.GetBaseTransformation()
            controller = self.app_data['controller']
            kin_solver = controller.GetKinSolver()
            ret, old_pose = waypoint.GetPose(kin_solver, self.ivis.env)
            if ret != RCI.RVSReturn_Success:
                return
            new_pose = ref_pose.Inverse() * old_pose
            waypoint.frame_name = frame_name
            waypoint.pose = new_pose
            waypoint.type = RPS.WaypointType_Cartesian
            logger.info(
                f"update waypoint {waypoint.name} frame to {waypoint.frame_name}")

    def __UpdateWaypointJointAndPoseToRobotCurrentPosition(self):
        item = self.prog_tree_widget.currentItem()
        if item.type() == self.NODE_TYPE_WAYPOINT:
            controller = self.app_data.get('controller')
            _, q = controller.GetJointPosition()
            _, p = controller.GetPose()
            waypoint: RPS.Waypoint = item.data(0, Qt.UserRole)
            if waypoint is not None:
                waypoint.pose = p
                waypoint.joint = q
                self.__UpdateWaypointInfoDisplay()

    def __ShowWaitBlockParameters(self):
        item = self.prog_tree_widget.currentItem()
        if item.type() == RPS.UICmdType_Wait:
            block: RPS.ScriptBlock = item.data(0, Qt.UserRole)

            form = QtWidgets.QFormLayout()
            self.wait_time_spin_box = QtWidgets.QDoubleSpinBox()
            self.wait_time_spin_box.setMinimum(0.)
            self.wait_time_spin_box.setMaximum(float("inf"))
            self.wait_time_spin_box.setValue(block.timeout)
            self.wait_time_spin_box.setDecimals(3)
            self.wait_time_spin_box.valueChanged.connect(
                self.__UpdateWaitBlockTimeout)
            form.addRow("等待时间(s)", self.wait_time_spin_box)

            group = QtWidgets.QGroupBox("程序命令参数设置")
            group.setLayout(form)
            self.main_layout.replaceWidget(self.parameter_widget, group)
            self.parameter_widget.deleteLater()
            self.parameter_widget = group

    def __UpdateWaypointInfoDisplay(self, waypoint=None):
        item = self.prog_tree_widget.currentItem()
        if item.type() == self.NODE_TYPE_WAYPOINT:
            waypoint: RPS.Waypoint = item.data(0, Qt.UserRole)
        q = waypoint.joint
        p = waypoint.pose
        q_text = ",".join(
            ["%.2f" % v for v in q.Coeffs()])
        p_text = ",".join(
            ["%.2f" % v for v in p.Coeffs()])
        self.waypoint_type_com_box.setCurrentIndex(waypoint.type)
        self.waypoint_frame_name_combo_box.setCurrentText(waypoint.frame_name)
        self.waypoint_joint_label.setText(q_text)
        self.waypoint_cart_label.setText(p_text)
        self.__UpdateWaypointVisual(waypoint)

    def __UpdateWaitBlockTimeout(self):
        item = self.prog_tree_widget.currentItem()
        if item.type() == RPS.UICmdType_Wait:
            block: RPS.ScriptBlock = item.data(0, Qt.UserRole)
            block.timeout = self.wait_time_spin_box.value()
            item.setText(0, block.Repr())

    def __DeleteWaypoint(self):
        item = self.prog_tree_widget.currentItem()
        if item.type() == self.NODE_TYPE_WAYPOINT:
            waypoint = item.data(0, Qt.UserRole)
            traj_data: RPS.TrajectoryData = waypoint.GetTrajData()
            if traj_data:
                traj_data.DeleteWaypoint(waypoint)
            parent = item.parent()
            parent.removeChild(item)
            self.__DeleteWaypointVisual(waypoint)

    def __UpdateWaypointVisual(self, waypoint=None):
        if waypoint is None:
            item = self.prog_tree_widget.currentItem()
            if item.type() == self.NODE_TYPE_WAYPOINT:
                waypoint = item.data(0, Qt.UserRole)
        waypoint_visual = self.app_data['waypoint_visuals'].get(
            waypoint)
        if waypoint_visual:
            self.__DeleteWaypointVisual(waypoint)
            self.__ToggleWaypointVisual(waypoint)

    def __MoveToWaypoint(self): 
        controller = self.app_data.get('controller')
        kin_solver = controller.GetKinSolver()
        item = self.prog_tree_widget.currentItem()
        if item.type() == self.NODE_TYPE_WAYPOINT:
            waypoint: RPS.Waypoint = item.data(0, Qt.UserRole)
            res, q = waypoint.GetJoint(kin_solver, self.ivis.env) 
            if res == RCI.RVSReturn_Success:
                controller.MoveJoints(q, False)

    def __DeleteBlock(self, block=None):
        if block is None:
            block = self.__GetSelectedScriptBlock()
        # clear trajectory visual
        if block.cmd_type == RPS.UICmdType_Traj:
            traj_data = block.traj_data
            traj_visual = self.app_data['traj_visuals'].get(traj_data)
            if traj_visual is not None:
                del self.app_data['traj_visuals'][traj_data]
                self.ivis.iviewer.DeleteShape(traj_visual)
            for waypoint in traj_data.GetWaypoints():
                self.__DeleteWaypointVisual(waypoint)

        container = block.GetParentContainer()
        container.Delete(block)
        item = self.prog_tree_widget.currentItem()
        if item.type() == self.NODE_TYPE_WAYPOINT:
            item = item.parent()
        parent = item.parent()
        if parent:
            parent.removeChild(item)

    def __ShowProgTreeContextMenu(self):
        item = self.prog_tree_widget.currentItem()
        if item.type() == self.NODE_TYPE_WAYPOINT:
            menu = QtWidgets.QMenu("轨迹路径点")
            menu.addAction("运动至", self.__MoveToWaypoint)
            menu.addAction("删除", self.__DeleteWaypoint)
            menu.addAction(
                "更新", self.__UpdateWaypointJointAndPoseToRobotCurrentPosition)
            menu.addAction("更新显示", self.__UpdateWaypointInfoDisplay)
            menu.addAction("显示/隐藏", self.__ToggleWaypointVisual)
            menu.exec_(QtGui.QCursor.pos())
        elif item.type() == self.NODE_TYPE_SCRIPT_CONTAINER:
            menu = QtWidgets.QMenu("程序")
            menu.addAction("运行", self.__ExecProg)
            menu.addAction("删除", self.__DeleteProg)
            menu.exec_(QtGui.QCursor.pos())
        else:  # item.type() == self.NODE_TYPE_TRAJ_DATA:
            menu = QtWidgets.QMenu("程序块")
            menu.addAction("单步运行", self.__StepExecProg)
            menu.addAction("删除", self.__DeleteBlock)
            menu.exec_(QtGui.QCursor.pos())

    def __DeleteWaypointVisual(self, waypoint=None):
        if waypoint is None:
            waypoint = self.__GetSelectedWaypoint()
        waypoint_visual = self.app_data['waypoint_visuals'].get(
            waypoint)
        if waypoint_visual:
            self.ivis.iviewer.DeleteShape(waypoint_visual)
            del self.app_data['waypoint_visuals'][waypoint]

    def __ToggleWaypointVisual(self, waypoint=None):
        if waypoint is None:
            waypoint = self.__GetSelectedWaypoint()

        waypoint_visual = self.app_data['waypoint_visuals'].get(
            waypoint)
        if waypoint_visual:
            self.__DeleteWaypointVisual(waypoint)
        else:
            controller = self.app_data['controller']
            kin_solver = controller.GetKinSolver()
            ret, pose = waypoint.GetPose(kin_solver, self.ivis.env)
            if ret == RCI.RVSReturn_Success:
                waypoint_visual = self.ivis.iviewer.Axes(
                    positions=pose.Translation(), quats=pose.Quat())
                self.app_data['waypoint_visuals'][waypoint] = waypoint_visual

    def __GetSelectedScriptBlockTreeItem(self) -> QtWidgets.QTreeWidgetItem:
        """Get the selected TreeItem which holds a ScriptBlock. There are some special cases:
        - if current item is a waypoint node, return the trajectory data tree item; because a waypoint is not a ScriptBlock
        - if current item is program root, return None
        - if nothing selected, return None

        :return: [description]
        :rtype: QtWidgets.QTreeWidgetItem
        """
        item = self.prog_tree_widget.currentItem()
        if item is None:
            return None
        # if we add if/else/for/while, the code may need to be changed
        if item.type() == self.NODE_TYPE_SCRIPT_CONTAINER:
            return None
        if item.type() == self.NODE_TYPE_WAYPOINT:
            item = item.parent()
        return item

    def __GetSelectedScriptBlock(self) -> RPS.ScriptBlock:
        item = self.__GetSelectedScriptBlockTreeItem()
        if item:
            block = item.data(0, Qt.UserRole)
            return block
        return None

    def __GetSelectedWaypoint(self) -> RPS.Waypoint:
        item = self.prog_tree_widget.currentItem()
        if item.type() == self.NODE_TYPE_WAYPOINT:
            waypoint = item.data(0, Qt.UserRole)
            return waypoint
        return None

    def __DeleteProg(self):
        item = self.prog_tree_widget.currentItem()
        if item.type() == self.NODE_TYPE_SCRIPT_CONTAINER:
            prog = item.data(0, Qt.UserRole)
            for block in prog.GetScriptBlocks():
                self.__DeleteBlock(block)
            self.app_data['programs'].remove(prog)
            index = self.prog_tree_widget.indexOfTopLevelItem(item)
            self.prog_tree_widget.takeTopLevelItem(index)
            count = self.prog_tree_widget.topLevelItemCount()
            if count > 0:
                self.prog_top_level_item = self.prog_tree_widget.topLevelItem(
                    count - 1)
                self.prog = self.prog_top_level_item.data(0, Qt.UserRole)
            else:
                self.prog_top_level_item = None
                self.prog = None
                self.prog_cmd_tab_widget.setEnabled(False)

    def __StepExecProg(self):
        block = self.__GetSelectedScriptBlock()
        if block:
            controller = self.app_data.get('controller')
            executor: RPS.ScriptExecutor = RPS.ScriptExecutor(
                controller, self.ivis.env)
            executor.ExecuteScriptBlock(block, False)

    def __ExecProg(self):
        self.status_bar.showMessage("执行程序")
        controller = self.app_data.get('controller')
        executor: RPS.ScriptExecutor = RPS.ScriptExecutor(
            controller, self.ivis.env)
        executor.ExecuteScriptContainer(self.prog, False)
        self.app_data['executor'] = executor

    def __StopExecProg(self):
        self.status_bar.showMessage("暂停程序")
        executor = self.app_data.get('executor')
        if executor:
            executor.Terminate()
            self.status_bar.showMessage("暂停执行程序")

    def __ContinueExecProg(self):
        self.status_bar.showMessage("继续程序")
        executor = self.app_data.get('executor')
        if executor:
            executor.ExecuteScriptContainer(self.prog, False, False)
            logger.info("继续执行程序")
            self.status_bar.showMessage("继续执行程序")

    def __GetUniqueWaypointName(self):
        name = f"waypoint_{self.waypoint_seq}"
        self.waypoint_seq += 1
        return name

    def __PlanAndShowTraj(self):
        controller = self.app_data.get('controller')
        kin_solver = controller.GetKinSolver()
        block = self.__GetSelectedScriptBlock()
        if block.cmd_type == RPS.UICmdType_Traj:
            traj_data: RPS.TrajectoryData = block.traj_data
            _, q = controller.GetJointPosition()
            if traj_data.GetMotionType() == RPS.MotionType_MoveJ:
                traj = traj_data.GenerateJointTrajectory(
                    kin_solver, self.ivis.env)
            else:
                traj = traj_data.GenerateCartesianTrajectory(
                    kin_solver, self.ivis.env)
            if self.app_data['traj_visuals'].get(traj_data):
                self.ivis.iviewer.DeleteShape(self.app_data['traj_visuals'].get(traj_data))

            traj_visual = self.ivis.DrawTraj(
                traj, kin_solver, length=0.05, line_width=1, points_num=50)
            self.app_data['traj_visuals'][traj_data] = traj_visual
            for waypoint in traj_data.GetWaypoints():
                self.__DeleteWaypointVisual(waypoint)
                self.__ToggleWaypointVisual(waypoint)
            item = self.prog_tree_widget.currentItem()
            if item.type() == self.NODE_TYPE_WAYPOINT:
                item = item.parent()
            if item.type() == self.NODE_TYPE_TRAJ_DATA:
                item.setData(1, Qt.UserRole, traj)
                logger.info(f"store traj {traj} to tree item: {item.text(0)}")

    def __VerifyTraj(self):
        item = self.prog_tree_widget.currentItem()
        if item.type() == self.NODE_TYPE_WAYPOINT:
            item = item.parent()
        logger.info(f"verify trajectory...")
        traj = item.data(1, Qt.UserRole)
        if traj is None:
            logger.error(f"compute trajectroy first!")
            return

        controller = self.app_data.get('controller')
        ret, q_seed = controller.GetJointPosition()
        if ret != RCI.RVSReturn_Success:
            q_seed = controller.GetManipulator().GetHomePosition()
        kin_solver = controller.GetKinSolver()
        ret, ratio = RPS.CheckTrajectoryReachability(traj, kin_solver, q_seed)
        if ret != RCI.RVSReturn_Success:
            self.status_bar.showMessage(f"轨迹包含不能到达的点： {ret}")
        else:
            self.status_bar.showMessage(f"轨迹验证成功！")

    def __ClearTrajVisual(self):
        block = self.__GetSelectedScriptBlock()
        if block is not None and block.cmd_type == RPS.UICmdType_Traj:
            traj_data: RPS.TrajectoryData = block.traj_data
            if self.app_data['traj_visuals'].get(traj_data):
                self.ivis.iviewer.DeleteShape(self.app_data['traj_visuals'].get(traj_data))
                del self.app_data['traj_visuals'][traj_data]
            for waypoint in traj_data.GetWaypoints():
                self.__DeleteWaypointVisual(waypoint)

    def __ExecTraj(self):
        item = self.prog_tree_widget.currentItem()
        if item.type() == self.NODE_TYPE_WAYPOINT:
            item = item.parent()
        traj = item.data(1, Qt.UserRole)
        controller = self.app_data.get('controller')
        if traj is not None:
            ret = controller.ExecuteTrajectory(traj, wait=False)
            if ret != RCI.RVSReturn_Success:
                self.status_bar.showMessage(f"执行轨迹失败！请尝试先验证轨迹，或将机器人移动到轨迹起点！")


class PropertyDockPanel(QtWidgets.QDockWidget):
    def __init__(self, main_window: MainWindow, parent=None):
        super().__init__(parent=parent)
        self.setWindowTitle("属性")
        self.status_bar = main_window.statusBar()
        self.ivis = main_window.ivis

        widget = QtWidgets.QWidget()
        self.setWidget(widget)
        self.main_layout = QtWidgets.QVBoxLayout(widget)

        grid = QtWidgets.QGridLayout()
        self.main_layout.addLayout(grid)
        self.main_layout.addStretch(1)

        self.shape_color_editor = QtWidgets.QPushButton("设置颜色")
        self.shape_color_editor.clicked.connect(self.__SetColor)

        self.shape_position_editor = QtWidgets.QPushButton("设置位置")
        self.shape_position_editor.clicked.connect(self.__SetPosition)

        self.body_rename_btn = QtWidgets.QPushButton("重命名")
        self.body_rename_btn.clicked.connect(self.__SetName)
        self.body_name_editor = QtWidgets.QLineEdit()

        self.start_drag_body_btn = QtWidgets.QPushButton("拖动物体")
        self.start_drag_body_btn.clicked.connect(self.__StartDragBody)
        self.stop_drag_body_btn = QtWidgets.QPushButton("结束拖动")
        self.stop_drag_body_btn.clicked.connect(self.__StopDragBody)

        name = "请选择物体"
        selected_body = self.ivis.GetSelectedBody()
        if selected_body:
            name = selected_body.GetName()
        self.body_name_editor.setText(name)

        grid.addWidget(self.body_rename_btn, 0, 0)
        grid.addWidget(self.body_name_editor, 0, 1)
        grid.addWidget(self.shape_color_editor, 1, 0)
        grid.addWidget(self.shape_position_editor, 1, 1)
        grid.addWidget(self.start_drag_body_btn, 2, 0)
        grid.addWidget(self.stop_drag_body_btn, 2, 1)

    def mousePressEvent(self, event: PySide2.QtGui.QMouseEvent) -> None:
        selected_body = self.ivis.GetSelectedBody()
        if selected_body:
            self.body_name_editor.setText(selected_body.GetName())
        else:
            self.body_name_editor.setText("请选择物体")
        return super().mousePressEvent(event)

    def __StartDragBody(self):
        selected_body = self.ivis.GetSelectedBody()
        if selected_body:
            self.ivis.StartDragger(selected_body)

    def __StopDragBody(self):
        self.ivis.StopDragger()

    def __SetName(self):
        selected_body = self.ivis.GetSelectedBody()
        if selected_body is None:
            self.status_bar.showMessage("请选择一个物体")
            return

        # NOTE: after theRCI.Multibody is in the environment, you can't rename it unless you take it out.
        self.ivis.env.RemoveBody(selected_body)
        name = self.body_name_editor.text()
        logger.info(f"rename body {selected_body.GetName()} to {name}")
        selected_body.SetName(name)
        self.ivis.env.AddBody(selected_body)

    def __SetColor(self):
        shape: coin.SoShapeKit = self.ivis.selected_node
        if shape is None:
            self.status_bar.showMessage("请选择一个物体")
            return

        color = QtWidgets.QColorDialog.getColor(
            options=QtWidgets.QColorDialog.ShowAlphaChannel | QtWidgets.QColorDialog.DontUseNativeDialog)
        rgba = list(color.getRgbF())
        rgba[-1] = 1-rgba[-1]
        self.ivis.SetColor(shape, rgba)

    def __SetPosition(self):
        shape: coin.SoShapeKit = self.ivis.GetSelectedShape()
        if shape is None:
            self.status_bar.showMessage("请选择一个物体")
            return

        transform: coin.SoTransform = shape.getPart("transform", True)
        init_pose = SoTransformToPose(transform)

        def SetObjectPose(pose):
            body: RCI.Multibody = self.ivis.GetShapeMultibody(shape)
            if body is not None:
                body.SetBaseTransformation(pose)
            else:
                PoseToSoTransform(pose, transform)

        PoseWidget.GetPoseWidget(
            init_pose, parent=self, confirm_cb=SetObjectPose)


class SceneDockPanel(QtWidgets.QDockWidget):
    def __init__(self, main_window: MainWindow, parent=None):
        super().__init__(parent=parent)
        self.setWindowTitle("Scene")
        self.setMinimumWidth(100)

        widget = QtWidgets.QWidget()
        self.setWidget(widget)
        main_layout = QtWidgets.QVBoxLayout(widget)

        self.scene_label = QtWidgets.QLabel("Scene")
        self.scene_label.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        main_layout.addWidget(self.scene_label)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, parent: typing.Optional[PySide2.QtWidgets.QWidget] = None) -> None:
        super().__init__(parent=parent)
        self.setStyleSheet(Read(RESOURCES_DIR/"Qss/MainWindow.qss"))
        self.statusBar().setStyleSheet(Read(RESOURCES_DIR/"Qss/StatusBar.qss"))

        self.app_data = {'controller': None, 'programs': []}

        rect = QtGui.QGuiApplication.primaryScreen().availableGeometry()
        self.setWindowTitle("RobotIVis")
        self.setGeometry(0, 0, rect.width() // 2, rect.height())
        self.setWindowIcon(QtGui.QIcon(str(RESOURCES_DIR / "Icons/Robot.svg")))

        # central widget
        self.ivis = IVis(background="white")
        self.setCentralWidget(self.ivis.iviewer)

        # status bar setting
        self.status_bar_robot_status = QtWidgets.QLabel(
            "<font color='red'>机器人未初始化!</font>")
        self.status_bar_robot_speed_ratio = QtWidgets.QLabel(
            "<font color='blue'> 0%</font>")
        self.statusBar().addPermanentWidget(self.status_bar_robot_status)
        self.statusBar().addPermanentWidget(self.status_bar_robot_speed_ratio)
        self.statusBar().showMessage("已就绪")

        # menu bar setting
        # file menu
        self.file_menu = self.menuBar().addMenu("文件")
        self.file_menu.addAction("打开", self.__Open, "Ctrl+O")
        self.file_menu.addAction("保存", self.__Save, "Ctrl+S")
        self.file_menu.addAction("&Embed", embed, "Ctrl+E")
        # scene menu
        self.scene_menu = self.menuBar().addMenu("场景")
        self.geometry_menu = self.scene_menu.addMenu("几何体")
        self.geometry_menu.addAction("盒子", self.__AddBox)
        self.geometry_menu.addAction("球", self.__AddSphere)

        # view menu
        self.view_menu = self.menuBar().addMenu("视图")
        self.scene_panel = SceneDockPanel(self, parent=self)
        self.addDockWidget(Qt.LeftDockWidgetArea, self.scene_panel)
        self.view_menu.addAction(self.scene_panel.toggleViewAction())

        self.property_panel = PropertyDockPanel(self, parent=self)
        self.view_menu.addAction(self.property_panel.toggleViewAction())
        self.tabifyDockWidget(self.scene_panel, self.property_panel)
        self.property_panel.hide()

        self.program_panel = ProgramDockPanel(self, parent=self)
        self.addDockWidget(Qt.RightDockWidgetArea, self.program_panel)
        self.view_menu.addAction(self.program_panel.toggleViewAction())
        self.program_panel.hide()

        self.robot_panel = None

        # tool bar setting
        # TODO: support OrthogonalView later, now it's PerspectiveView
        self.view_tool_bar = QtWidgets.QToolBar("View")
        self.view_tool_bar.addAction(QtGui.QIcon(
            str(RESOURCES_DIR/"Icons/PerspectiveView.svg")), "PerspectiveView", self.ivis.iviewer.PerspectiveView)
        self.view_tool_bar.addAction(QtGui.QIcon(
            str(RESOURCES_DIR/"Icons/OrthogonalView.svg")), "OrthogonalView", self.ivis.iviewer.OrthogonalView)
        self.view_tool_bar.addAction(QtGui.QIcon(
            str(RESOURCES_DIR/"Icons/ViewAll.svg")), "ViewAll", self.ivis.iviewer.viewAll)
        self.view_tool_bar.addAction(QtGui.QIcon(
            str(RESOURCES_DIR/"Icons/HomeView.svg")), "HomeView", self.ivis.iviewer.HomeView)
        self.view_tool_bar.addAction(QtGui.QIcon(
            str(RESOURCES_DIR/"Icons/ZoomIn.svg")), "ZoomIn", self.ivis.iviewer.ZoomIn)
        self.view_tool_bar.addAction(QtGui.QIcon(
            str(RESOURCES_DIR/"Icons/ZoomOut.svg")), "ZoomOut", self.ivis.iviewer.ZoomOut)
        self.view_tool_bar.addAction(QtGui.QIcon(
            str(RESOURCES_DIR/"Icons/FrontView.svg")), "FrontView", self.ivis.iviewer.FrontView)
        self.view_tool_bar.addAction(QtGui.QIcon(
            str(RESOURCES_DIR/"Icons/BackView.svg")), "BackView", self.ivis.iviewer.BackView)
        self.view_tool_bar.addAction(QtGui.QIcon(
            str(RESOURCES_DIR/"Icons/LeftView.svg")), "TopView", self.ivis.iviewer.LeftView)
        self.view_tool_bar.addAction(QtGui.QIcon(
            str(RESOURCES_DIR/"Icons/RightView.svg")), "RightView", self.ivis.iviewer.RightView)
        self.view_tool_bar.addAction(QtGui.QIcon(
            str(RESOURCES_DIR/"Icons/TopView.svg")), "TopView", self.ivis.iviewer.TopView)
        self.view_tool_bar.addAction(QtGui.QIcon(
            str(RESOURCES_DIR/"Icons/BottomView.svg")), "BottomView", self.ivis.iviewer.BottomView)
        self.addToolBar(self.view_tool_bar)

        self.__UpdateSceneGraphDescription()

        self.startTimer(100, Qt.CoarseTimer)

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        self.ivis.is_dragging = False
        return super().closeEvent(event)

    def timerEvent(self, event: PySide2.QtCore.QTimerEvent) -> None:
        if self.app_data.get('executor'):
            executor: RPS.ScriptExecutor = self.app_data.get('executor')
            if executor.IsBusy():
                self.robot_panel.setEnabled(False)
            else:
                self.robot_panel.setEnabled(True)
        return super().timerEvent(event)

    def __AddBox(self):
        self.ivis.AddBox(length=0.3, width=0.2, height=0.1,
                         pose=RCI.Pose(x=0.3, y=0.3), color=(1., 1., 0.))
        self.__UpdateSceneGraphDescription()

    def __AddSphere(self):
        self.ivis.AddSphere(radius=0.1, pose=RCI.Pose(x=0.5, y=0.5),
                            color=(1., 0., 0.))
        self.__UpdateSceneGraphDescription()

    def __InitRobotPanel(self):
        """NOTE: robot panel is initialized after a robot is loaded
        """
        controller = self.app_data.get('controller')
        if controller is None:
            self.statusBar().showMessage("机器人未导入!")
            logger.error(
                "robot controller is not initialized! please load a robot!")
            return
        if self.robot_panel:
            self.robot_panel.toggleViewAction().toggle()
            return

        self.robot_panel = RobotDockPanel(self, parent=self)
        self.view_menu.addAction(self.robot_panel.toggleViewAction())
        self.tabifyDockWidget(self.scene_panel, self.robot_panel)

    def __UpdateSceneGraphDescription(self):
        if self.scene_panel:
            self.scene_panel.scene_label.setText(
                self.ivis.iviewer.GetSceneDescription())

    def __Open(self, fname=""):
        if not fname:
            fname, _ = QtWidgets.QFileDialog.getOpenFileName(
                caption="Select a file", dir=str(DATADIR),
                filter="Rvdf(*.rvdf);;Environment(*.gltf *.glb);;Model(*.stl);;SceneGraph(*.iv);;All(*)",
                options=QtWidgets.QFileDialog.DontUseNativeDialog)
            logger.info(f"selected file is {fname}")

        if len(fname) == 0:
            logger.info("fname is empty!")
            return

        if str(fname).endswith(".gltf") or str(fname).endswith(".glb"):
            env = RCI.Environment()
            if not env.LoadFromFile(fname):
                return
            robot_models = env.GetAllRobotModels()
            if len(robot_models):
                robot_model = robot_models[0]
                manipulator = robot_model.GetActiveManipulator()
                self.__CreateController(manipulator)
            self.ivis.LoadEnvironment(env)
            # load programs
            ret, programs = env.GetExtraInfo('programs')
            if ret:
                programs = json.loads(programs)
                for prog_name, prog_content in programs.items():
                    prog = RPS.ScriptContainer.Create(prog_name)
                    prog.FromJson(prog_content)
                    self.program_panel.LoadProgFromScriptContainer(prog)

        elif str(fname).endswith(".rvdf"):
            robot_model = RCI.RobotModel()
            if not robot_model.InitFromRVDF(fname):
                return
            if self.app_data.get('controller') is not None:
                controller_old = self.app_data.get('controller')
                robot_model_old = controller_old.GetRobotModel()
                self.ivis.RemoveBody(robot_model_old)
            manipulator = robot_model.GetActiveManipulator()
            self.ivis.AddBody(robot_model)
            self.__CreateController(manipulator)
        elif str(fname).endswith(".stl"):
            self.ivis.AddModel(fname)
        elif len(fname) > 0:
            in_ = coin.SoInput()
            if not in_.openFile(fname):
                return None
            shape = coin.SoDB.readAll(in_)
            if shape:
                self.ivis.IViewer.root.addChild(shape)
        self.__UpdateSceneGraphDescription()

    def __Save(self, fname=""):
        if not fname:
            fname, _ = QtWidgets.QFileDialog.getSaveFileName(
                caption="Save file name", dir=str(DATADIR), filter="Environment(*.gltf *.glb);;SceneGraph(*.iv);;All(*)",
                options=QtWidgets.QFileDialog.DontUseNativeDialog)
        if str(fname).endswith(".iv"):
            output = coin.SoOutput()
            output.openFile(fname)
            writer = coin.SoWriteAction(output)
            writer.apply(self.viewer.root)
            output.closeFile()
            logger.info(f"save scene graph to {fname}")
        elif str(fname).endswith(".gltf") or str(fname).endswith(".glb"):
            env: RCI.Environment = self.ivis.env
            progs = self.app_data.get('programs')
            prog_contents: Dict[str, str] = {}
            for prog in progs:
                prog_name = prog.GetName()
                prog_content = prog.ToJson()
                prog_contents[prog_name] = prog_content
            programs = json.dumps(prog_contents)
            env.AddExtraInfo('programs', programs)
            env.SaveToFile(fname)
            logger.info(f"save environment to {fname}")
        else:
            fname = str(pathlib.Path(fname).with_suffix(".gltf"))
            self.__Save(fname)

    def __CreateController(self, manipulator: RCI.Manipulator) -> bool:
        controller_sim = RCI.SimController.Create(
            manipulator)
        self.app_data['controller'] = controller_sim
        self.app_data['controller_sim'] = controller_sim
        robot_model = manipulator.GetRobotModel()
        robot_name = robot_model.GetName()
        rbrand, rtype = RCI.FindRobotBrandAndType(robot_name)
        self.app_data['controller_real'] = None
        logger.info(f"Create robot controller: {robot_name}")
        if rbrand == "Motoman":
            try:
                from RVBUST.Motoman import PyMotoman
                controller_real = PyMotoman.MotomanControllerBase.Create(
                    manipulator)
                manipulator.SetArmController(controller_sim)
                self.app_data['controller_real'] = controller_real
                logger.info(
                    f"Create robot controller: {robot_name} successfully")
            except ImportError as e:
                logger.error(e)
        elif rbrand == "UniversalRobots":
            try:
                from RVBUST.UR import PyUR
                controller_real = PyUR.URControllerBase.Create(manipulator)
                manipulator.SetArmController(controller_sim)
                self.app_data['controller_real'] = controller_real
                logger.info(
                    f"Create robot controller: {robot_name} successfully")
            except ImportError as e:
                logger.error(e)
        else:
            logger.warning(f"Failed to create real controller {robot_name}")

        if self.robot_panel:
            self.view_menu.removeAction(self.robot_panel.toggleViewAction())
            self.robot_panel.close()
            self.robot_panel.destroy()
            self.robot_panel = None

        self.__InitRobotPanel()


if __name__ == "__main__":
    app = QtWidgets.QApplication([])
    main_window = MainWindow()
    main_window.show()
    app.processEvents()
    # main_window.__Open(DATADIR + "/UR10.rvdf")
    app.exec_()
