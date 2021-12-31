# !/usr/bin/python3
# -*- coding: utf-8 -*-
# Copyright (c) RVBUST, Inc - All rights reserved.

"""
 A simplified User Interface (UI) for jogging 
"""

import datetime
import os
from threading import Thread

import dearpygui.dearpygui as dpg
import dearpygui.themes as themes
import numpy as np
from IPython import embed
from RVBUST.RCI import *

try:
    from .PyRPS import *
    from .TrajectorySerialization import *
except ImportError:
    from RVBUST.RPS import *

logger = Logger.GetConsoleLogger("Jogger")

############## GLOBAL VARIABLES ########################

UR5RVDF = "{}/Rvbust/Data/Multibody/RobotModels/UniversalRobots/UR5/UR5.rvdf".format(
    os.getenv('HOME'))
MotoMiniRVDF = "{}/Rvbust/Data/Multibody/RobotModels/Motoman/MotoMini/MotoMini.rvdf".format(
    os.getenv('HOME'))
GP12RVDF = "{}/Rvbust/Data/Multibody/RobotModels/Motoman/GP12/GP12.rvdf".format(
    os.getenv('HOME'))
GP7RVDF = "{}/Rvbust/Data/Multibody/RobotModels/Motoman/GP7/GP7.rvdf".format(
    os.getenv('HOME'))
Irb1200RVDF = "{}/Rvbust/Data/Multibody/RobotModels/ABB/IRB1200_5_90/IRB1200_5_90.rvdf".format(
    os.getenv('HOME'))
##########################################################

bottom_status_id = -1


class ControllerJoggerUI:
    def __init__(self, robot_controller: GenericRobotController, limits_scale=0.1,
                 home=[0, 0, 0, 0, 0, 0], viewer=None):
        """Controller Jogger User Interface

        :param robot_controller: [description]
        :type robot_controller: GenericRobotController
        :param limits_scale: [description], defaults to 0.1
        :type limits_scale: float, optional
        :param home: [description], defaults to [0, 0, 0, 0, 0, 0]
        :type home: list, optional
        :param viewer: [description], defaults to None
        :type viewer: [type], optional
        """
        self.m_robot_controller = robot_controller
        self.m_jogger = None
        self.m_kin_solver = self.m_robot_controller.GetKinSolver()
        self.m_joint_limits = ConvertJointLimits2Arr(
            self.m_kin_solver.GetJointLimits())
        self.m_cartesian_limits = self.m_robot_controller.GetCartesianLimits()
        self.m_manip = self.m_robot_controller.GetManipulator()
        self.m_dof = self.m_manip.GetDoF()
        self.m_speed_ratio = self.m_robot_controller.GetSpeedRatio()
        self.m_cartesian_directions = np.eye(6)
        self.m_joint_directions = np.eye(self.m_dof)
        self.m_record_poses = []
        self.m_record_joints = []
        self.m_plot_handles = []
        self.m_vis = None
        self.m_display = viewer
        self.m_trajectory = None
        self.m_joint_home = Rx(home)
        self.m_limits_scale = limits_scale
        self.m_trajectory_jogger = None
        self.m_joint_labels = ["J%d" % i for i in range(1, self.m_dof+1)]
        self.m_pose_labels = ["X", "Y", "Z", "OX", "OY", "OZ", "OW"]
        self.m_cartesian_labels = ["X", "Y", "Z", "Rx", "Ry", "Rz"]
        self.m_path_type_names = ["Bezier2nd",
                                  "Bezier5th", "Circle", "Bezier5thCartesian"]
        self.m_path_types = [PathType_Bezier2ndBlend, PathType_Bezier5thBlend,
                             PathType_CircleBlend, PathType_Bezier5thBlendCartesian]
        self.m_traj_type_names = [
            "Trapezoidal", "DoubleS", "Totp3", "Toppra", "ConstantVelocity"]
        self.m_traj_types = [
            TrajType_Trapezoidal, TrajType_DoubleS, TrajType_Totp3, TrajType_Toppra, None]
        self.m_previous_status_movejl = False
        self.m_previous_status_jog_items = False
        self.m_pressed_show_clear_button = False
        self.m_jog_robot_switch_status = False
        self.m_robot_controller_ip_addr = "127.0.0.1"
        self.m_is_running = False
        self._init_widgets_id()

    def _init_widgets_id(self):
        self.m_joint_values_id = []
        self.m_poses_id = []
        self.m_widgets_id_dict = dict()

    @staticmethod
    def _update_status_info(msg: str, color: list = [0, 0, 0, -1]):
        # dpg.delete_item("Status Result")
        # dpg.add_text("Status Result", default_value=msg,
        #              color=color, parent="Status Info")
        global bottom_status_id
        dpg.configure_item(
            bottom_status_id, default_value=msg, color=color)

    @staticmethod
    def _exec_in_new_thread(func, *args, **kwargs):
        logger.Info("Executing {} in new thread.".format(func))
        Thread(target=func, args=args, kwargs=kwargs).start()

    def _connect_to_server_cb(self, sender, data):
        robot_states = self.m_robot_controller.GetControllerState()
        res = RVSReturn_Success
        if not robot_states.IsStateServerConnected():
            try:
                if self.m_robot_controller.Connect() != RVSReturn_Success:
                    res = self.m_robot_controller.Connect(
                        self.m_robot_controller_ip_addr)
            except TypeError:
                res = self.m_robot_controller.Connect(
                    self.m_robot_controller_ip_addr)
        if res != RVSReturn_Success:
            self._update_status_info(
                msg="Failed to connect to server. Maybe you can update \n"
                "ip address and reconnect.", color=[255, 0, 0, 255])
        else:
            dpg.configure_item(
                self.m_widgets_id_dict["Update IP"], enabled=False)
            dpg.configure_item(
                self.m_widgets_id_dict["Connect to Server"], enabled=False)
            dpg.configure_item(
                self.m_widgets_id_dict["Disconnect to Server"], enabled=True)
            dpg.configure_item(
                self.m_widgets_id_dict["Enable Robot"], enabled=True)
            dpg.configure_item(
                self.m_widgets_id_dict["EnableRobot##Group"], show=True)
            dpg.configure_item(
                self.m_widgets_id_dict["Set As Home Position"], enabled=True)
            dpg.configure_item(
                self.m_widgets_id_dict["Controller Speed Ratio"], enabled=True)
            dpg.lock_mutex()
            self._update_status_info(
                msg="Connected to Server", color=[0, 255, 0, 255])
            dpg.unlock_mutex()

    def _render_cb(self, sender, data):
        self._update_robot_state_table()
        self._joint_cart_space_jog()
        if self.m_trajectory is not None:
            dpg.configure_item(
                self.m_widgets_id_dict["Show/Clear Traj"], enabled=True)

    def _enable_move_jl_utils(self, enable=False):
        dpg.configure_item(
            self.m_widgets_id_dict["MoveJ##Joints"], enabled=enable)
        for i in range(0, self.m_dof):
            dpg.configure_item(self.m_widgets_id_dict["{}##Joints".format(
                self.m_joint_labels[i])], enabled=enable)
            if enable is True:
                dpg.configure_item(self.m_widgets_id_dict["{}##Joints".format(
                    self.m_joint_labels[i])], readonly=False)
        dpg.configure_item(
            self.m_widgets_id_dict["MoveL##Pose"], enabled=enable)
        for i in range(0, 7):
            dpg.configure_item(self.m_widgets_id_dict["{}##Pose".format(
                self.m_pose_labels[i])], enabled=enable)
            if enable is True:
                dpg.configure_item(self.m_widgets_id_dict["{}##Pose".format(
                    self.m_pose_labels[i])], readonly=False)
        dpg.configure_item(
            self.m_widgets_id_dict["Reset to current robot state"], enabled=enable)

    def _enable_robot_cb(self, sender, data):
        res = self.m_robot_controller.EnableRobot()
        if res != RVSReturn_Success:
            dpg.configure_item(
                self.m_widgets_id_dict["Back Home Position"], enabled=False)
            dpg.configure_item(
                self.m_widgets_id_dict["Enable Jog Robot"], enabled=False)
            dpg.configure_item(
                self.m_widgets_id_dict["Disable Jog Robot"], enabled=False)
            self._update_status_info(
                msg="Failed to enable robot", color=[255, 0, 0, 255])
        else:
            dpg.configure_item(
                self.m_widgets_id_dict["Enable Robot"], enabled=False)
            dpg.configure_item(
                self.m_widgets_id_dict["Disable Robot"], enabled=True)
            dpg.configure_item(
                self.m_widgets_id_dict["Back Home Position"], enabled=True)
            dpg.configure_item(
                self.m_widgets_id_dict["Enable Jog Robot"], enabled=True)
            dpg.configure_item(
                self.m_widgets_id_dict["Disable Jog Robot"], enabled=False)
            self._enable_move_jl_utils(enable=True)
            self._reset_to_current_robot_state(sender=None, data=None)
            dpg.configure_item(
                self.m_widgets_id_dict["Record Joints##Record"], enabled=True)
            dpg.configure_item(
                self.m_widgets_id_dict["Record Poses##Record"], enabled=True)
            dpg.configure_item(
                self.m_widgets_id_dict["JogRobotHeadGroup"], show=True)
            dpg.configure_item(
                self.m_widgets_id_dict["RecordRobotStatusGroup"], show=True)
            if len(self.m_record_joints) >= 2 or len(self.m_record_poses) >= 2:
                dpg.configure_item(
                    self.m_widgets_id_dict["Enable Jog Trajectory##Trajectory"], enabled=True)
            elif self.m_trajectory is not None:
                if self.m_trajectory_jogger is None:
                    self.m_trajectory_jogger = TrajectoryJogger(
                        self.m_robot_controller)
                self.m_trajectory_jogger.SetTrajectory(self.m_trajectory)
                dpg.configure_item(
                    self.m_widgets_id_dict["Enable Jog Trajectory##Trajectory"], enabled=True)
            self._update_status_info(
                msg="Enabled Robot", color=[0, 255, 0, 255])

    def _disconnect_to_server_cb(self, sender, data):
        self.m_robot_controller.DisableRobot()
        res = self.m_robot_controller.Disconnect()
        if res != RVSReturn_Success:
            self._update_status_info(
                msg="Failed to disconnect to server", color=[255, 255, 0, 255])
        else:
            dpg.configure_item(
                self.m_widgets_id_dict["Disconnect to Server"], enabled=False)
            dpg.configure_item(
                self.m_widgets_id_dict["Connect to Server"], enabled=True)
            dpg.configure_item(
                self.m_widgets_id_dict["Update IP"], enabled=True)
            dpg.configure_item(
                self.m_widgets_id_dict["Enable Robot"], enabled=False)
            dpg.configure_item(
                self.m_widgets_id_dict["Disable Robot"], enabled=False)
            dpg.configure_item(
                self.m_widgets_id_dict["EnableRobot##Group"], show=False)
            dpg.configure_item(
                self.m_widgets_id_dict["Back Home Position"], enabled=False)
            dpg.configure_item(
                self.m_widgets_id_dict["Set As Home Position"], enabled=False)
            dpg.configure_item(
                self.m_widgets_id_dict["Controller Speed Ratio"], enabled=False)
            dpg.configure_item(
                self.m_widgets_id_dict["Enable Jog Robot"], enabled=False)
            dpg.configure_item(
                self.m_widgets_id_dict["Disable Jog Robot"], enabled=False)
            self._enable_move_jl_utils(enable=False)
            dpg.configure_item(
                self.m_widgets_id_dict["Refer to Base Frame"], enabled=False)
            self._switch_jog_in_joint_cart_space(switch=False)
            self._show_move_jl_items(display=True)
            self._show_jog_items(display=False)
            dpg.configure_item(
                self.m_widgets_id_dict["Record Joints##Record"], enabled=False)
            dpg.configure_item(
                self.m_widgets_id_dict["Record Poses##Record"], enabled=False)
            dpg.configure_item(
                self.m_widgets_id_dict["Enable Jog Trajectory##Trajectory"], enabled=False)
            dpg.configure_item(
                self.m_widgets_id_dict["Disable Jog Trajectory##Trajectory"], enabled=False)
            dpg.configure_item(
                self.m_widgets_id_dict["JogTrajectoryGroup"], show=False)
            self._update_status_info(
                msg="Disconnected to server", color=[0, 255, 0, 255])

    def _disable_robot_cb(self, sender, data):
        res = self.m_robot_controller.DisableRobot()
        if res != RVSReturn_Success:
            self._update_status_info(
                msg="Failed to disable robot", color=[255, 255, 0, 255])
        else:
            dpg.configure_item(
                self.m_widgets_id_dict["Disable Robot"], enabled=False)
            dpg.configure_item(
                self.m_widgets_id_dict["Enable Robot"], enabled=True)
            dpg.configure_item(
                self.m_widgets_id_dict["Back Home Position"], enabled=False)
            dpg.configure_item(
                self.m_widgets_id_dict["Enable Jog Robot"], enabled=False)
            dpg.configure_item(
                self.m_widgets_id_dict["Disable Jog Robot"], enabled=False)
            self._enable_move_jl_utils(enable=False)
            dpg.configure_item(
                self.m_widgets_id_dict["Refer to Base Frame"], enabled=False)
            self._switch_jog_in_joint_cart_space(switch=False)
            self._show_move_jl_items(display=True)
            self._show_jog_items(display=False)
            dpg.configure_item(
                self.m_widgets_id_dict["Record Joints##Record"], enabled=False)
            dpg.configure_item(
                self.m_widgets_id_dict["Record Poses##Record"], enabled=False)
            dpg.configure_item(
                self.m_widgets_id_dict["Enable Jog Trajectory##Trajectory"], enabled=False)
            dpg.configure_item(
                self.m_widgets_id_dict["Disable Jog Trajectory##Trajectory"], enabled=False)
            dpg.configure_item(
                self.m_widgets_id_dict["JogTrajectoryGroup"], show=False)
            self._update_status_info(
                msg="Disabled Robot", color=[0, 255, 0, 255])

    def _get_robot_joints_values(self):
        robot_state = self.m_robot_controller.GetControllerState()
        curr_joint_values = None
        curr_eef_pose = None
        if robot_state.IsStateServerConnected() is True:
            _, joints = self.m_robot_controller.GetJointPosition()
            _, pose = self.m_robot_controller.GetPose()
            curr_joint_values = joints.Coeffs()
            curr_eef_pose = pose.Coeffs()
        return [curr_joint_values, curr_eef_pose]

    def _update_robot_state_table(self):
        if self.m_robot_controller.GetControllerState().IsStateServerConnected():
            # The update function will be put into a new thread, it
            # will synchronize the robot state in a sampling interval of 0.1s
            joint_values, eef_pose = self._get_robot_joints_values()
            # display current joint value of robot
            if joint_values is None:
                if len(self.m_joint_values_id) != 0:
                    for i in range(self.m_dof):
                        dpg.configure_item(
                            self.m_joint_values_id[i], default_value="")
            else:
                if len(self.m_joint_values_id) != 0:
                    for i in range(self.m_dof):
                        dpg.configure_item(self.m_joint_values_id[i],
                                           default_value="{:6.3f}".format(joint_values[i]))

            # display the pose of end-effector of robot
            if eef_pose is None:
                if len(self.m_poses_id) != 0:
                    for i in range(0, 7):
                        dpg.configure_item(
                            self.m_poses_id[i], default_value="")
            else:
                if len(self.m_poses_id) != 0:
                    for i in range(0, 7):
                        dpg.configure_item(
                            self.m_poses_id[i], default_value="{:6.4f}".format(eef_pose[i]))

    def _move_joints_cb(self, sender, data):
        joint_list = []
        for i in range(0, self.m_dof):
            value = dpg.get_value(
                self.m_widgets_id_dict["{}##Joints".format(self.m_joint_labels[i])])
            joint_list.append(value)
        robot_state = self.m_robot_controller.GetControllerState()
        if robot_state.IsMovable() is True:
            self.m_robot_controller.MoveJoints(Rx(joint_list), False)
            self._update_status_info(
                msg="Moved robot in joint space", color=[125, 125, 255, 255])
        else:
            self._update_status_info(
                msg="Failed to move robot. Maybe it is unconnected to"
                "server or fails to enable robot", color=[255, 255, 0, 255])

    def _move_linear_cb(self, sender, data):
        pose_list = []
        for i in range(0, 7):
            value = dpg.get_value(
                self.m_widgets_id_dict["{}##Pose".format(self.m_pose_labels[i])])
            pose_list.append(value)
        robot_state = self.m_robot_controller.GetControllerState()
        if robot_state.IsMovable() is True:
            self.m_robot_controller.MoveLinear(Pose(pose_list), False)
            self._update_status_info(
                msg="Moved robot in cartesian space", color=[125, 125, 255, 255])
        else:
            self._update_status_info(msg="Failed to move robot. Maybe it is unconnected to"
                                     "server or fails to enable robot",
                                     color=[255, 255, 0, 255])

    def _reset_to_current_robot_state(self, sender, data):
        joint_values, eef_pose = self._get_robot_joints_values()
        if joint_values is None or eef_pose is None:
            self._update_status_info(
                msg="Unconnected server. Failed to reset it", color=[255, 255, 0, 255])
        else:
            for i in range(0, self.m_dof):
                dpg.set_value(self.m_widgets_id_dict["{}##Joints".format(
                    self.m_joint_labels[i])], joint_values[i])
            for i in range(0, 7):
                dpg.set_value(self.m_widgets_id_dict["{}##Pose".format(
                    self.m_pose_labels[i])], eef_pose[i])
            self._update_status_info(
                msg="Reseted current robot state", color=[0, 255, 0, 255])

    def _back_home_position_cb(self, sender, data):
        robot_state = self.m_robot_controller.GetControllerState()
        if robot_state.IsControllerReady() is True:
            self.m_robot_controller.MoveJoints(self.m_joint_home, False)
            self._update_status_info(
                msg="Moved robot to home position", color=[125, 125, 255, 255])
        else:
            self._update_status_info(
                msg="Failed to move robot to home position. Maybe \n it's unconnected"
                " to server or disable robot", color=[255, 255, 0, 255])

    def _open_vis_cb(self, sender, data):
        if self.m_vis is None:
            self.m_vis = RobotVis(self.m_display)
            self.m_vis.AddBody(self.m_robot_controller.GetRobotModel())
        else:
            self.m_vis = RobotVis()
            self.m_vis.AddBody(self.m_robot_controller.GetRobotModel())
        dpg.configure_item(self.m_widgets_id_dict["Open Vis"], enabled=False)
        dpg.configure_item(self.m_widgets_id_dict["Close Vis"], enabled=True)
        self._update_status_info(
            msg="Opened display for robot", color=[125, 125, 255, 255])

    def _close_vis_cb(self, sender, data):
        if not self.m_vis.GetView().IsClosed():
            self.m_vis.GetView().Close()
        dpg.configure_item(self.m_widgets_id_dict["Close Vis"], enabled=False)
        dpg.configure_item(self.m_widgets_id_dict["Open Vis"], enabled=True)
        self._update_status_info(
            msg="Closed display for robot", color=[125, 125, 255, 255])

    def _close_controller_jogger_ui_cb(self, sender, data):
        dpg.delete_item(self.m_widgets_id_dict["ControllerJoggerUI"])
        self.m_is_running = False
        # self._update_status_info(
        #     msg="Closed ControllerJoggerUI window", color=[125, 125, 255, 255])

    def _set_joints_as_home_cb(self, sender, data):
        robot_state = self.m_robot_controller.GetControllerState()
        if robot_state.IsStateServerConnected() is True:
            _, joint_pos = self.m_robot_controller.GetJointPosition()
            self.m_joint_home = joint_pos
            self._update_status_info(
                msg="Updated joint home position", color=[0, 255, 0, 255])
        else:
            self._update_status_info(
                msg="Unconnected to server. Failed to update joint home position")
        dpg.configure_item(
            self.m_widgets_id_dict["Shortcut##Group"], show=True)
        dpg.configure_item(
            self.m_widgets_id_dict["SetHomePos##Group"], show=False)

    def _cancel_as_home_cb(self, sender, data):
        self._update_status_info(
            msg="Canceled to save current joint \n values as home joint position",
            color=[125, 125, 255, 255])
        dpg.configure_item(
            self.m_widgets_id_dict["Shortcut##Group"], show=True)
        dpg.configure_item(
            self.m_widgets_id_dict["SetHomePos##Group"], show=False)

    def _close_set_as_home_window_cb(self, sender, data):
        dpg.delete_item(self.m_widgets_id_dict["Set As Home Window"])
        self._update_status_info(
            msg="Closed set as home window without \n saving home joint position",
            color=[125, 125, 255, 255])

    def _set_as_home_cb(self, sender, data):
        dpg.configure_item(
            self.m_widgets_id_dict["Shortcut##Group"], show=False)
        dpg.configure_item(
            self.m_widgets_id_dict["SetHomePos##Group"], show=True)
        self._update_status_info(msg="Set new home position for robot",
                                 color=[125, 255, 255, 255])

    def _update_controller_speed_ratio_cb(self, sender, data):
        robot_state = self.m_robot_controller.GetControllerState()
        if robot_state.IsStateServerConnected() is True:
            value = dpg.get_value(
                self.m_widgets_id_dict["Controller Speed Ratio"])
            self.m_robot_controller.SetSpeedRatio(value)
            if self.m_jogger is not None:
                self.m_jogger.UpdateSpeedRatio(value)
            self._update_status_info(
                msg="Updated controller speed ratio", color=[0, 255, 0, 255])

    def _switch_jog_in_joint_cart_space(self, switch=False):
        for i in range(0, self.m_dof):
            dpg.configure_item(
                self.m_widgets_id_dict["{}+##Jogger".format(self.m_joint_labels[i])], enabled=switch)
            dpg.configure_item(
                self.m_widgets_id_dict["{}-##Jogger".format(self.m_joint_labels[i])], enabled=switch)
        for i in range(0, 6):
            dpg.configure_item(
                self.m_widgets_id_dict["{}+##Jogger".format(self.m_cartesian_labels[i])], enabled=switch)
            dpg.configure_item(
                self.m_widgets_id_dict["{}-##Jogger".format(self.m_cartesian_labels[i])], enabled=switch)

    def _init_jogger(self):
        self.m_jogger = ControllerJogger(
            self.m_robot_controller, self.m_limits_scale)
        res = self.m_jogger.StartJog()
        if res is False:
            self._update_status_info(
                msg="Failed to start to jog robot.", color=[255, 255, 0, 255])
            return False
        temp = dpg.get_value(self.m_widgets_id_dict["Controller Speed Ratio"])
        self.m_jogger.UpdateSpeedRatio(temp)
        return True

    def _enable_jog_robot_cb(self, sender, data):
        res = self._init_jogger()
        if res is True:
            self._enable_move_jl_utils(enable=False)
            dpg.configure_item(
                self.m_widgets_id_dict["Refer to Base Frame"], enabled=True)
            self._switch_jog_in_joint_cart_space(switch=True)
            dpg.configure_item(
                self.m_widgets_id_dict["Enable Jog Robot"], enabled=False)
            dpg.configure_item(
                self.m_widgets_id_dict["Disable Jog Robot"], enabled=True)
            dpg.configure_item(
                self.m_widgets_id_dict["Back Home Position"], enabled=False)
            self._show_jog_items(display=True)
            self._show_move_jl_items(display=False)
            self._update_status_info(
                msg="Enable jog robot in joint/cartesian space", color=[0, 255, 0, 255])

    def _disable_jog_robot_cb(self, sender, data):
        self.m_jogger = None
        dpg.configure_item(
            self.m_widgets_id_dict["Refer to Base Frame"], enabled=False)
        self._switch_jog_in_joint_cart_space(switch=False)
        dpg.configure_item(
            self.m_widgets_id_dict["Disable Jog Robot"], enabled=False)
        dpg.configure_item(
            self.m_widgets_id_dict["Enable Jog Robot"], enabled=True)
        self._enable_move_jl_utils(enable=True)
        self._reset_to_current_robot_state(sender=None, data=None)
        dpg.configure_item(
            self.m_widgets_id_dict["Back Home Position"], enabled=True)
        self._show_jog_items(display=False)
        self._show_move_jl_items(display=True)
        self._update_status_info(
            msg="Disable jog robot in joint/cartesian space", color=[0, 255, 0, 255])

    def _jog_robot_block(self):
        any_pressed = False
        jog_location = []
        for i in range(0, self.m_dof):
            is_actived_plus = dpg.is_item_active(
                self.m_widgets_id_dict["{}+##Jogger".format(self.m_joint_labels[i])])
            is_actived_minus = dpg.is_item_active(
                self.m_widgets_id_dict["{}-##Jogger".format(self.m_joint_labels[i])])
            if is_actived_plus is True:
                any_pressed = True
                self.m_jogger.UpdateDirection(
                    RxTangent(self.m_joint_directions[i]))
            if is_actived_minus is True:
                any_pressed = True
                self.m_jogger.UpdateDirection(
                    RxTangent(-self.m_joint_directions[i]))
            if is_actived_minus or is_actived_plus:
                jog_location = ["Joint", i]
        is_refer_to_base_frame = dpg.get_value(
            self.m_widgets_id_dict["Refer to Base Frame"])
        for i in range(0, 6):
            is_actived_plus = dpg.is_item_active(
                self.m_widgets_id_dict["{}+##Jogger".format(self.m_cartesian_labels[i])])
            is_actived_minus = dpg.is_item_active(
                self.m_widgets_id_dict["{}-##Jogger".format(self.m_cartesian_labels[i])])
            if is_actived_plus is True:
                any_pressed = True
                self.m_jogger.UpdateDirection(SE3Tangent(
                    self.m_cartesian_directions[i]), is_refer_to_base_frame)
            if is_actived_minus is True:
                any_pressed = True
                self.m_jogger.UpdateDirection(SE3Tangent(
                    -self.m_cartesian_directions[i]), is_refer_to_base_frame)
            if is_actived_minus or is_actived_plus:
                jog_location = ["Cartesian", i]
        if not any_pressed:
            self.m_jogger.StopMove()
        if len(jog_location) != 0:
            loc, idx = jog_location
            if loc == "Joint":
                self._update_status_info(msg="Jog robot joint {} in joint space".format(
                    idx+1), color=[125, 125, 255, 255])
            elif loc == "Cartesian":
                self._update_status_info(
                    msg="Jog robot endeffector along {} axis in cartesian space".format(
                        self.m_cartesian_labels[idx]),
                    color=[125, 125, 255, 255])

    def _switch_jag_trajetory_button(self, ratio):
        if abs(ratio-1) < 1e-6:
            dpg.configure_item(
                self.m_widgets_id_dict["Jog Traj Forward"], enabled=False)
        elif ratio < 1e-6:
            dpg.configure_item(
                self.m_widgets_id_dict["Jog Traj Back"], enabled=False)
        else:
            dpg.configure_item(
                self.m_widgets_id_dict["Jog Traj Forward"], enabled=True)
            dpg.configure_item(
                self.m_widgets_id_dict["Jog Traj Back"], enabled=True)

    def _jog_trajectory_block(self):
        jog_traj_button_pressed = False
        is_actived_plus = dpg.is_item_active(
            self.m_widgets_id_dict["Jog Traj Forward"])
        is_actived_minus = dpg.is_item_active(
            self.m_widgets_id_dict["Jog Traj Back"])
        if is_actived_plus is True:
            jog_traj_button_pressed = True
            self.m_trajectory_jogger.JogTrajectory(True)
        if is_actived_minus is True:
            jog_traj_button_pressed = True
            self.m_trajectory_jogger.JogTrajectory(False)
        if jog_traj_button_pressed is False:
            self.m_trajectory_jogger.StopJog()
        if is_actived_plus is True or is_actived_minus is True:
            curr_ratio = self.m_trajectory_jogger.GetJogPercentage()
            dpg.set_value(
                self.m_widgets_id_dict["Jog Trajectory Progress Bar"], curr_ratio)
            dpg.configure_item(self.m_widgets_id_dict["Jog Trajectory Progress Bar"],
                               overlay="%.2f" % curr_ratio)
            self._switch_jag_trajetory_button(ratio=curr_ratio)
        if is_actived_plus or is_actived_minus:
            self._update_status_info(msg="Jog trajectory in cartesian space",
                                     color=[0, 255, 0, 255])

    def _joint_cart_space_jog(self):
        if self.m_jogger is not None:
            self._jog_robot_block()
        if self.m_trajectory_jogger is not None:
            self._jog_trajectory_block()

    def _show_move_jl_items(self, display=False):
        dpg.configure_item(self.m_widgets_id_dict["MoveJLGroup"], show=display)

    def _show_jog_items(self, display=False):
        for i in range(0, self.m_dof):
            dpg.configure_item(
                self.m_widgets_id_dict["{}+##Jogger".format(self.m_joint_labels[i])], show=display)
            dpg.configure_item(
                self.m_widgets_id_dict["{}-##Jogger".format(self.m_joint_labels[i])], show=display)
        dpg.configure_item(
            self.m_widgets_id_dict["Refer to Base Frame"], show=display)
        for i in range(0, 6):
            dpg.configure_item(
                self.m_widgets_id_dict["{}+##Jogger".format(self.m_cartesian_labels[i])], show=display)
            dpg.configure_item(
                self.m_widgets_id_dict["{}-##Jogger".format(self.m_cartesian_labels[i])], show=display)

    def _update_record_clear_save_status_with_joint(self):
        dpg.configure_item(self.m_widgets_id_dict["Record Joints##Record"], label="Record Joints ({})".format(
            len(self.m_record_joints)))
        dpg.configure_item(
            self.m_widgets_id_dict["Clear Joints##Record"], enabled=True)
        dpg.configure_item(
            self.m_widgets_id_dict["Save Joints##Record"], enabled=True)
        if len(self.m_record_joints) >= 2:
            dpg.configure_item(
                self.m_widgets_id_dict["Enable Jog Trajectory##Trajectory"], enabled=True)
            dpg.configure_item(
                self.m_widgets_id_dict["Generate Joint Traj"], enabled=True)

    def _record_robot_joints_cb(self, sender, data):
        curr_joint_pos = self.m_robot_controller.GetJointPosition()[1]
        is_approx = False
        for temp_j in self.m_record_joints:
            if temp_j.IsApprox(curr_joint_pos) is True:
                is_approx = True
                break
        if not is_approx:
            self.m_record_joints.append(curr_joint_pos)
        self._update_record_clear_save_status_with_joint()
        self._update_status_info(
            msg="Reocrd robot joint status", color=[125, 125, 255, 255])

    def _clear_robot_joints_cb(self, sender, data):
        self.m_record_joints.clear()
        dpg.configure_item(
            self.m_widgets_id_dict["Record Joints##Record"], label="Record Joints")
        dpg.configure_item(
            self.m_widgets_id_dict["Clear Joints##Record"], enabled=False)
        dpg.configure_item(
            self.m_widgets_id_dict["Save Joints##Record"], enabled=False)
        dpg.configure_item(
            self.m_widgets_id_dict["Generate Joint Traj"], enabled=False)
        if len(self.m_record_poses) < 2:
            dpg.configure_item(
                self.m_widgets_id_dict["Enable Jog Trajectory##Trajectory"], enabled=False)
        self._update_status_info(
            msg="Cleared all recorded joints", color=[0, 255, 0, 255])

    def _save_robot_joints_cb(self, sender, data):
        if len(self.m_record_joints) > 0:
            file_name = "./robot_joints_{}.txt".format(
                datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S"))
            np.savetxt(file_name, [j.Coeffs()
                                   for j in self.m_record_joints], fmt="%.4f")
            dpg.configure_item(
                self.m_widgets_id_dict["Save Joints##Record"], enabled=False)
            self._update_status_info(
                msg="Saved recorded joints to \n {}".format(file_name), color=[0, 255, 0, 255])

    def _update_record_clear_save_status_with_pose(self):
        dpg.configure_item(self.m_widgets_id_dict["Record Poses##Record"], label="Record Poses ({})".format(
            len(self.m_record_poses)))
        dpg.configure_item(
            self.m_widgets_id_dict["Clear Poses##Record"], enabled=True)
        dpg.configure_item(
            self.m_widgets_id_dict["Save Poses##Record"], enabled=True)
        if len(self.m_record_poses) >= 2:
            dpg.configure_item(
                self.m_widgets_id_dict["Enable Jog Trajectory##Trajectory"], enabled=True)
            dpg.configure_item(
                self.m_widgets_id_dict["Generate Cartesian Traj"], enabled=True)

    def _record_robot_poses_cb(self, sender, data):
        curr_eef_pose = self.m_robot_controller.GetPose()[1]
        is_approx = False
        for temp_p in self.m_record_poses:
            if temp_p.IsApprox(curr_eef_pose) is True:
                is_approx = True
                break
        if not is_approx:
            self.m_record_poses.append(curr_eef_pose)
        self._update_record_clear_save_status_with_pose()
        self._update_status_info(
            msg="Record robot endeffector pose", color=[125, 125, 255, 255])

    def _clear_robot_poses_cb(self, sender, data):
        self.m_record_poses.clear()
        dpg.configure_item(
            self.m_widgets_id_dict["Record Poses##Record"], label="Record Poses")
        dpg.configure_item(
            self.m_widgets_id_dict["Clear Poses##Record"], enabled=False)
        dpg.configure_item(
            self.m_widgets_id_dict["Save Poses##Record"], enabled=False)
        dpg.configure_item(
            self.m_widgets_id_dict["Generate Cartesian Traj"], enabled=False)
        if len(self.m_record_joints) < 2:
            dpg.configure_item(
                self.m_widgets_id_dict["Enable Jog Trajectory##Trajectory"], enabled=False)
        self._update_status_info(
            msg="Cleared all recorded robot poses", color=[0, 255, 0, 255])

    def _save_robot_poses_cb(self, sender, data):
        if len(self.m_record_poses) > 0:
            file_name = "./robot_poses_{}.txt".format(
                datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S"))
            np.savetxt(file_name, [p.Coeffs()
                                   for p in self.m_record_poses], fmt="%.6f")
            dpg.configure_item(
                self.m_widgets_id_dict["Save Poses##Record"], enabled=False)
            self._update_status_info(
                msg="Saved recorded poses to \n {}".format(file_name), color=[0, 255, 0, 255])

    def _enable_jog_trajectory_cb(self, sender, data):
        movejl_config = dpg.get_item_configuration(
            self.m_widgets_id_dict["MoveJLGroup"])
        self.m_previous_status_movejl = movejl_config["show"]
        jog_config = dpg.get_item_configuration(
            self.m_widgets_id_dict["Refer to Base Frame"])
        self.m_previous_status_jog_items = jog_config["show"]
        dpg.configure_item(
            self.m_widgets_id_dict["JogRobotHeadGroup"], show=False)
        self._disable_jog_robot_cb(sender=None, data=None)
        self._show_jog_items(display=False)
        self._show_move_jl_items(display=False)
        dpg.configure_item(
            self.m_widgets_id_dict["RecordRobotStatusGroup"], show=False)
        dpg.configure_item(
            self.m_widgets_id_dict["JogTrajectoryGroup"], show=True)
        dpg.configure_item(
            self.m_widgets_id_dict["Enable Jog Trajectory##Trajectory"], enabled=False)
        dpg.configure_item(
            self.m_widgets_id_dict["Disable Jog Trajectory##Trajectory"], enabled=True)
        self._update_status_info(
            msg="Enable jog trajectory", color=[0, 255, 0, 255])

    def _disable_jog_trajectory_cb(self, sender, data):
        # self.m_trajectory = None
        self.m_pressed_show_clear_button = True
        self._show_clear_cart_trajectory_cb(sender=None, data=None)
        dpg.configure_item(
            self.m_widgets_id_dict["JogTrajectoryGroup"], show=False)
        dpg.configure_item(
            self.m_widgets_id_dict["RecordRobotStatusGroup"], show=True)
        dpg.configure_item(
            self.m_widgets_id_dict["JogRobotHeadGroup"], show=True)
        if self.m_previous_status_jog_items is True:
            self._enable_jog_robot_cb(sender=None, data=None)
        self._show_jog_items(display=self.m_previous_status_jog_items)
        self._show_move_jl_items(display=self.m_previous_status_movejl)
        self._reset_to_current_robot_state(sender=None, data=None)
        dpg.configure_item(
            self.m_widgets_id_dict["Enable Jog Trajectory##Trajectory"], enabled=True)
        dpg.configure_item(
            self.m_widgets_id_dict["Disable Jog Trajectory##Trajectory"], enabled=False)
        self._update_status_info(
            msg="Disable jog trajectory", color=[0, 255, 0, 255])

    def _path_type_combo_cb(self, sender, data):
        self._update_status_info(
            msg="Update path type", color=[125, 125, 255, 255])

    def _traj_type_combo_cb(self, sender, data):
        curr_value = dpg.get_value(
            self.m_widgets_id_dict["TrajType##Trajectory"])
        if curr_value == self.m_traj_type_names[-1]:
            dpg.configure_item(self.m_widgets_id_dict["Velocity##Trajectory"],
                               enabled=True, readonly=False)
        else:
            dpg.configure_item(self.m_widgets_id_dict["Velocity##Trajectory"],
                               enabled=False, readonly=True)
        self._update_status_info(
            msg="Update trajectory type", color=[125, 125, 255, 255])

    def _get_traj_info(self):
        curr_blend_tolerence = dpg.get_value(
            self.m_widgets_id_dict["BlendTolerance##Trajectory"])
        curr_path_type = dpg.get_value(
            self.m_widgets_id_dict["PathType##Trajectory"])
        curr_traj_type = dpg.get_value(
            self.m_widgets_id_dict["TrajType##Trajectory"])
        idx_p = self.m_path_type_names.index(curr_path_type)
        idx_t = self.m_traj_type_names.index(curr_traj_type)
        path_type = self.m_path_types[idx_p]
        traj_type = self.m_traj_types[idx_t]
        return [curr_blend_tolerence, path_type, traj_type]

    def _generate_joint_trajectory_cb(self, sender, data):
        curr_blend_tolerence, path_type, traj_type = self._get_traj_info()
        temp_path = CreatePath(self.m_record_joints,
                               curr_blend_tolerence, path_type)
        if traj_type is not None:
            self.m_trajectory = CreateTrajectory(
                temp_path, *self.m_joint_limits, traj_type)
        else:
            curr_traj_const_vel = dpg.get_value(
                self.m_widgets_id_dict["Velocity##Trajectory"])
            self.m_trajectory = CreateTrajectory(
                temp_path, curr_traj_const_vel)
        if self.m_trajectory.IsValid() is True:
            if self.m_trajectory_jogger is None:
                self.m_trajectory_jogger = TrajectoryJogger(
                    self.m_robot_controller)
            self.m_trajectory_jogger.SetTrajectory(self.m_trajectory)
            self.m_pressed_show_clear_button = True
            self._show_clear_cart_trajectory_cb(sender=None, data=None)
            dpg.configure_item(
                self.m_widgets_id_dict["Show/Clear Traj"], enabled=True)
            dpg.configure_item(
                self.m_widgets_id_dict["Jog Traj Forward"], enabled=True)
            dpg.configure_item(
                self.m_widgets_id_dict["Jog Traj Back"], enabled=True)
            self._update_status_info(
                msg="Generated trajectory in joint space", color=[0, 255, 0, 255])
        else:
            self._update_status_info(
                msg="Failed to yield valid trajectory \n in joint space", color=[255, 0, 0, 255])

    def _generate_cartesian_trajectory_cb(self, sender, data):
        curr_blend_tolerence, path_type, traj_type = self._get_traj_info()
        path_coeffs = [p.Coeffs() for p in self.m_record_poses]
        temp_path = CreatePath(path_coeffs, curr_blend_tolerence, path_type)
        if traj_type is not None:
            self.m_trajectory = CreateTrajectory(
                temp_path, *self.m_cartesian_limits, traj_type)
        else:
            curr_traj_const_vel = dpg.get_value(
                self.m_widgets_id_dict["Velocity##Trajectory"])
            self.m_trajectory = CreateTrajectory(
                temp_path, curr_traj_const_vel)
        if self.m_trajectory.IsValid() is True:
            if self.m_trajectory_jogger is None:
                self.m_trajectory_jogger = TrajectoryJogger(
                    self.m_robot_controller)
            self.m_trajectory_jogger.SetTrajectory(self.m_trajectory)
            self.m_pressed_show_clear_button = True
            self._show_clear_cart_trajectory_cb(sender=None, data=None)
            dpg.configure_item(
                self.m_widgets_id_dict["Show/Clear Traj"], enabled=True)
            dpg.configure_item(
                self.m_widgets_id_dict["Jog Traj Forward"], enabled=True)
            dpg.configure_item(
                self.m_widgets_id_dict["Jog Traj Back"], enabled=True)
            self._update_status_info(
                msg="Generated trajectory in Cartesian space", color=[0, 255, 0, 255])
        else:
            self._update_status_info(
                msg="Failed to yield valid trajectory \n in Cartesian space", color=[255, 0, 0, 255])

    def _show_clear_cart_trajectory_cb(self, sender, data):
        self.m_pressed_show_clear_button = not self.m_pressed_show_clear_button
        if self.m_pressed_show_clear_button is True:
            if self.m_vis is None or self.m_vis.GetView().IsClosed() is True:
                self._open_vis_cb(sender=None, data=None)
            if self.m_trajectory is not None:
                self.m_plot_handles.extend(DrawTraj(self.m_trajectory, view=self.m_vis.GetView(),
                                                    kin_solver=self.m_robot_controller.GetKinSolver(),
                                                    show_in_vis=True, show_in_plt=False))
                dpg.configure_item(
                    self.m_widgets_id_dict["Jog Traj Forward"], enabled=True)
                dpg.configure_item(
                    self.m_widgets_id_dict["Jog Traj Back"], enabled=True)
        else:
            if self.m_vis is not None:
                self.m_vis.GetView().Delete(self.m_plot_handles)
            self.m_plot_handles.clear()
            # self.m_trajectory = None
            dpg.configure_item(
                self.m_widgets_id_dict["Jog Traj Forward"], enabled=False)
            dpg.configure_item(
                self.m_widgets_id_dict["Jog Traj Back"], enabled=False)
            # dpg.configure_item(self.m_widgets_id_dict["Show/Clear Traj"], enabled=False)
        info = None
        if self.m_pressed_show_clear_button is True:
            info = "Show"
        else:
            info = "Clear"
        self._update_status_info(
            msg="{} trajectory in cartesian space.".format(info),
            color=[0, 255, 0, 255])

    def _initialize_controller_jogger(self):
        if self.m_vis is not None:
            if not self.m_vis.GetView().IsClosed():
                dpg.configure_item(
                    self.m_widgets_id_dict["Open Vis"], enabled=False)
                dpg.configure_item(
                    self.m_widgets_id_dict["Close Vis"], enabled=True)
        robot_states = self.m_robot_controller.GetControllerState()
        if robot_states.IsStateServerConnected():
            self._connect_to_server_cb(sender=None, data=None)
        if len(self.m_record_joints) != 0:
            self._update_record_clear_save_status_with_joint()
        if len(self.m_record_poses) != 0:
            self._update_record_clear_save_status_with_pose()
        dpg.configure_item(
            self.m_widgets_id_dict["Enable Jog Trajectory##Trajectory"], enabled=False)

    def _update_ip_addr_cb(self, sender, data):
        dpg.configure_item(
            self.m_widgets_id_dict["ConnectToServer##Group"], show=False)
        dpg.configure_item(self.m_widgets_id_dict["SetIP##Group"], show=True)
        self._update_status_info(msg="Set new ip address for connecting to controller",
                                 color=[125, 255, 255, 255])

    def _confirm_new_ip_addr_cb(self, sender, data):
        self.m_robot_controller_ip_addr = dpg.get_value(
            self.m_widgets_id_dict["Set new ip address"])
        dpg.configure_item(
            self.m_widgets_id_dict["ConnectToServer##Group"], show=True)
        dpg.configure_item(self.m_widgets_id_dict["SetIP##Group"], show=False)
        self._update_status_info(msg="Set new ip address \"{}\".".format(self.m_robot_controller_ip_addr),
                                 color=[0, 255, 0, 255])

    def _cancel_new_ip_addr_cb(self, sender, data):
        dpg.configure_item(
            self.m_widgets_id_dict["ConnectToServer##Group"], show=True)
        dpg.configure_item(self.m_widgets_id_dict["SetIP##Group"], show=False)
        self._update_status_info(
            msg="Cancel to update ip address.", color=[125, 125, 255, 255])

    def _post_process_jogger_tools(self):
        if self.m_jogger is not None:

            self.m_jogger.StopJog()
        if self.m_trajectory_jogger is not None:
            self.m_trajectory_jogger.StopJog()
        logger.SetLevelForAll(2)
        logger.Info("Closed to jog robot and jog trajectory.")

    def Run(self):
        self._init_widgets_id()
        dpg.setup_viewport()
        dpg.set_viewport_title("Main Window")
        dpg.set_viewport_height(820)
        dpg.set_viewport_width(560)
        h = dpg.get_viewport_client_height()
        w = dpg.get_viewport_client_width()
        # dpg.setup_dearpygui()
        with dpg.window(label="ControllerJoggerUI", pos=[0, 0], width=w, height=h,
                        on_close=self._close_controller_jogger_ui_cb, no_title_bar=True,
                        no_move=True, no_resize=True, no_scrollbar=True) as main_window:
            dpg.add_text("Controller Configuration")
            dpg.add_same_line(spacing=306)
            with dpg.theme(default_theme=False) as temp_th:
                dpg.add_theme_color(dpg.mvThemeCol_Button,
                                    value=[255, 0, 0, 255])
                dpg.add_theme_color(dpg.mvThemeCol_Text, value=[0, 0, 0, 255])
            temp_id = dpg.add_button(label="Quit", enabled=True, width=60,
                                     callback=self._close_controller_jogger_ui_cb)
            dpg.set_item_theme(temp_id, temp_th)
            dpg.add_text(f"Name: {self.m_robot_controller.GetName()}", color=[
                125, 125, 255, 255])
            dpg.add_spacing(count=1)
            with dpg.group(label="ConnectToServer##Group") as temp_ctsg:
                # dpg.add_group(label="ConnectToServer##Group")
                temp_id = dpg.add_button(filter_key="Button", label="Connect to Server",
                                         callback=self._connect_to_server_cb, enabled=True)
                self.m_widgets_id_dict["Connect to Server"] = temp_id
                dpg.add_same_line(spacing=10)
                temp_id = dpg.add_button(filter_key="Button", label="Disconnect to Server", enabled=False,
                                         callback=self._disconnect_to_server_cb)
                self.m_widgets_id_dict["Disconnect to Server"] = temp_id
                dpg.add_same_line(spacing=177)
                temp_id = dpg.add_button(filter_key="Button", label="Update IP", enabled=True,
                                         callback=self._update_ip_addr_cb)
                self.m_widgets_id_dict["Update IP"] = temp_id
                # dpg.end()
            self.m_widgets_id_dict["ConnectToServer##Group"] = temp_ctsg
            with dpg.group(label="SetIP##Group", show=False) as temp_sipg:
                # dpg.add_group(label="SetIP##Group", show=False)
                temp_id = dpg.add_input_text(label="Set new ip address",
                                             default_value="127.0.0.1")
                self.m_widgets_id_dict["Set new ip address"] = temp_id
                dpg.add_spacing(count=1)
                temp_id = dpg.add_button(filter_key="Button",
                                         label="Confirm##IP", callback=self._confirm_new_ip_addr_cb, width=60)
                self.m_widgets_id_dict["Confirm##IP"] = temp_id
                dpg.add_same_line(spacing=10)
                temp_id = dpg.add_button(filter_key="Button",
                                         label="Cancel##IP", callback=self._cancel_new_ip_addr_cb, width=60)
                self.m_widgets_id_dict["Cancel##IP"] = temp_id
                dpg.add_spacing(count=1)
                # dpg.end()
            self.m_widgets_id_dict["SetIP##Group"] = temp_sipg
            dpg.add_spacing(count=1)
            with dpg.group(label="EnableRobot##Group", show=False) as temp_erg:
                # dpg.add_group(label="EnableRobot##Group", show=False)
                temp_id = dpg.add_button(filter_key="Button",
                                         label="Enable Robot", callback=self._enable_robot_cb, enabled=False)
                self.m_widgets_id_dict["Enable Robot"] = temp_id
                dpg.add_same_line(spacing=10)
                temp_id = dpg.add_button(filter_key="Button", label="Disable Robot", enabled=False,
                                         callback=self._disable_robot_cb)
                self.m_widgets_id_dict["Disable Robot"] = temp_id
                dpg.add_spacing(count=2)
                # dpg.end()
            self.m_widgets_id_dict["EnableRobot##Group"] = temp_erg
            dpg.add_separator()
            dpg.add_text("Robot State Info")
            # dpg.add_table(label="Table##JointValues",
            #               headers=temp_header, height=60)
            with dpg.table(label="Table##JointValues", header_row=True, policy=dpg.mvTable_SizingStretchSame,
                           borders_innerH=False, borders_innerV=True, borders_outerH=True, borders_outerV=True, row_background=True):
                dpg.add_table_column()
                for i in range(len(self.m_joint_labels)):
                    dpg.add_table_column(label=self.m_joint_labels[i])
                temp_row = ["Joints"] + ["" for i in range(self.m_dof)]
                for i in range(len(temp_row)):
                    temp_id = dpg.add_text(temp_row[i])
                    if i != 0:
                        self.m_joint_values_id.append(temp_id)
                    if i != len(temp_row)-1:
                        dpg.add_table_next_column()
            with dpg.table(label="Table##EEFPose", header_row=True, policy=dpg.mvTable_SizingStretchSame,
                           borders_innerH=True, borders_innerV=True, borders_outerH=True, borders_outerV=True,
                           row_background=True):
                dpg.add_table_column()
                for i in range(len(self.m_pose_labels)):
                    dpg.add_table_column(label=self.m_pose_labels[i])
                temp_row = ["Pose"] + ["" for i in range(7)]
                for i in range(len(temp_row)):
                    temp_id = dpg.add_text(temp_row[i])
                    if i != 0:
                        self.m_poses_id.append(temp_id)
                    if i != len(temp_row) - 1:
                        dpg.add_table_next_column()
            dpg.add_spacing(count=2)
            with dpg.group(label="Shortcut##Group") as temp_scg:
                # dpg.add_group(label="Shortcut##Group")
                dpg.add_separator()
                dpg.add_text("Shortcut")
                dpg.add_spacing(count=1)
                temp_id = dpg.add_slider_float(label="Controller Speed Ratio", default_value=0.5,
                                               min_value=0.0, max_value=1.0, format="%.2f",
                                               clamped=True, enabled=False, callback=self._update_controller_speed_ratio_cb)
                self.m_widgets_id_dict["Controller Speed Ratio"] = temp_id
                dpg.add_spacing(count=1)
                temp_id = dpg.add_button(filter_key="Button", label="Back Home Position", enabled=False,
                                         callback=self._back_home_position_cb)
                self.m_widgets_id_dict["Back Home Position"] = temp_id
                dpg.add_same_line(spacing=10)
                temp_id = dpg.add_button(filter_key="Button", label="Set As Home Position", enabled=False,
                                         callback=self._set_as_home_cb)
                self.m_widgets_id_dict["Set As Home Position"] = temp_id
                dpg.add_same_line(spacing=95)
                temp_id = dpg.add_button(filter_key="Button", label="Open Vis", enabled=True,
                                         callback=self._open_vis_cb)
                self.m_widgets_id_dict["Open Vis"] = temp_id
                dpg.add_same_line(spacing=10)
                temp_id = dpg.add_button(filter_key="Button", label="Close Vis", enabled=False,
                                         callback=self._close_vis_cb)
                self.m_widgets_id_dict["Close Vis"] = temp_id
                # dpg.end()
            self.m_widgets_id_dict["Shortcut##Group"] = temp_scg
            with dpg.group(label="SetHomePos##Group", show=False) as temp_shpg:
                # dpg.add_group(label="SetHomePos##Group", show=False)
                dpg.add_separator()
                dpg.add_text(
                    "Are you sure to set the current joint  \n values as home position value?")
                dpg.add_spacing(count=5)
                dpg.add_same_line(spacing=40)
                dpg.add_button(filter_key="Button",
                               label="Yes##HomePos", callback=self._set_joints_as_home_cb, width=50)
                dpg.add_same_line(spacing=70)
                dpg.add_button(filter_key="Button",
                               label="No##HomePos", callback=self._cancel_as_home_cb, width=50)
                # dpg.end()
            self.m_widgets_id_dict["SetHomePos##Group"] = temp_shpg
            dpg.add_spacing(count=2)
            with dpg.group(label="MoveJLGroup") as temp_mjlg:
                # dpg.add_group(label="MoveJLGroup")
                dpg.add_separator()
                dpg.add_text("MoveJ/L")
                for i in range(0, self.m_dof):
                    temp_id = dpg.add_input_float(label="{}##Joints".format(
                        self.m_joint_labels[i]), default_value=0.0, width=105, format="%.3f",
                        enabled=False, readonly=False, min_value=-2*np.pi, max_value=2*np.pi)
                    self.m_widgets_id_dict["{}##Joints".format(
                        self.m_joint_labels[i])] = temp_id
                    cond1 = i == 0
                    cond2 = (i+1) % 3 != 0 and i != self.m_dof-1
                    if cond1 or cond2:
                        dpg.add_same_line(spacing=20)
                dpg.add_spacing(count=1)
                temp_id = dpg.add_button(filter_key="Button", label="MoveJ##Joints", enabled=False,
                                         callback=self._move_joints_cb)
                self.m_widgets_id_dict["MoveJ##Joints"] = temp_id
                dpg.add_spacing(count=2)
                init_value = Pose()
                for i in range(0, 7):
                    temp_id = dpg.add_input_float(label="{}##Pose".format(
                        self.m_pose_labels[i]), default_value=init_value[i], width=105, format="%.4f",
                        enabled=False, readonly=False, min_value=-10.0, max_value=10.0)
                    self.m_widgets_id_dict["{}##Pose".format(
                        self.m_pose_labels[i])] = temp_id
                    if i != 2 and i != 6:
                        dpg.add_same_line(spacing=14)
                dpg.add_spacing(count=1)
                temp_id = dpg.add_button(filter_key="Button", label="MoveL##Pose", enabled=False,
                                         callback=self._move_linear_cb)
                self.m_widgets_id_dict["MoveL##Pose"] = temp_id
                dpg.add_same_line(spacing=280)
                temp_id = dpg.add_button(filter_key="Button", label="Reset to current robot state", enabled=False,
                                         callback=self._reset_to_current_robot_state)
                self.m_widgets_id_dict["Reset to current robot state"] = temp_id
                dpg.add_spacing(count=1)
                # dpg.end()
            self.m_widgets_id_dict["MoveJLGroup"] = temp_mjlg
            with dpg.group(label="JogRobotHeadGroup") as temp_jrhg:
                # dpg.add_group(label="JogRobotHeadGroup")
                dpg.add_separator()
                dpg.add_text("Jog Robot")
                dpg.add_spacing(count=1)
                temp_id = dpg.add_button(filter_key="Button", label="Enable Jog Robot", enabled=False,
                                         callback=self._enable_jog_robot_cb)
                self.m_widgets_id_dict["Enable Jog Robot"] = temp_id
                dpg.add_same_line(spacing=20)
                temp_id = dpg.add_button(filter_key="Button", label="Disable Jog Robot", enabled=False,
                                         callback=self._disable_jog_robot_cb)
                self.m_widgets_id_dict["Disable Jog Robot"] = temp_id
                dpg.add_spacing(count=1)
                # dpg.end()
            self.m_widgets_id_dict["JogRobotHeadGroup"] = temp_jrhg
            for i in range(0, self.m_dof):
                temp_id = dpg.add_button(filter_key="Button",
                                         label="{}+##Jogger".format(self.m_joint_labels[i]), enabled=False, show=False)
                self.m_widgets_id_dict["{}+##Jogger".format(
                    self.m_joint_labels[i])] = temp_id
                if i != self.m_dof-1:
                    dpg.add_same_line(spacing=10)
            for i in range(0, self.m_dof):
                temp_id = dpg.add_button(filter_key="Button",
                                         label="{}-##Jogger".format(self.m_joint_labels[i]), enabled=False, show=False)
                self.m_widgets_id_dict["{}-##Jogger".format(
                    self.m_joint_labels[i])] = temp_id
                if i != self.m_dof-1:
                    dpg.add_same_line(spacing=10)
            dpg.add_spacing(count=2)
            temp_id = dpg.add_checkbox(label="Refer to Base Frame",
                                       default_value=True, show=False)
            self.m_widgets_id_dict["Refer to Base Frame"] = temp_id
            for i in range(0, 6):
                temp_id = dpg.add_button(filter_key="Button",
                                         label="{}+##Jogger".format(self.m_cartesian_labels[i]), enabled=False, show=False)
                self.m_widgets_id_dict["{}+##Jogger".format(
                    self.m_cartesian_labels[i])] = temp_id
                if i != 5:
                    dpg.add_same_line(spacing=10)
            for i in range(0, 6):
                temp_id = dpg.add_button(filter_key="Button",
                                         label="{}-##Jogger".format(self.m_cartesian_labels[i]), enabled=False, show=False)
                self.m_widgets_id_dict["{}-##Jogger".format(
                    self.m_cartesian_labels[i])] = temp_id
                if i != 5:
                    dpg.add_same_line(spacing=10)
            dpg.add_spacing(count=1)
            with dpg.group(label="RecordRobotStatusGroup") as temp_rrsg:
                # dpg.add_group(label="RecordRobotStatusGroup")
                dpg.add_separator()
                dpg.add_spacing(count=1)
                dpg.add_text("Record robot status")
                dpg.add_spacing(count=1)
                dpg.add_text("Joints")
                temp_id = dpg.add_button(filter_key="Button", label="Record Joints##Record",
                                         callback=self._record_robot_joints_cb, width=140, enabled=False)
                self.m_widgets_id_dict["Record Joints##Record"] = temp_id
                dpg.add_same_line(spacing=20)
                temp_id = dpg.add_button(filter_key="Button", label="Clear Joints##Record",
                                         callback=self._clear_robot_joints_cb, width=110, enabled=False)
                self.m_widgets_id_dict["Clear Joints##Record"] = temp_id
                dpg.add_same_line(spacing=20)
                temp_id = dpg.add_button(filter_key="Button", label="Save Joints##Record",
                                         callback=self._save_robot_joints_cb, width=110, enabled=False)
                self.m_widgets_id_dict["Save Joints##Record"] = temp_id
                dpg.add_spacing(count=1)
                dpg.add_text("Poses")
                temp_id = dpg.add_button(filter_key="Button", label="Record Poses##Record",
                                         callback=self._record_robot_poses_cb, width=140, enabled=False)
                self.m_widgets_id_dict["Record Poses##Record"] = temp_id
                dpg.add_same_line(spacing=20)
                temp_id = dpg.add_button(filter_key="Button", label="Clear Poses##Record",
                                         callback=self._clear_robot_poses_cb, width=110, enabled=False)
                self.m_widgets_id_dict["Clear Poses##Record"] = temp_id
                dpg.add_same_line(spacing=20)
                temp_id = dpg.add_button(filter_key="Button", label="Save Poses##Record",
                                         callback=self._save_robot_poses_cb, width=110, enabled=False)
                self.m_widgets_id_dict["Save Poses##Record"] = temp_id
                dpg.add_spacing(count=1)
                # dpg.end()
            self.m_widgets_id_dict["RecordRobotStatusGroup"] = temp_rrsg
            dpg.add_separator()
            dpg.add_spacing(count=1)
            dpg.add_text("Jog Trajectory")
            dpg.add_spacing(count=1)
            temp_id = dpg.add_button(filter_key="Button", label="Enable Jog Trajectory##Trajectory",
                                     callback=self._enable_jog_trajectory_cb, enabled=False)
            self.m_widgets_id_dict["Enable Jog Trajectory##Trajectory"] = temp_id
            dpg.add_same_line(spacing=20)
            temp_id = dpg.add_button(filter_key="Button", label="Disable Jog Trajectory##Trajectory",
                                     callback=self._disable_jog_trajectory_cb, enabled=False)
            self.m_widgets_id_dict["Disable Jog Trajectory##Trajectory"] = temp_id
            dpg.add_spacing(count=2)
            with dpg.group(label="JogTrajectoryGroup", show=False) as temp_jtg:
                # dpg.add_group(label="JogTrajectoryGroup", show=False)
                dpg.add_text("Trajectory Parameters")
                temp_id = dpg.add_input_float(label="BlendTolerance##Trajectory", default_value=0.1, width=150, format="%.4f",
                                              enabled=True, readonly=False)
                self.m_widgets_id_dict["BlendTolerance##Trajectory"] = temp_id
                dpg.add_same_line(spacing=20)
                temp_id = dpg.add_combo(label="PathType##Trajectory", items=self.m_path_type_names,
                                        default_value=self.m_path_type_names[1], width=150, callback=self._path_type_combo_cb)
                self.m_widgets_id_dict["PathType##Trajectory"] = temp_id
                dpg.add_spacing(count=1)
                temp_id = dpg.add_combo(label="TrajType##Trajectory", items=self.m_traj_type_names,
                                        default_value=self.m_traj_type_names[1], width=150, callback=self._traj_type_combo_cb)
                self.m_widgets_id_dict["TrajType##Trajectory"] = temp_id
                dpg.add_same_line(spacing=62)
                temp_id = dpg.add_input_float(label="Velocity##Trajectory", default_value=0.1, width=150, format="%.3f",
                                              enabled=False, readonly=True)
                self.m_widgets_id_dict["Velocity##Trajectory"] = temp_id
                dpg.add_spacing(count=2)
                temp_id = dpg.add_button(filter_key="Button", label="Generate Joint Traj",
                                         callback=self._generate_joint_trajectory_cb, enabled=False)
                self.m_widgets_id_dict["Generate Joint Traj"] = temp_id
                dpg.add_same_line(spacing=20)
                temp_id = dpg.add_button(filter_key="Button", label="Generate Cartesian Traj",
                                         callback=self._generate_cartesian_trajectory_cb, enabled=False)
                self.m_widgets_id_dict["Generate Cartesian Traj"] = temp_id
                dpg.add_same_line(spacing=20)
                temp_id = dpg.add_button(filter_key="Button", label="Show/Clear Traj",
                                         callback=self._show_clear_cart_trajectory_cb, enabled=False)
                self.m_widgets_id_dict["Show/Clear Traj"] = temp_id
                dpg.add_spacing(count=1)
                dpg.add_text("Jog Trajectory Progress")
                temp_id = dpg.add_progress_bar(
                    label="Jog Trajectory Progress Bar", default_value=0.0, overlay="0.0")
                self.m_widgets_id_dict["Jog Trajectory Progress Bar"] = temp_id
                dpg.add_spacing(count=1)
                temp_id = dpg.add_button(filter_key="Button",
                                         label="Jog Traj Forward", enabled=False)
                self.m_widgets_id_dict["Jog Traj Forward"] = temp_id
                dpg.add_same_line(spacing=40)
                temp_id = dpg.add_button(
                    filter_key="Button", label="Jog Traj Back", enabled=False)
                self.m_widgets_id_dict["Jog Traj Back"] = temp_id
                # dpg.end()
            self.m_widgets_id_dict["JogTrajectoryGroup"] = temp_jtg
            dpg.add_spacing(count=1)
            global bottom_status_id
            with dpg.child(label="Status Info", height=55):
                # dpg.add_child("Status Info", height=55)
                dpg.add_text("Current Status: ")
                dpg.add_same_line(spacing=10)
                bottom_status_id = dpg.add_text(
                    "Ready to initialize", label="Status Result")
                # dpg.end()
        self._initialize_controller_jogger()
        self.m_widgets_id_dict["ControllerJoggerUI"] = main_window
        dpg.set_primary_window(main_window, True)
        dpg.set_item_theme(main_window, themes.create_theme_imgui_dark())
        for key in self.m_widgets_id_dict:
            temp_id = self.m_widgets_id_dict[key]
            temp_fk = dpg.get_item_filter_key(temp_id)
            if temp_fk == "Button":
                dpg.set_item_disabled_theme(
                    temp_id, themes.create_theme_imgui_light())
        self.m_is_running = True
        while self.m_is_running:
            self._render_cb(None, None)
            dpg.render_dearpygui_frame()
        dpg.cleanup_dearpygui()
        # dpg.start_dearpygui()
        self._post_process_jogger_tools()


if __name__ == "__main__":
    Logger.SetLevelForAll(3)
    Logger.SetLevelForFiles(1)
    print(GetLogOutputPath())
    robot_sim_controller = SimController.Create("Motoman_GP12")
    jogger_ui = ControllerJoggerUI(robot_sim_controller,
                                   limits_scale=0.2,
                                   home=[0, 0, 0, 0, -np.pi/2, 0])
    jogger_ui.Run()
    embed()
