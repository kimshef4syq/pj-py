#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Copyright (c) RVBUST, Inc - All rights reserved.

from RVBUST import RCI
from RVBUST import Vis
import numpy as np
from IPython import embed


def GetVersion():
    return RCI.GetRCIVersion()


Pose = RCI.SE3
JointPosition = RCI.Rx


class Robot(object):
    """Robot is a user-friendly API, includes:
        - Robot Database
        - RobotModel
        - Manipulator
        - EndEffector
        - Kinematics
        - Trajectory
        - Controller
    """

    def __init__(self, robot_type):
        self.m_has_setup = False
        self.m_logger = RCI.Logger.GetConsoleLogger("Robot")

        if isinstance(robot_type, str):
            self.m_name = robot_type
            db = RCI.Database()
            res, path = db.GetRobotModelFile(self.m_name)
            if not res:
                self.m_logger.Error(
                    "Cannot find robot model {}".format(self.m_name))
                return
            self.m_robot_model = RCI.LoadRobotModelFromFile(path)
        elif isinstance(robot_type, RCI.RobotModel):
            self.m_robot_model = robot_type
            self.m_name = self.m_robot_model.GetName()
        else:
            self.m_logger.Error(
                "Invalid input robot type {}".format(robot_type))
            return

        self.m_manipulator = self.m_robot_model.GetActiveManipulator()
        self.m_gripper = self.m_manipulator.GetActiveEndEffector()
        self.m_controller = RCI.SimController.Create(self.m_manipulator)
        self.m_kin_solver = self.m_controller.GetKinSolver()
        self.m_joint_limits = RCI.GlobalUniformProfileConstraints(
            self.m_controller.GetJointLimits())
        cart_limits = self.m_controller.GetCartesianLimits()
        self.m_cart_limits = RCI.GlobalUniformProfileConstraints(
            cart_limits[0], cart_limits[1], cart_limits[2])
        self.m_base_control = RCI.BasicMotionUtilities(self.m_controller)
        self.m_logger.Info("Successfully setup robot {}".format(self.m_name))
        self.m_has_setup = True
        return

    @staticmethod
    def ListRobotBrand():
        db = RCI.Database()  # Manage robot models
        return db.ListRobotBrand()

    @staticmethod
    def ListRobotNamesByBrand(robot_brand):
        db = RCI.Database()
        return db.ListRobotNamesByBrand(robot_brand)

    def HasSetup(self) -> bool:
        return self.m_has_setup

    def GetName(self) -> str:
        return self.m_name

    def GetDoF(self):
        return self.m_robot_model.GetDoF()

    def GetJointLimits(self):
        return self.m_controller.GetJointLimits()

    def SetJointLimits(self, joint_limits):
        if self.m_controller.SetJointLimits(joint_limits):
            self.m_joint_limits = RCI.GlobalUniformProfileConstraints(
                self.m_controller.GetJointLimits())
            return True
        else:
            return False

    def GetCartesianLimits(self):
        return self.m_controller.GetCartesianLimits()

    def SetCartesianLimits(self, max_vels, max_accs, max_jerks):
        self.m_controller.SetCartesianLimits(max_vels, max_accs, max_jerks)
        self.m_cart_limits = RCI.GlobalUniformProfileConstraints(
            max_vels, max_accs, max_jerks)
        return True

    def Connect(self, ip: str = "127.0.0.1"):
        if not self.HasSetup():
            return False
        return self.m_controller.Connect(ip)

    def Disconnet(self):
        return self.m_controller.Disconnet()

    def EnableRobot(self):
        return self.m_controller.EnableRobot()

    def DisableRobot(self):
        return self.m_controller.DisableRobot()

    def SetSpeedRatio(self, ratio: float):
        return self.m_controller.SetSpeedRatio(ratio)

    def GetSpeedRatio(self) -> float:
        return self.m_controller.GetSpeedRatio()

    def SetPayload(self, payload: RCI.Payload):
        return self.m_controller.SetPayload(payload)

    def GetPayload(self):
        return self.m_controller.GetPayload()

    def SetTCP(self, tcp: Pose):
        return self.m_controller.SetTCP(tcp)

    def GetTCP(self):
        return self.m_controller.GetTCP()

    def SetEndEffector(self, eef):
        pass

    def GetErrorCode(self) -> str:
        _, code = self.m_controller.GetErrorCode()
        return code

    def ResetRobot(self):
        return self.m_controller.ResetRobot()

    def Grab(self, body, do_port, do_value=True):
        return self.m_controller.Grab(body, do_port, do_value)

    def Release(self, do_port, do_value=True):
        return self.m_controller.Release(do_port, do_value)

    def SetDigitalOutput(self, port, value):
        return self.m_controller.SetDigitalOutput(port, value)

    def GetDigitalOutput(self, port) -> (bool): 
        return self.m_controller.GetDigitalOutput(port)

    def SetDigitalInput(self, port, value):
        return self.m_controller.SetDigitalInput(port, value)

    def GetDigitalInput(self, port) -> (bool) :
        return self.m_controller.GetDigitalInput(port)

    def SetAnalogOutput(self, port, value):
        return self.m_controller.SetAnalogOutput(port, value)

    def GetAnalogOutput(self, port):
        return self.m_controller.GetAnalogOutput(port)

    def SetAnalogInput(self, port, value):
        return self.m_controller.SetAnalogInput(port, value)

    def GetAnalogInput(self, port):
        return self.m_controller.GetAnalogInput(port)

    def StopMove(self):
        return self.m_controller.StopMove()

    def CreateTrajectory(self):
        # todo:
        pass

    def ExecuteTrajectory(self, traj):
        return self.m_controller.ExecuteTrajectory(traj)

    def _ConvertToJointPosition(self, waypoint, q_seed=None):
        if q_seed is None:
            _, q_seed = self.m_controller.GetJointPosition()

        if isinstance(waypoint, JointPosition):
            # if input waypoint is a Joint position
            return True, waypoint
        elif (isinstance(waypoint, list) or isinstance(waypoint, np.ndarray)) and (len(waypoint) == self.m_controller.GetDoF()):
            # if input waypoint is a list, convert to Joint position
            try:
                joint_pose = JointPosition(waypoint)
            except AttributeError:
                self.m_logger.Debug(
                    "Invalid waypoint type of {}".format(waypoint))
                return False, q_seed
            return True, joint_pose
        elif isinstance(waypoint, Pose):
            # if input waypoint is a Cartesian pose, use ik to convert to joint position
            res, ik_report, _ = self.m_kin_solver.GetNearestIK(
                waypoint, q_seed)
            if res == RCI.RVSReturn_Success:
                return True, ik_report[0]
            else:
                self.m_logger.Debug(
                    "Cannot find IK for waypoint {}".format(waypoint))
                return False, q_seed
        else:
            self.m_logger.Debug(
                "Input waypoint {} is in valid".format(waypoint))
            return False, q_seed

    def MoveJoint(self, waypoint, wait=True, blend_tolerance=0.1, min_duration=0) -> bool:
        # Add current joint positions
        qlist = [self.m_controller.GetJointPosition()[1]]
        res, joint_pose = self._ConvertToJointPosition(waypoint, qlist[0])
        if res:
            qlist.append(joint_pose)
        elif (isinstance(waypoint, list) or isinstance(waypoint, np.ndarray)) and (len(waypoint) > 0):
            for w in waypoint:
                res, joint_pose = self._ConvertToJointPosition(w, qlist[-1])
                if res:
                    qlist.append(joint_pose)
                else:
                    self.m_logger.Error(
                        "Cannot generate joint path from input {}".format(waypoint))
                    return False
        else:
            self.m_logger.Error(
                "Cannot generate joint path from input {}".format(waypoint))
            return False
        path = RCI.PathBezier5thBlendRn(qlist, blend_tolerance)
        traj = RCI.TrajectoryDoubleSRn(
            path, self.m_joint_limits, min_duration=min_duration)
        return (self.m_controller.ExecuteTrajectory(traj, wait) == RCI.RVSReturn_Success)

    def _ConvertToCartesionPose(self, waypoint):
        if isinstance(waypoint, JointPosition):
            # if input waypoint is a Joint position, use fk to convert to joint position
            res, cart_pose = self.m_kin_solver.GetPositionFK(waypoint)
            if res == RCI.RVSReturn_Success:
                return True, cart_pose
            else:
                return False, None
        elif (isinstance(waypoint, list) or isinstance(waypoint, np.ndarray)) and (len(waypoint) == self.m_controller.GetDoF()):
            # if input waypoint is a list, convert to Joint position
            try:
                joint_pose = JointPosition(waypoint)
                res, cart_pose = self.m_kin_solver.GetPositionFK(joint_pose)
                if res == RCI.RVSReturn_Success:
                    return True, cart_pose
                else:
                    return False, None
            except AttributeError:
                self.m_logger.Debug(
                    "Invalid waypoint type of {}".format(waypoint))
                return False, None
        elif isinstance(waypoint, Pose):
            # if input waypoint is a Cartesian pose
            return True, waypoint
        else:
            self.m_logger.Debug(
                "Input waypoint {} is in valid".format(waypoint))
            return False, None

    def MoveLinear(self, waypoint, wait=True, blend_tolerance=0.1, min_duration=0, tool_velocity=0) -> bool:
        # Add current pose
        wlist = [self.m_controller.GetPose()[1]]
        res, wpose = self._ConvertToCartesionPose(waypoint)
        if res:
            wlist.append(wpose)
        elif (type(waypoint) == list or type(waypoint) == np.ndarray) and (len(waypoint) > 0):
            for w in waypoint:
                res, wpose = self._ConvertToCartesionPose(w)
                if res:
                    wlist.append(wpose)
                else:
                    self.m_logger.Error(
                        "Cannot generate Cartesian path from input {}".format(waypoint))
                    return False
        else:
            self.m_logger.Error(
                "Cannot generate Cartesian path from input {}".format(waypoint))
            return False
        ws = [RCI.R3xSO3(wi.Coeffs()) for wi in wlist]
        # todo: we should check path validation here
        path = RCI.PathBezier5thBlendR3xSO3(ws)
        traj = RCI.TrajectoryDoubleSR3xSO3(path, self.m_cart_limits)
        return (self.m_controller.ExecuteTrajectory(traj, wait) == RCI.RVSReturn_Success)

    def MoveBX(self, v, wait=True):
        return self.m_base_control.MoveBX(v, wait)

    def MoveBY(self, v, wait=True):
        return self.m_base_control.MoveBY(v, wait)

    def MoveBZ(self, v, wait=True):
        return self.m_base_control.MoveBZ(v, wait)

    def RotateBX(self, v, wait=True):
        return self.m_base_control.RotateBX(v, wait)

    def RotateBY(self, v, wait=True):
        return self.m_base_control.RotateBY(v, wait)

    def RotateBZ(self, v, wait=True):
        return self.m_base_control.RotateBZ(v, wait)

    def MoveTX(self, v, wait=True):
        return self.m_base_control.MoveTX(v, wait)

    def MoveTY(self, v, wait=True):
        return self.m_base_control.MoveTY(v, wait)

    def MoveTZ(self, v, wait=True):
        return self.m_base_control.MoveTZ(v, wait)

    def RotateTX(self, v, wait=True):
        return self.m_base_control.RotateTX(v, wait)

    def RotateTY(self, v, wait=True):
        return self.m_base_control.RotateTY(v, wait)

    def RotateTZ(self, v, wait=True):
        return self.m_base_control.RotateTZ(v, wait)

    def GetHomePosition(self):
        return self.m_manipulator.GetHomePosition()

    def GoHome(self):
        return self.MoveJoint(self.GetHomePosition())

    def GetJointPosition(self):
        return self.m_controller.GetJointPosition()[1]

    def GetPose(self):
        return self.m_controller.GetPose()[1]

    def GetPositionFK(self, joint_value):
        return self.m_kin_solver.GetPositionFK(joint_value)

    def GetPositionIK(self, pose, joint_seed=JointPosition(0)):
        return self.m_kin_solver.GetPositionIK(pose, joint_seed)

    def GetNearestIK(self, pose, joint_values_seed):
        return self.m_kin_solver.GetNearestIK(pose, joint_values_seed)


class Simulator(object):
    """Simulator is a user-friendly API, includes:
        - Environment
        - RobotVis
        - Collision Checker (todo)
        - ODE (todo)
    """

    def __init__(self):
        # Launch RobotVis for visualization
        self.m_view = Vis.View("RobotDev Simulator")
        self.m_logger = RCI.Logger.GetConsoleLogger("Simulator")
        self.m_rvis = RCI.RobotVis(self.m_view)
        self.m_robots = []

    def LoadEnvironment(self, env_file: str) -> bool:
        """Load Environment from file

        : param env: [description]
        : type env: RCI.Environment
        : return: [description]
        : rtype: bool
        """
        self.Clear()
        env = RCI.Environment()
        if not env.LoadFromFile(env_file):
            return False

        if not self.m_rvis.LoadEnvironment(env):
            return False

        robot_models = self.m_rvis.m_env.GetAllRobotModels()
        for robot_model in robot_models:
            robot = Robot(robot_model)
            self.m_robots.append(robot)

        return True

    def SaveEnvironment(self, path):
        return self.m_rvis.m_env.SaveToFile(path)

    def Clear(self):
        self.m_rvis.m_env.Clear()
        self.m_view.Clear()
        self.m_robots = []

    def AddRobot(self, robot: Robot):
        if self.m_robots.count(robot) == 0:
            self.m_rvis.AddBody(robot.m_robot_model)
            self.m_robots.append(robot)
            return True
        else:
            self.m_logger.Error(
                "Robot {} is already in simulator".format(robot.GetName()))
            return False

    def AddBox(self, length: float, width: float, height: float):
        box = RCI.Multibody()
        box.InitFromBox(length, width, height)
        self.m_rvis.AddBody(box)
        return box

    def AddCylinder(self, radius: float, height: float):
        cylinder = RCI.Multibody()
        cylinder.InitFromCylinder(radius, height)
        self.m_rvis.AddBody(cylinder)
        return cylinder

    def AddCone(self, height: float, radius: float):
        cone = RCI.Multibody()
        cone.InitFromCone(height, radius)
        self.m_rvis.AddBody(cone)
        return cone

    def AddSphere(self, radius: float):
        sphere = RCI.Multibody()
        sphere.InitFromSphere(radius)
        self.m_rvis.AddBody(sphere)
        return sphere

    def AddPlane(self, length: float, width: float, unit_l: float, unit_w: float):
        plane = RCI.Multibody()
        plane.InitFromPlane(length, width, unit_l, unit_w)
        self.m_rvis.AddBody(plane)
        return plane

    def AddMeshFile(self, file: str):
        mul = RCI.Multibody()
        mul.InitFromMeshFile(file)
        self.m_rvis.AddBody(mul)
        return mul

    def GetRobots(self):
        return self.m_robots

    def Remove(self, body):
        if isinstance(body, Robot):
            self.m_rvis.RemoveBody(body.m_robot_model)
            self.m_robots.remove(body)
        else:
            self.m_rvis.RemoveBody(body)

    def Copy(self, body):
        if isinstance(body, RCI.Multibody):
            body_copy = body.Copy()
            self.m_rvis.AddBody(body_copy)
            return body_copy

    def GetName(self, body):
        return body.GetUniqueName()

    def SetPose(self, body, pose):
        if isinstance(body, Robot):
            return body.m_robot_model.SetBaseTransformation(pose)
        return body.SetBaseTransformation(pose)

    def GetPose(self, body):
        if isinstance(body, Robot):
            return body.m_robot_model.GetBaseTransformation()
        return body.GetBaseTransformation()

    def ChooseShowMode(self, body=None, mode=1):
        return self.m_rvis.ChooseShowMode(body, mode)

    def PlotFrame(self, pose, axis_len=0.2, axis_size=2):
        return self.m_rvis.PlotFrame(pose, axis_len, axis_size)

    def PlotPath(self, robot, waypoints):
        pass

    def PlotTraj(self, robot, traj):
        pass

    def DeletePlot(self, h):
        return self.m_rvis.Delete(h)

    def StartDragging(self, drag_body, drag_robot_base=False):
        if isinstance(drag_body, Robot):
            if drag_robot_base:
                self.m_rvis.StartDragging(drag_body.m_robot_model)
            else:
                self.m_rvis.StartDragging(drag_body.m_manipulator)
        else:
            self.m_rvis.StartDragging(drag_body)

    def StopDragging(self):
        self.m_rvis.StopDragging()


if __name__ == "__main__":
    robot = Robot("FANUC_LR_Mate_200iD")
    simulator = Simulator()
    simulator.AddRobot(robot)
    robot.Connect()
    robot.EnableRobot()

    box = simulator.AddBox(.1, .2, .1)
    simulator.SetPose(box, Pose([.3, 0, .2], [0, 0, 0]))
    robot.MoveBZ(-.1)

    simulator.StartDragging(robot, drag_robot_base=True)
    embed()
    simulator.StopDragging()
    embed()
    simulator.StartDragging(robot, drag_robot_base=False)
    embed()
    simulator.StopDragging()
    embed()
