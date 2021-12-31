#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Copyright (c) RVBUST, Inc - All rights reserved.

import os.path
from xml.dom import minidom
from xml.etree import ElementTree, cElementTree

import numpy as np

from .PyRPS import *
from .RVDFUtils.ReplaceName import ReplaceJointName, ReplaceLinkName


class RVDFEditor(object):
    def __init__(self):
        self.m_materials = []

    def AddRoot(self, robot_name):
        self.root = ElementTree.Element("robot", name=robot_name)

    def AddMaterial(self, materials):
        i = 0
        for material in materials:
            material_name = material.name
            if material.name == "":
                i = i + 1
                material_name = "material_name_" + str(i)
            self.material = ElementTree.SubElement(self.root,
                                                   "material",
                                                   name=material_name)
            ElementTree.SubElement(
                self.material,
                "color",
                rgba="%s" %
                (' '.join(format(ji, ".6f") for ji in material.color)))

            self.m_materials.append((material_name, material.color))

    def AddLink(self, link_info):

        # link_name = ReplaceLinkName(link_info.name)
        link_name = link_info.name

        self.link = ElementTree.SubElement(self.root, "link", name=link_name)

        if link_info.visual_geometry.type == GeometryType.GeometryType_Mesh:

            path = link_info.visual_geometry.path
            pose = link_info.visual_geometry.pose

            self.visual = ElementTree.SubElement(self.link, "visual")
            xyz = pose.GetR3().Coeffs()
            rpy = pose.GetSO3().RPY()
            ElementTree.SubElement(
                self.visual,
                "origin",
                rpy="%s" % (' '.join(format(ji, ".9f") for ji in rpy)),
                xyz="%s" % (' '.join(format(ji, ".9f") for ji in xyz)))
            self.geometry = ElementTree.SubElement(self.visual, "geometry")
            mesh_type = (path.split("/")[-1]).split(".")[-1]
            ElementTree.SubElement(self.geometry,
                                   "mesh",
                                   filename="package://Visual/" + link_name +
                                   "." + mesh_type.lower())

            material = link_info.visual_geometry.material
            material_name = material.name
            if material.name == "" and material.color != []:
                for material_pair in self.m_materials:
                    if np.sum(
                            np.abs(
                                np.mat(material.color) -
                                np.mat(material_pair[1]))) < 0.001:
                        material_name = material_pair[0]
                        # print("**********")
                        # embed()
            if material.color != []:
                self.material = ElementTree.SubElement(self.visual,
                                                       "material",
                                                       name=material_name)
                ElementTree.SubElement(
                    self.material,
                    "color",
                    rgba="%s" %
                    (' '.join(format(ji, ".9f") for ji in material.color)))

            path = link_info.collision_geometry.path
            pose = link_info.collision_geometry.pose

            self.collision = ElementTree.SubElement(self.link, "collision")
            ElementTree.SubElement(
                self.collision,
                "origin",
                rpy="%s" % (' '.join(format(ji, ".9f") for ji in rpy)),
                xyz="%s" % (' '.join(format(ji, ".9f") for ji in xyz)))
            self.geometry = ElementTree.SubElement(self.collision, "geometry")
            mesh_type = (path.split("/")[-1]).split(".")[-1]
            ElementTree.SubElement(self.geometry,
                                   "mesh",
                                   filename="package://Collision/" +
                                   link_name + "." + mesh_type.lower())
            material = link_info.collision_geometry.material
            material_name = material.name
            if material.name == "" and material.color != []:
                for material_pair in self.m_materials:
                    if np.sum(
                            np.abs(
                                np.mat(material.color) -
                                np.mat(material_pair[1]))) < 0.001:
                        material_name = material_pair[0]
            if material.color != []:
                self.material = ElementTree.SubElement(self.collision,
                                                       "material",
                                                       name=material_name)
                ElementTree.SubElement(
                    self.material,
                    "color",
                    rgba="%s" %
                    (' '.join(format(ji, ".9f") for ji in material.color)))

    def AddEndLink(self, end_link_name):
        self.link = ElementTree.SubElement(self.root,
                                           "link",
                                           name=end_link_name)

    def AddJoint(self, joint_info):

        # joint_name = ReplaceJointName(joint_info.name)
        joint_name = joint_info.name

        joint_axis = []
        if (joint_info.joint_type == JointType_Fixed):
            self.joint = ElementTree.SubElement(self.root,
                                                "joint",
                                                name=joint_name,
                                                type="fixed")

        if (joint_info.joint_type == JointType_Revolute):
            self.joint = ElementTree.SubElement(self.root,
                                                "joint",
                                                name=joint_name,
                                                type="revolute")
            joint_axis = joint_info.axis.GetSO3Tangent().Coeffs()
            ElementTree.SubElement(
                self.joint,
                "axis",
                xyz="%s" % (' '.join(format(ji, ".9f") for ji in joint_axis)))

        if (joint_info.joint_type == JointType_Continuous):
            self.joint = ElementTree.SubElement(self.root,
                                                "joint",
                                                name=joint_name,
                                                type="continuous")
            joint_axis = joint_info.axis.GetSO3Tangent().Coeffs()
            ElementTree.SubElement(
                self.joint,
                "axis",
                xyz="%s" % (' '.join(format(ji, ".9f") for ji in joint_axis)))

        if (joint_info.joint_type == JointType_Prismatic):
            self.joint = ElementTree.SubElement(self.root,
                                                "joint",
                                                name=joint_name,
                                                type="prismatic")
            joint_axis = joint_info.axis.GetR3Tangent().Coeffs()
            ElementTree.SubElement(
                self.joint,
                "axis",
                xyz="%s" % (' '.join(format(ji, ".9f") for ji in joint_axis)))

        parent_link_name = joint_info.parent_link_name
        parent_link_name = ReplaceLinkName(parent_link_name)
        ElementTree.SubElement(self.joint, "parent", link=parent_link_name)

        child_link_name = joint_info.child_link_name
        child_link_name = ReplaceLinkName(child_link_name)
        ElementTree.SubElement(self.joint, "child", link=child_link_name)

        xyz = joint_info.local_pose.GetR3().Coeffs()
        rpy = joint_info.local_pose.GetSO3().RPY()
        ElementTree.SubElement(
            self.joint,
            "origin",
            rpy="%s" % (' '.join(format(ji, ".9f") for ji in rpy)),
            xyz="%s" % (' '.join(format(ji, ".9f") for ji in xyz)))

        if (joint_name != "JointTip" and joint_name != "JointEffector"
                and joint_info.joint_type != JointType_Fixed):
            mimic = joint_info.mimic
            if (mimic.joint_name != ""):
                mimic_joint_name = ReplaceJointName(mimic.joint_name)
                ElementTree.SubElement(self.joint,
                                       "mimic",
                                       joint=mimic_joint_name,
                                       multiplier=str(mimic.multiplier)[0:7],
                                       offset=str(mimic.offset)[0:7])
            joint_limit = joint_info.joint_limit
            effort = joint_limit.GetMaxEffort()
            upper = joint_limit.GetMaxPosition()
            lower = joint_limit.GetMinPosition()
            velocity = joint_limit.GetMaxVelocity()
            acceleration = joint_limit.GetMaxAcceleration()
            ElementTree.SubElement(self.joint,
                                   "limit",
                                   effort=str(effort)[0:7],
                                   lower=str(lower)[0:7],
                                   upper=str(upper)[0:7],
                                   velocity=str(velocity)[0:7],
                                   acceleration=str(acceleration)[0:7])

    def AddEndJoint(self, link_name, end_link_name, joint_name):
        self.joint = ElementTree.SubElement(self.root,
                                            "joint",
                                            name=joint_name,
                                            type="fixed")

        ElementTree.SubElement(self.joint, "parent", link=link_name)
        ElementTree.SubElement(self.joint, "child", link=end_link_name)
        xyz = [0, 0, 0]
        rpy = [0, 0, 0]
        ElementTree.SubElement(
            self.joint,
            "origin",
            rpy="%s" % (' '.join(format(ji, ".9f") for ji in rpy)),
            xyz="%s" % (' '.join(format(ji, ".9f") for ji in xyz)))

    def AddEndEffector(self, attaching_pose, tip_pose, open_joint_states,
                       close_joint_states):

        self.endeffector = ElementTree.SubElement(self.root, "eef_info")
        ElementTree.SubElement(
            self.endeffector,
            "attaching_pose",
            pose="%s" %
            (' '.join(format(ji, ".9f") for ji in attaching_pose.Coeffs())))
        ElementTree.SubElement(
            self.endeffector,
            "tip_pose",
            pose="%s" %
            (' '.join(format(ji, ".9f") for ji in tip_pose.Coeffs())))
        ElementTree.SubElement(
            self.endeffector,
            'open_joint_states',
            position="%s" %
            (' '.join(format(ji, ".9f") for ji in open_joint_states)))
        ElementTree.SubElement(
            self.endeffector,
            'close_joint_states',
            position="%s" %
            (' '.join(format(ji, ".9f") for ji in close_joint_states)))

    def AddManipulator(self, dof_weight):
        self.manipulator = ElementTree.SubElement(self.root,
                                                  "manipulator_info")
        ElementTree.SubElement(self.manipulator, "manip", name="Arm")
        ElementTree.SubElement(self.manipulator, "base_link", name="BaseLink")
        ElementTree.SubElement(self.manipulator,
                               "ee_link",
                               name="EffectorLink")
        ElementTree.SubElement(self.manipulator,
                               "T_e_t",
                               pose="0.0 0.0 0.0 0.0 0.0 0.0 1.0")
        ElementTree.SubElement(
            self.manipulator,
            "dof_weights",
            weight="%s" % (' '.join(format(ji, ".9f") for ji in dof_weight)))
        home_position = Rx(len(dof_weight))
        ElementTree.SubElement(
            self.manipulator,
            "home_position",
            position="%s" %
            (' '.join(format(ji, ".9f") for ji in home_position.Coeffs())))

    def AddControllerInfo(self):
        self.controller = ElementTree.SubElement(self.root, "controller_info")
        ElementTree.SubElement(self.controller,
                               "controller_box",
                               name="",
                               software_version="")
        ElementTree.SubElement(self.controller,
                               "status",
                               jog="No",
                               upload_file="No",
                               generate_program="No")

    def AddGeneralInfo(self, brand, robot_name, dof, payload, weight, reach,
                       rept):
        self.general = ElementTree.SubElement(self.root, "general_info")
        ElementTree.SubElement(self.general, "brand", name=brand)
        ElementTree.SubElement(self.general, "dof", value=str(dof))
        ElementTree.SubElement(self.general, "type", name="Robot")
        ElementTree.SubElement(self.general, "payload", value=payload)
        ElementTree.SubElement(self.general, "weight", value=weight)
        ElementTree.SubElement(self.general, "reach", value=reach)
        ElementTree.SubElement(self.general, "repeat", value=rept)

        index = robot_name.find("_")
        image_name = robot_name[index + 1:]
        ElementTree.SubElement(self.general,
                               "image",
                               filename="package://" + image_name + ".png")

    def AddEndEffectorGeneralInfo(self, brand, endeffector_name, dof):
        self.general = ElementTree.SubElement(self.root, "general_info")
        ElementTree.SubElement(self.general, "brand", name=brand)
        ElementTree.SubElement(self.general, "dof", value=str(dof))
        ElementTree.SubElement(self.general, "type", name="EndEffector")
        index = endeffector_name.find("_")
        image_name = endeffector_name[index + 1:]
        ElementTree.SubElement(self.general,
                               "image",
                               filename="package://" + image_name + ".png")

    def AddBoundingBox(self, bounding_box):
        self.bounding_box = ElementTree.SubElement(self.root, "bounding_box")
        ElementTree.SubElement(
            self.bounding_box,
            "pose",
            value="%s" %
            (' '.join(format(ji, ".9f") for ji in bounding_box.pose.Coeffs())))
        ElementTree.SubElement(self.bounding_box,
                               "size",
                               length=str(bounding_box.length)[0:6],
                               width=str(bounding_box.width)[0:6],
                               height=str(bounding_box.height)[0:6])

    def AddSelfCollision(self, collision_matrix):
        self.collision = ElementTree.SubElement(self.root, "self_collision")
        for index, every in enumerate(collision_matrix.m_enabled_pairs):
            link1_name = every[0].GetName()
            link2_name = every[1].GetName()

            link1_name = ReplaceLinkName(link1_name)
            link2_name = ReplaceLinkName(link2_name)
            ElementTree.SubElement(self.collision,
                                   "enable_pair",
                                   link_first=link1_name,
                                   link_second=link2_name)

    def SaveRVDF(self, file_name):
        tree = cElementTree.ElementTree(self.root)
        t = minidom.parseString(ElementTree.tostring(self.root)).toprettyxml()
        tree1 = ElementTree.ElementTree(ElementTree.fromstring(t))
        tree1.write(file_name, encoding='utf-8', xml_declaration=True)


if __name__ == "__main__":
    pass
