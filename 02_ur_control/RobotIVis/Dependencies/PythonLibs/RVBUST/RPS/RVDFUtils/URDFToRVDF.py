#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Copyright (c) RVBUST, Inc - All rights reserved.

import os
import re
import shutil
import sys
from shutil import copyfile, copytree, ignore_patterns

save_path = GetDataPath() + "Multibody/RobotModels"


def RVDF(robot_model, path):
    generator_rvdf = GeneratorRVDF()
    name = robot_model.GetName()
    brand = name.split("_")[0]
    robot_name = brand + "_" + name[len(brand) + 1:].upper()
    generator_rvdf.Init(robot_name)
    materials = []

    links = robot_model.GetLinks()

    for link in links:
        if link.GetInfo().visual_geometry.type == GeometryType.GeometryType_Mesh:
            link_visual_material = link.GetInfo().visual_geometry.material
            link_collision_material = link.GetInfo().collision_geometry.material
            is_visual_exit = False
            is_collision_exit = False
            for material in materials:
                if material.color == link_visual_material.color or link_visual_material.color == []:
                    is_visual_exit = True
                if material.color == link_collision_material.color or link_collision_material.color == []:
                    is_collision_exit = True

            if not is_visual_exit and link_visual_material.color != []:
                materials.append(link_visual_material)
            if not is_collision_exit and link_collision_material.color != []:
                materials.append(link_collision_material)

    generator_rvdf.AddMaterial(materials)

    for link in links:
        generator_rvdf.AddLink(link.GetInfo())

    generator_rvdf.AddEndEffectorLink()

    joints = robot_model.GetJoints()
    for joint in joints:
        generator_rvdf.AddJoint(joint.GetInfo())

    generator_rvdf.AddEndEffectorJoint()

    dof_weight = robot_model.ComputeDoFWeights()
    generator_rvdf.AddManipulator(dof_weight)
    generator_rvdf.AddControllerInfo()

    dof = robot_model.GetDoF()
    generator_rvdf.AddGeneralInfo(brand, robot_name, dof)

    bounding_box = GetBoundingBox(robot_model)
    generator_rvdf.AddBoundingBox(bounding_box)

    ComputeSelfCollisionMatrix(robot_model, 1000)
    collision_matrix = robot_model.GetSelfCollisionMatrix()
    generator_rvdf.AddSelfCollision(collision_matrix)

    generator_rvdf.SaveRVDF(path)


def URDFToRVDF(urdf_path):

    robot_model = RobotModel()
    robot_model.InitFromURDF(urdf_path)

    name = robot_model.GetName()
    brand = name.split("_")[0]
    robot_name = name[len(brand) + 1:].upper()

    urdf_save_path = save_path + "/" + \
        brand + "/" + \
        robot_name

    if os.path.exists(urdf_save_path):
        print("***********************")
        print(" the path is exist, please delete it: ",
              urdf_save_path)
    else:
        os.makedirs(urdf_save_path)
        os.makedirs(urdf_save_path + "/Collision")
        os.makedirs(urdf_save_path + "/Visual")
        links = robot_model.GetLinks()

        for link in links:
            if(link.GetInfo().visual_geometry.type != GeometryType.GeometryType_None):
                mesh_path = link.GetInfo().visual_geometry.path

                # chang the link mesh name
                link_mesh_name = mesh_path.split(
                    "/")[-1].split(".")[0]
                link_type = mesh_path.split(
                    "/")[-1].split(".")[1]
                link_mesh_name = ReplaceLinkName(
                    link_mesh_name)
                copyfile(mesh_path, urdf_save_path +
                         "/Visual" + "/" + link_mesh_name + "." + link_type.lower())

            if(link.GetInfo().collision_geometry.type != GeometryType.GeometryType_None):
                mesh_path = link.GetInfo().collision_geometry.path
                link_mesh_name = mesh_path.split(
                    "/")[-1].split(".")[0]
                link_type = mesh_path.split(
                    "/")[-1].split(".")[1]
                link_mesh_name = ReplaceLinkName(
                    link_mesh_name)

                copyfile(mesh_path, urdf_save_path +
                         "/Collision" + "/" + link_mesh_name + "." + link_type.lower())

        copyfile(urdf_path, urdf_save_path +
                 "/" + urdf_path.split("/")[-1])

        rvdf_path = urdf_save_path + "/" + robot_name + ".rvdf"

        RVDF(robot_model, rvdf_path)

        print("\n Success to generation rvdf file!\n")
        print("\n The file path is: ", urdf_save_path)
