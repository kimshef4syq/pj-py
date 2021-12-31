#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Copyright (c) RVBUST, Inc - All rights reserved.

from .RVDFUtils.URDFToRVDF import URDFToRVDF
from .RVDFUtils.XacroToURDF import XacroToURDF


def XacroToRVDF(file_path):
    if not os.path.exists(file_path):
        print("The file not exist!")
        return False

    if not os.path.isfile(file_path):
        print("This is not a file!")
        return False

    file_type = file_path.split(".")[-2]
    urdf_file = file_path
    if file_type == "xacro":
        res_urdf, urdf_file = XacroToURDF(file_path)
        if res_urdf:
            print("\n Success to generation urdf file!\n")
        else:
            print("\n Generation urdf failed!\n")
            return False
    URDFToRVDF(urdf_file)
    return True
