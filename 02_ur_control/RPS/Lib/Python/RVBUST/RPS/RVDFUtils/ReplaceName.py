#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Copyright (c) RVBUST, Inc - All rights reserved.
import os
import re
import sys

from IPython import embed

link_old_name_1 = ["base_link", "link_1", "link_2",
                   "link_3", "link_4", "link_5", "link_6"]

link_old_name_2 = ["base_link", "link_s", "link_l",
                   "link_u", "link_r", "link_b", "link_t"]

link_old_name_3 = ["base_link", "link_1_s", "link_2_l",
                   "link_3_u", "link_4_r", "link_5_b", "link_6_t"]

link_old_name_4 = ["MH5_BASE_AXIS", "MH5_S_AXIS", "MH5_L_AXIS",
                   "MH5_U_AXIS", "MH5_R_AXIS", "MH5_B_AXIS", "MH5_T_AXIS"]

link_old_name_5 = ["base_link", "shoulder_Link", "upperArm_Link", "foreArm_Link",
                   "wrist1_Link", "wrist2_Link", "wrist3_Link"]

link_new_name = ["BaseLink", "Link1", "Link2",
                 "Link3", "Link4", "Link5", "Link6"]


joint_old_name_1 = ["joint_1", "joint_2",
                    "joint_3", "joint_4", "joint_5", "joint_6"]

joint_old_name_2 = ["joint_s", "joint_l",
                    "joint_u", "joint_r", "joint_b", "joint_t"]

joint_old_name_3 = ["joint_1_s", "joint_2_l",
                    "joint_3_u", "joint_4_r", "joint_5_b", "joint_6_t"]

joint_old_name_4 = ["shoulder_joint", "upperArm_joint",
                    "foreArm_joint", "wrist1_joint", "wrist2_joint", "wrist3_joint"]

joint_new_name = ["Joint1", "Joint2", "Joint3",
                  "Joint4", "Joint5", "Joint6"]


def ReplaceLinkName(name):
    for i in range(0, 7):
        if name == link_old_name_1[i]:
            name = link_new_name[i]
            break
        if name == link_old_name_2[i]:
            name = link_new_name[i]
            break
        if name == link_old_name_3[i]:
            name = link_new_name[i]
            break
        if name == link_old_name_4[i]:
            name = link_new_name[i]
            break
        if name == link_old_name_5[i]:
            name = link_new_name[i]
            break
    return name


def ReplaceJointName(name):
    for i in range(0, 6):
        if name == joint_old_name_1[i]:
            name = joint_new_name[i]
            break
        if name == joint_old_name_2[i]:
            name = joint_new_name[i]
            break
        if name == joint_old_name_3[i]:
            name = joint_new_name[i]
            break
        if name == joint_old_name_4[i]:
            name = joint_new_name[i]
            break

    return name
