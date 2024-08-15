import argparse
import os
import pathlib
from typing import Optional

import numpy as np
import rerun as rr  # pip install rerun-sdk
import scipy.spatial.transform as st
import trimesh
import trimesh.visual
from PIL import Image
from urdf_parser_py import urdf as urdf_parser

import pinocchio as pin
import hppfcl
import time

import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper

import FlexivPy.robot.vis.rerunio as rerunio


if __name__ == "__main__":

    rr.init("Visualize Robot!", spawn=True)
    prefix = "Flexiv"


    ASSETS_PATH = "FlexivPy/assets/"

    urdf = os.path.join(ASSETS_PATH, "flexiv_rizon10s_kinematics.urdf")
    meshes_dir = os.path.join(ASSETS_PATH, "meshes")

    q0 = np.array( [ 0.000, -0.698, 0.000, 1.571, -0.000, 0.698, -0.000 ] )
    robot = RobotWrapper.BuildFromURDF(urdf, meshes_dir)

    urdf_logger = rerunio.Robot_logger_rerunio(robot, prefix)
    urdf_logger.log(q= q0 , log_meshes = True)

    dt = 0.05
    for i in range(1000):
        q = q0 + 0.1 * np.sin(i * 0.1) * np.ones(7) + 0.001 * np.random.randn(7)
        urdf_logger.log(q= q, log_meshes = False)

        for j in range(7):
            rr.log(f"joint/q{j}", rr.Scalar(q[j]))

        time.sleep(dt)





