# a pinocchio model of the robot

import os

import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.visualize import MeshcatVisualizer
from sys import argv
import os
from os.path import dirname, join, abspath


ASSETS_PATH = "FlexivPy/assets/"


class FlexivModel:

    def __init__(self, render=False, urdf=None, meshes_dir=None):
        """ """
        self.urdf = urdf
        self.meshes_dir = meshes_dir
        if urdf is None:
            self.urdf = os.path.join(ASSETS_PATH, "flexiv_rizon10s_kinematics.urdf")
        if meshes_dir is None:
            self.meshes_dir = os.path.join(ASSETS_PATH, "meshes")

        robot = RobotWrapper.BuildFromURDF(self.urdf, self.meshes_dir)

        VISUALIZER = MeshcatVisualizer
        robot.setVisualizer(VISUALIZER())
        robot.initViewer()
        robot.loadViewerModel("pinocchio")

        q0 = robot.q0
        robot.display(q0)
        self.robot = robot
