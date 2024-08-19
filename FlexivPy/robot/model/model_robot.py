# a pinocchio model of the robot

import os

import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.visualize import MeshcatVisualizer
from sys import argv
import os
from os.path import dirname, join, abspath
import numpy as np


ASSETS_PATH = "FlexivPy/assets/"


class FlexivModel:

    def __init__(self, render=False, urdf=None, meshes_dir=None, q0=None):
        """ """
        self.urdf = urdf
        self.meshes_dir = meshes_dir
        if urdf is None:
            self.urdf = os.path.join(ASSETS_PATH, "flexiv_rizon10s_kinematics.urdf")
            # self.urdf = os.path.join(ASSETS_PATH, "with_capsules.urdf")
        if meshes_dir is None:
            self.meshes_dir = os.path.join(ASSETS_PATH, "meshes")

        robot = RobotWrapper.BuildFromURDF(self.urdf, self.meshes_dir)
        self.render = render
        if self.render:
            self.vizer = MeshcatVisualizer(
                robot.model, robot.collision_model, robot.visual_model
            )
            self.vizer.initViewer(loadModel=True)
            self.vizer.display(robot.q0)

        self.robot = robot

        if q0 is None:
            self.q0 = np.array([0.0, -0.75, 0.0, 1.5, 0.0, 0.75, 0.0])
        else:
            self.q0 = q0

    def display(self, q):
        if self.render:
            self.vizer.display(q)
        else:
            print("warning: display is not enabled")
