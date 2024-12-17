import os
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.visualize import MeshcatVisualizer
import pinocchio as pin

import os
import numpy as np
from FlexivPy import ASSETS_PATH


class FlexivModel:
    def __init__(self, render=False, urdf=None, meshes_dir=None, q0=None):
        """ """
        self.urdf = urdf
        self.meshes_dir = meshes_dir
        if urdf is None:
            self.urdf = os.path.join(ASSETS_PATH, "flexiv_rizon10s_kinematics.urdf")
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
            self.q0 = np.array([0.0, -0.698, 0.000, 1.571, -0.000, 0.698, -0.000])
        else:
            self.q0 = q0
        self.q0.flags.writeable = False

        self.frmae_names = [
            "link1",
            "link2",
            "link3",
            "link4",
            "link5",
            "link6",
            "link7",
        ]
        self.jacobian_frame = pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        self.getInfo(self.q0, np.zeros_like(self.q0))

    def display(self, q):
        if self.render:
            self.vizer.display(q)
        else:
            print("warning: display is not enabled")

    def pin_se3_to_mat(self, pin_se3):
        t = pin_se3.translation.reshape(3, 1)
        R = pin_se3.rotation.reshape(3, 3)
        return np.vstack([np.hstack([R, t]), np.array([0, 0, 0, 1])])

    def getInfo(self, q, dq):
        _q = q.squeeze()
        _dq = dq.squeeze()
        self.robot.computeJointJacobians(_q)
        self.robot.framesForwardKinematics(_q)
        self.robot.centroidalMomentum(_q, _dq)

        frame_Js = {
            frame_name: self.robot.getFrameJacobian(
                self.robot.model.getFrameId(frame_name), self.jacobian_frame
            )
            for frame_name in self.frmae_names
        }
        frame_poses = {
            frame_name: self.pin_se3_to_mat(
                self.robot.data.oMf[self.robot.model.getFrameId(frame_name)]
            )
            for frame_name in self.frmae_names
        }

        # Get dynamics parameters
        Minv = pin.computeMinverse(self.robot.model, self.robot.data, _q)[:7, :7]
        M = self.robot.mass(_q)[:7, :7]
        nle = self.robot.nle(_q, _dq)[:7]
        gravity = self.robot.gravity(_q)[:7]

        self.info = {
            "M": M,
            "Minv": Minv,
            "C": nle[:7] - gravity,
            "G": gravity,
            "Js": frame_Js,
            "poses": frame_poses,
        }
        return self.info
