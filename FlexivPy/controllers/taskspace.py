import numpy as np
import pinocchio as pin
from FlexivPy.robot.dds.flexiv_messages import (
    FlexivCmd,
)
from FlexivPy.controllers.utils import *
from pinocchio.visualize import MeshcatVisualizer
import os
from FlexivPy import ASSETS_PATH
from FlexivPy.planners.rrt import RRT


class DiffIKController:
    def __init__(
        self,
        model,
        T_cmd_fun,
        dt=0.01,
        ef_frame="link7",
        kp=0.5 * np.diag([10, 10, 10, 10, 10, 10]),
        kv=0.0 * np.diag([2, 2, 2, 2, 2, 2, 2]),
        joint_kv=1.5 * np.array([80.0, 80.0, 40.0, 40.0, 8.0, 8.0, 8.0]),
        joint_kp=2.0 * np.array([3000.0, 3000.0, 800.0, 800.0, 200.0, 200.0, 200.0]),
        k_reg=0.5 * np.diag([10, 10, 10, 10, 10, 10, 10]),
        dq_max=10,
        control_mode="velocity",
        max_error=0.05,
    ):
        # define solver
        self.model = model
        self.kp = kp
        self.kv = kv
        self.k_reg = k_reg
        self.ef_frame = ef_frame
        self.q0 = self.model.q0.reshape(7, 1).copy()
        self.T_cmd_fun = T_cmd_fun
        self.T_cmd = None
        self.dt = dt
        self.joint_kv = joint_kv
        self.joint_kp = joint_kp
        self.dq_max = dq_max
        self.max_error = max_error
        try:
            self.control_mode = {"velocity": 3, "torque": 2, "position": 1}[
                control_mode
            ]
        except:
            raise Exception("The selected control mode is not valid!")

    def __call__(self, q, dq, T_cmd):
        _q = np.array(q).reshape(7, 1)
        _dq = np.array(dq).reshape(7, 1)
        info = self.model.getInfo(_q, _dq)
        _T_current = info["poses"][self.ef_frame]
        _R_current = _T_current[0:3, 0:3]
        _t_current = _T_current[0:3, -1].reshape(3, 1)
        _R_cmd = T_cmd[0:3, 0:3]
        _t_cmd = T_cmd[0:3, -1].reshape(3, 1)

        J = info["Js"][self.ef_frame]
        J_inv = np.linalg.inv(J.T @ J + 1e-5 * np.eye(J.shape[1])) @ J.T
        P = np.eye(J.shape[1]) - J_inv @ J
        R_error = pin.log3(_R_cmd @ _R_current.T).reshape(3, 1)
        t_error = (_t_cmd - _t_current).reshape(3, 1)
        if np.linalg.norm(t_error) > self.max_error:
            raise Exception("The goal pose is too far!")
        pose_error = np.vstack([t_error, R_error])
        if np.linalg.det(J @ J.T) > 0.00001:
            dq_des = (
                J_inv @ (self.kp @ pose_error)
                + P @ self.k_reg @ (self.q0 - _q)
                - self.kv @ _dq
            ).squeeze()
        else:
            raise Exception("The robot is too close to singularity!")

        dq_des = np.clip(dq_des, -self.dq_max * np.ones(7), self.dq_max * np.ones(7))
        return dq_des

    def set_target(self, T_cmd):
        self.T_cmd = T_cmd

    def setup(self, s):
        if self.T_cmd is None:
            info = self.model.getInfo(np.array(s.q), np.array(s.dq))
            self.T_cmd = info["poses"][self.ef_frame]
        self.q_des = np.array(s.q)

    def get_control(self, state, t):
        self.T_cmd = self.T_cmd_fun(state)
        dq_des = self.__call__(np.array(state.q), np.array(state.dq), self.T_cmd)
        self.q_des += self.dt * dq_des
        if self.control_mode == 3:
            return FlexivCmd(dq=dq_des, mode=self.control_mode)
        else:
            return FlexivCmd(
                q=self.q_des,
                dq=dq_des,
                kp=self.joint_kp,
                kv=self.joint_kv,
                mode=self.control_mode,
            )

    def applicable(self, s, t):
        return True

    def goal_reached(self, s, t):
        return False
