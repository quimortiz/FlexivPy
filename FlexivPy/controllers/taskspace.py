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
    def __init__(self, 
                 model, 
                 dt = 0.01,
                 ef_frame = 'link7',
                 kp = 0.5*np.diag([10, 10, 10, 10, 10, 10]), 
                 kv=  0.0*np.diag([2, 2, 2, 2, 2, 2,2]), 
                 joint_kv = np.ones(7)*8,
                 k_reg = 0.5*np.diag([10, 10, 10, 10, 10, 10, 10]),
                 dq_max = 10, 
                 T_cmd = None,
                 control_mode='velocity'):
        # define solver
        self.model = model
        self.kp = kp
        self.kv = kv
        self.k_reg = k_reg
        self.ef_frame = ef_frame
        self.q0 = self.model.q0.reshape(7,1).copy()
        self.T_cmd = T_cmd
        self.dt = dt
        self.joint_kv = joint_kv
        self.dq_max=dq_max
        try:
            self.control_mode = {'velocity':3, 'torque':2, 'position':1}[control_mode]
        except:
            raise Exception('The selected control mode is not valid!')

    def __call__(self, q, dq, T_cmd):
        _q = np.array(q).reshape(7,1)
        _dq = np.array(dq).reshape(7,1)
        info = self.model.getInfo(_q, _dq)
        _T_current = info['poses'][self.ef_frame]
        _R_current = _T_current[0:3,0:3]
        _t_current = _T_current[0:3,-1].reshape(3,1)
        _R_cmd = T_cmd[0:3,0:3]
        _t_cmd = T_cmd[0:3,-1].reshape(3,1)
        J = info['Js'][self.ef_frame]
        J_inv = np.linalg.inv(J.T@J+1e-5*np.eye(J.shape[1]))@J.T
        P = np.eye(J.shape[1]) - J_inv@J
        R_error = pin.log3(_R_cmd@_R_current.T).reshape(3,1)
        t_error = (_t_cmd - _t_current).reshape(3,1)
        pose_error = np.vstack([t_error, R_error])
        if np.linalg.det(J@J.T) > 0.001:
            dq_des = (J_inv@(self.kp@pose_error)+P@self.k_reg@(self.q0-_q)-self.kv@_dq).squeeze()
        else:
            raise Exception('The robot is too close to singularity!')

        dq_des = np.clip(dq_des, -self.dq_max*np.ones(7), self.dq_max*np.ones(7))
        return dq_des
    
    def set_target(self, T_cmd):
        self.T_cmd = T_cmd
    
    def setup(self, s):
        if self.T_cmd is None:
            info = self.model.getInfo(np.array(s.q), np.array(s.dq))
            self.T_cmd = info['poses'][self.ef_frame]

    def get_control(self, state, t):
        dq_des = self.__call__(np.array(state.q), np.array(state.dq), self.T_cmd)
        return FlexivCmd(
            q=state.q,
            dq=dq_des,
            kp=np.zeros(7),
            kv=self.joint_kv,
            mode=self.control_mode
        )

    def applicable(self, s, t):
        return True

    def goal_reached(self, s, t):
        return False

