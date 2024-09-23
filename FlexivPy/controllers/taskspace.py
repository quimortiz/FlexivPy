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


class RRTController:
    def __init__(
        self, 
        home_state=None, 
        pin_robot=None, 
        dt=0.01, 
        goal_tolerance=5 * 1e-2, 
        max_velocity=0.2, 
        visualize = False, 
        kp = 2.0 * np.array([3000.0, 3000.0, 800.0, 800.0, 200.0, 200.0, 200.0]), 
        kv = 1.5 * np.array([80.0, 80.0, 40.0, 40.0, 8.0, 8.0, 8.0]),
    ):
        self.kp = kp
        self.kv = kv
        self.visualize = visualize
        if home_state is not None:
            self.goal = home_state
        else:
            self.goal = np.array([0.0, -0.698, 0.000, 1.571, -0.000, 0.698, -0.000])
        
        if pin_robot is not None:
            self.pin_robot = pin_robot
        else:
            urdf = os.path.join(ASSETS_PATH, "r10s_with_capsules.urdf")
            # flexiv_rizon10s_kinematics.urdf"
            meshes = os.path.join(ASSETS_PATH, "meshes/")
            pin_robot = pin.RobotWrapper.BuildFromURDF(urdf, meshes)

            pin_robot.collision_model.addAllCollisionPairs()

            pin.removeCollisionPairs(
                pin_robot.model,
                pin_robot.collision_model,
                os.path.join(ASSETS_PATH, "r10s_with_capsules.srdf"),
            )
            pin_robot.rebuildData()
            self.pin_robot = pin_robot
        
        self.goal_tolerance = goal_tolerance
        self.dt = dt
        self.max_velocity = max_velocity

        self.collision_check_fn = lambda q: not pin.computeCollisions(
            self.pin_robot.model,
            self.pin_robot.data,
            self.pin_robot.collision_model,
            self.pin_robot.collision_data,
            q,
            True,
        )

        self.rrt = RRT(is_collision_free=self.collision_check_fn,
                       sample_fun=lambda: np.random.rand(7) * 2 * np.pi - np.pi,
                )

    def setup(self, s):

        start = np.array(s.q)
        goal = self.goal
        self.compute_path(start, goal)
        if self.visualize:
            self.visualize_path(self.path)

    def compute_path(self, start, goal):
        self.goal = goal
        self.start = start
        self.path = self.rrt.solve(self.start, self.goal)
        self.current_subgoal_id = 0
    
    def visualize_path(self, path):

        vizer = MeshcatVisualizer(
            self.pin_robot.model,
            self.pin_robot.collision_model,
            self.pin_robot.visual_model,
        )
        vizer.initViewer(loadModel=True)
        vizer.displayCollisions(True)
        vizer.displayVisuals(True)
        vizer.displayFrames(True)
        for q in path:
            vizer.display(q)
            input("Press Enter to continue...")

    def get_control(self, state, tic):

        # now we try to follow the path.

        # Check if I have reached the subgoal
        q = np.array(state.q)

        error = np.linalg.norm(q - self.path[self.current_subgoal_id])
        if error < self.goal_tolerance:
            self.current_subgoal_id += 1
            self.current_subgoal_id = min(self.current_subgoal_id, len(self.path) - 1)

        q_current_goal = self.path[self.current_subgoal_id]

        error = q_current_goal - q
        velocity = 2.0 * error

        if np.linalg.norm(velocity) > self.max_velocity:
            velocity = self.max_velocity * velocity / np.linalg.norm(velocity)

        desq_next = q + self.dt * velocity


        return FlexivCmd(
            q=desq_next,
            dq=velocity,
            kp=self.kp,
            kv=self.kv,
        )

    def applicable(self, s, tic):
        return True

    def goal_reached(self, s, tic):
        q = np.array(s.q)
        return np.linalg.norm(q - self.goal) < self.goal_tolerance
    

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
                 T_cmd = None):
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
        dq_des = (J_inv@(self.kp@pose_error)+P@self.k_reg@(self.q0-_q)-self.kv@_dq).squeeze()
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
        )

    def applicable(self, s, t):
        return True

    def goal_reached(self, s, t):
        return False

