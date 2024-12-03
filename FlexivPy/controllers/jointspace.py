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
        control_mode='velocity'
    ):
        try:
            self.control_mode = {'velocity':3, 'torque':2, 'position':1}[control_mode]
        except:
            raise Exception('The selected control mode is not valid!')
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

        # self.collision_check_fn = lambda q:  True

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
            mode = self.control_mode
        )

    def applicable(self, s, tic):
        return True

    def goal_reached(self, s, tic):
        q = np.array(s.q)
        return np.linalg.norm(q - self.goal) < self.goal_tolerance


class JointFloating:
    def __init__(self, kv_scale=1., dt=0.01):
        self.kv_scale = kv_scale
        self.kv = kv_scale * np.array([80.0, 80.0, 40.0, 40.0, 8.0, 8.0, 8.0])
        self.dt = dt

    def setup(self, s):
        pass

    def get_control(self, state, tic):
        return FlexivCmd(kv=self.kv)

    def applicable(self, state, tic):
        """ """
        return True

    def goal_reached(self, state, tic):
        """ """
        return False


class GoJointConfiguration:
    def __init__(
        self,
        qdes=None,
        max_v=None,
        motion_time=None,
        max_extra_time_rel=0.2,
        error_running=2 * 1e-2,
        error_goal=5 * 1e-3,
        min_motion_time=1.0,
        kp_scale=1.0,
        kv_scale=1.0,
        control_mode="torque",
    ):
        self.control_mode = control_mode
        self.max_v = max_v
        self.kp = (
            kp_scale
            * 2.0
            * np.array([3000.0, 3000.0, 800.0, 800.0, 200.0, 200.0, 200.0])
        )
        self.kv = kv_scale * 2.0 * np.array([80.0, 80.0, 40.0, 40.0, 8.0, 8.0, 8.0])
        self.qdes = qdes
        self.max_extra_time_rel = max_extra_time_rel
        self.error_running = error_running
        self.error_goal = error_goal
        self.min_motion_time = min_motion_time

        if max_v is None and motion_time is None:
            raise ValueError("define either max_v or motion_time")

    def setup(self, s):
        q0 = np.array(s.q)

        self.coefficients = fit_3rd_order_polynomial(
            q0, np.zeros(7), self.qdes, np.zeros(7)
        )

        if self.max_v is not None:
            max_vel, time_max_vel = polynomial_max_velocity(self.coefficients)

            rs = np.zeros(7)
            for i in range(7):
                r = max_vel[i] / self.max_v
                # print(
                #     "joint:",
                #     i,
                #     "max vel:",
                #     max_vel[i],
                #     "time:",
                #     time_max_vel[i],
                #     "r: ",
                #     r,
                # )
                rs[i] = r
            rmax = np.max(rs)
            # print("max ratio is: ", rmax)
            # print("to get a max vel of ", self.max_v, "we need to scale time by", rmax)
            self.motion_time = rmax

            if self.motion_time < self.min_motion_time:
                self.motion_time = self.min_motion_time

    def get_control(self, state, tic):
        t_i = min(tic / self.motion_time, 1.0)
        p, pdot, _ = evaluate_polynomial(self.coefficients, t_i)
        pdot /= self.motion_time

        if self.control_mode == "torque":
            cmd = FlexivCmd(q=p, dq=pdot, kp=self.kp, kv=self.kv)
        elif self.control_mode == "position":
            cmd = FlexivCmd(q=p, dq=pdot, mode=1)
        elif self.control_mode == "velocity":
            raise NotImplementedError
            # NOTE: this does not reach the goal and lags behind. 
            # print("pdot is", pdot)
            # cmd = FlexivCmd(dq=pdot, mode=1)
        return cmd

    def applicable(self, state, tic):
        """ """
        if tic > self.motion_time * (1 + self.max_extra_time_rel):
            print("too much time!")
            print("tic", tic, "max ", self.motion_time * (1 + self.max_extra_time_rel))
            return False

        q = np.array(state.q)
        t_i = min(tic / self.motion_time, 1.0)
        p, pq, _ = evaluate_polynomial(self.coefficients, t_i)
        if np.linalg.norm(p - q) > self.error_running:
            print("distance to goal is too large", np.linalg.norm(p - q))
            return False
        else:
            return True

    def goal_reached(self, state, tic):
        """ """
        q = np.array(state.q)
        qv = np.array(state.dq)
        return (
            np.linalg.norm(qv) < self.error_goal
            and np.linalg.norm(q - self.qdes) < self.error_goal
        )
    

class GoJointConfigurationVelocity:
    def __init__(self, qgoal, error_goal=0.01, max_v = .1,
                 kd = 1., smooth_velocity = .9):
        self.kd = kd
        self.qgoal = qgoal
        self.error_goal = error_goal
        self.max_v = max_v
        self.smooth_velocity = smooth_velocity

    def setup(self, s):
        pass

    def get_control(self, state, tic):
        """

        """
        v = self.kd * (self.qgoal - state.q) 
        if np.linalg.norm(v) > self.max_v:
            v /= np.linalg.norm(v)

        if self.smooth_velocity > 1e-6:
            v = self.smooth_velocity * np.array(state.dq) + (1 - self.smooth_velocity) * v

        return FlexivCmd(dq=v , mode=3)

    def applicable(self, state, tic):
        return True

    def goal_reached(self, state, tic):
        q = np.array(state.q)
        error = np.linalg.norm(q - self.qgoal)
        error_v = np.linalg.norm(state.dq)
        return error + error_v  < self.error_goal
    

class JointFloatingPython():
    def __init__(self, robot): 
        self.kv = 1./4. * np.array([80.0, 80.0, 40.0, 40.0, 8.0, 8.0, 8.0])
        self.robot = robot

    def setup(self,s):
        pass

    def get_control(self,state,tic):

        tau_g = self.robot.gravity(np.array(state.q))


        # get gravity compensation
        return  FlexivCmd(
                    tau_ff = tau_g,
                    kv=self.kv,
                    tau_ff_with_gravity=True,
        )
    
class Stay:
    def __init__(self):
        self.kp = 1 * np.array([3000.0, 3000.0, 800.0, 800.0, 200.0, 200.0, 200.0])
        self.kv = 3.0 * np.array([80.0, 80.0, 40.0, 40.0, 8.0, 8.0, 8.0])

    def setup(self, s):
        self.q = s.q

    def get_control(self, state, tic):
        return FlexivCmd(q=self.q, kp=self.kp, kv=self.kv)

    def applicable(self, state, tic):
        """ """
        return True

    def goal_reached(self, state, tic):
        """ """
        return False