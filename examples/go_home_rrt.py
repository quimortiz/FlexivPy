import numpy as np
from pinocchio.visualize import MeshcatVisualizer

import FlexivPy.robot.interface as interface
from FlexivPy.robot.sim.sim_robot import FlexivSim
import numpy as np
import pinocchio as pin
import easy_controllers
import os
from FlexivPy import ASSETS_PATH

from FlexivPy.robot.dds.flexiv_messages import (
    FlexivCmd,
)
from FlexivPy.robot.planners.rrt import RRT
import time 


class RRTPathFollowerController:
    def __init__(
        self, 
        home_state=None, 
        pin_robot=None, 
        approx_dt=0.01, 
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
        self.approx_dt = approx_dt
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

        start = s.q
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

        desq_next = q + self.approx_dt * velocity


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



if __name__ == "__main__":
    # robot = robot_client.Flexiv_client(render=False, create_sim_server=False)
    robot = FlexivSim(render=True)
    dt_control = 0.01
    controller = RRTPathFollowerController(approx_dt=dt_control)
    goal = np.array([0.5, -0.698, 0.000, 1.571, -0.000, 0.698, -0.000])
    start = robot.get_robot_state().q
    controller.compute_path(start, goal)
    easy_controllers.run_controller(robot, controller, dt=dt_control, max_time=100, sync_sim=True, dt_sim=0.001)
    robot.close()
