import pygame
import numpy as np
import time
from pinocchio.visualize import MeshcatVisualizer

import FlexivPy.robot.sim.sim_robot as sim_robot
import FlexivPy.robot.robot_client as robot_client
import FlexivPy.robot.model.model_robot as model_robot
import numpy as np
import time
import yaml
import argparse
import pinocchio as pin
import easy_controllers

from FlexivPy.robot.dds.flexiv_messages import (
    FlexivCmd,
    FlexivState,
    EnvState,
    EnvImage,
)

from numpy.linalg import solve
from easy_controllers import frame_error, frame_error_jac


class RRT:
    def __init__(
        self,
        start,
        goal,
        goal_bias=0.2,
        collision_resolution=0.1,
        max_step=0.5,
        max_it=200,
        is_collision_free=lambda x: True,
        sample_fun=lambda: np.random.rand(7) * 2 * np.pi - np.pi,
        goal_tolerance=1e-3,
    ):

        self.collision_resolution = collision_resolution
        self.home = start
        self.goal = goal
        self.goal_bias = goal_bias
        self.max_step = max_step
        self.tree = []
        self.parents = []
        self.max_it = max_it
        self.is_collision_free = is_collision_free
        self.sample_fun = sample_fun
        self.goal_tolerance = goal_tolerance

    def get_nearest_neighbor_id(self, q):
        min_distance = np.inf
        min_index = -1
        for i, qtree in enumerate(self.tree):
            distance = np.linalg.norm(q - qtree)
            if distance < min_distance:
                min_distance = distance
                min_index = i
        return min_index

    def is_collision_free_edge(self, q1, q2):
        num_checks = int(np.linalg.norm(q1 - q2) / self.collision_resolution)
        for i in range(num_checks):
            qcheck = q1 + i * (q2 - q1) / num_checks
            if not self.is_collision_free(qcheck):
                return False
        return True

    def solve(self):

        solved = False
        goal_id = -1

        self.tree.append(self.home)
        self.parents.append(-1)

        if not self.is_collision_free(self.home):
            raise Exception("Initial configuration is in collision")

        if not self.is_collision_free(self.goal):
            raise Exception("Goal configuration is in collision")

        for _ in range(self.max_it):

            # sample a random configuration

            print("new it")
            if np.random.rand() < self.goal_bias:
                print("goal bias!")
                qrand = self.goal
            else:
                valid_sample = False
                while not valid_sample:
                    qrand = self.sample_fun()
                    valid_sample = self.is_collision_free(qrand)

                if not valid_sample:
                    raise Exception("Could not find a valid sample")

            # Get nearest neighbor
            nn_id = self.get_nearest_neighbor_id(qrand)
            qnear = self.tree[nn_id]

            # do a step
            if np.linalg.norm(qrand - qnear) > self.max_step:
                qnew = qnear + self.max_step * (qrand - qnear) / np.linalg.norm(
                    qrand - qnear
                )
            else:
                qnew = qrand

            # check if the edge is collisoin free

            if self.is_collision_free_edge(qnear, qnew):
                print("collision free")
                self.tree.append(qnew)
                self.parents.append(nn_id)

                if np.linalg.norm(qnew - self.goal) < self.goal_tolerance:
                    goal_id = len(self.tree) - 1
                    solved = True
                    break

        if solved:
            # trace back the solution
            id = goal_id
            path = []
            while id != -1:
                path.append(id)
                id = self.parents[id]

            # revese the path
            path = path[::-1]
            # return the corresponding configurations.
            return [self.tree[id] for id in path]

        else:
            raise Exception("Could not find a solution")


class GoRRT:
    def __init__(
        self, goal, pin_robot, approx_dt=0.01, goal_tolerance=5 * 1e-2, max_velocity=0.2
    ):
        self.goal = goal
        self.pin_robot = pin_robot
        self.goal_tolerance = goal_tolerance
        self.approx_dt = approx_dt
        self.max_velocity = max_velocity

    def setup(self, s):
        q = np.array(s.q)

        is_collision_free = lambda q: not pin.computeCollisions(
            self.pin_robot.model,
            self.pin_robot.data,
            self.pin_robot.collision_model,
            self.pin_robot.collision_data,
            q,
            True,
        )

        rrt = RRT(
            start=q,
            goal=self.goal,
            is_collision_free=is_collision_free,
            sample_fun=lambda: np.random.rand(7) * 2 * np.pi - np.pi,
        )

        self.path = rrt.solve()
        self.current_subgoal_id = 0
        print("path from rrt is")

        # visualize the collision path
        vizer = MeshcatVisualizer(
            self.pin_robot.model,
            self.pin_robot.collision_model,
            self.pin_robot.visual_model,
        )

        vizer.initViewer(loadModel=True)
        vizer.displayCollisions(True)
        vizer.displayVisuals(True)
        vizer.displayFrames(True)

        for q in self.path:
            vizer.display(q)
            input("Press Enter to continue...")

    def get_control(self, state, tic):

        # now we try to follow the path.

        # Check if I have reached the subgoal
        q = np.array(state.q)

        error = np.linalg.norm(q - self.path[self.current_subgoal_id])
        print("error is", error)
        if error < self.goal_tolerance:
            print("subgoal reached")
            self.current_subgoal_id += 1
            current_subgoal_id = min(self.current_subgoal_id, len(self.path) - 1)

        q_current_goal = self.path[self.current_subgoal_id]

        error = q_current_goal - q
        velocity = 2.0 * error

        if np.linalg.norm(velocity) > self.max_velocity:
            velocity = self.max_velocity * velocity / np.linalg.norm(velocity)

        desq_next = q + self.approx_dt * velocity

        print("q_curren_goal", q_current_goal)
        print("desq_next", desq_next)

        return FlexivCmd(
            q=desq_next,
            dq=velocity,
            kp=2.0 * np.array([3000.0, 3000.0, 800.0, 800.0, 200.0, 200.0, 200.0]),
            kv=1.5 * np.array([80.0, 80.0, 40.0, 40.0, 8.0, 8.0, 8.0]),
        )

    def applicable(self, s, tic):
        return True

    def goal_reached(self, s, tic):
        q = np.array(s.q)
        return np.linalg.norm(q - self.goal) < self.goal_tolerance


if __name__ == "__main__":

    robot = robot_client.Flexiv_client(render=False, create_sim_server=False)

    urdf = "/home/quim/code/FlexivPy/FlexivPy/assets/r10s_with_capsules.urdf"
    # flexiv_rizon10s_kinematics.urdf"
    meshes = "/home/quim/code/FlexivPy/FlexivPy/assets/meshes/"

    pin_robot = pin.RobotWrapper.BuildFromURDF(urdf, meshes)

    pin_robot.collision_model.addAllCollisionPairs()

    pin.removeCollisionPairs(
        pin_robot.model,
        pin_robot.collision_model,
        "/home/quim/code/FlexivPy/FlexivPy/assets/r10s_with_capsules.srdf",
    )

    pin_robot.rebuildData()

    dt_control = 0.01

    goal = np.array([0.000, -0.698, 0.000, 1.571, -0.000, 0.698, -0.000])

    controller = GoRRT(goal, pin_robot, approx_dt=dt_control)
    easy_controllers.run_controller(robot, controller, dt=dt_control, max_time=100)
