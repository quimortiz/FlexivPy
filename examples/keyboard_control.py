import pygame
import numpy as np
import time


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

from scipy.optimize import minimize


class KeyboardEndEff:
    def __init__(self, pin_robot, frame_id, approx_dt=0.01, dq_scale=1.0, max_dq=0.2):

        self.robot = pin_robot
        self.frame_id = frame_id
        self.approx_dt = approx_dt
        self.dq_scale = dq_scale
        self.max_dq = max_dq

        pygame.init()
        self.screen = pygame.display.set_mode(
            (100, 100)
        )  # Small window just to capture events

        self.rotation = np.array([0.0, 0.0, 0.0])  # [roll, pitch, yaw] orientation

        # Control speed
        self.position_speed = 0.005
        self.rotation_speed = 0.05
        self.q_last_key_press = None

    def setup(self, s):
        """ """
        self.q_last_key_press = s.q
        self.robot.framePlacement(np.array(s.q), self.frame_id, update_kinematics=True)
        self.target_position = self.robot.data.oMf[self.frame_id].translation
        self.target_rotation = self.robot.data.oMf[self.frame_id].rotation
        self.tic_last_press = 0.0

    def get_control(self, state, tic):

        q = np.array(state.q)
        self.robot.framePlacement(q, self.frame_id, update_kinematics=True)
        # self.current_position = self.robot.data.oMf[self.frame_id].translation
        # self.current_rotation = self.robot.data.oMf[self.frame_id].rotation

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Get all key states
        keys = pygame.key.get_pressed()
        dp = np.zeros(3)
        drotation = np.zeros(3)

        # Check if any key is pressed
        print("tic")
        print(tic)
        print(self.tic_last_press)
        if any(keys) and tic - self.tic_last_press > 0.1:
            self.tic_last_press = tic
            self.q_last_key_press = state.q
            print("A key is pressed!")
            # Position control (asd, re)

            if keys[pygame.K_q]:
                raise Exception("Quitting")

            if keys[pygame.K_a]:
                dp[0] -= self.position_speed  # Decrease X
            if keys[pygame.K_d]:
                dp[0] += self.position_speed  # Increase X
            if keys[pygame.K_s]:
                dp[1] -= self.position_speed  # Decrease Y
            if keys[pygame.K_w]:
                dp[1] += self.position_speed  # Increase Y
            if keys[pygame.K_e]:
                dp[2] += self.position_speed  # Increase Z
            if keys[pygame.K_r]:
                dp[2] -= self.position_speed  # Decrease Z

            # Rotation control (u,j,i,k,o,l)
            if keys[pygame.K_u]:
                drotation[0] -= self.rotation_speed  # Decrease Roll
            if keys[pygame.K_j]:
                drotation[0] += self.rotation_speed  # Increase Roll
            if keys[pygame.K_i]:
                drotation[1] -= self.rotation_speed  # Decrease Pitch
            if keys[pygame.K_k]:
                drotation[1] += self.rotation_speed  # Increase Pitch
            if keys[pygame.K_o]:
                drotation[2] -= self.rotation_speed  # Decrease Yaw
            if keys[pygame.K_l]:
                drotation[2] += self.rotation_speed  # Increase Yaw

        local_rotation = pin.rpy.rpyToMatrix(drotation)
        self.target_rotation = self.target_rotation @ local_rotation
        self.target_position = self.target_position + dp
        print("new target pos", self.target_position)

        oMdes = pin.SE3(self.target_rotation, self.target_position)
        damp = 1e-3
        fMd = self.robot.data.oMf[self.frame_id].actInv(oMdes)
        J = pin.computeFrameJacobian(
            self.robot.model, self.robot.data, q, self.frame_id
        )  # in joint frame
        J = -np.dot(pin.Jlog6(fMd.inverse()), J)
        err = pin.log(fMd).vector  # in frame frame
        v = -J.T.dot(solve(J.dot(J.T) + damp * np.eye(6), err))

        v *= self.dq_scale

        if np.linalg.norm(v) > self.max_dq:
            v /= np.linalg.norm(v)
            v *= self.max_dq

        print("error", err)
        print("desired p", self.target_position)

        des_q = q + self.approx_dt * v
        print("Desired q: ", des_q)
        print("Desired v: ", v)

        return FlexivCmd(
            q=des_q,
            dq=v,
            kp=1.0 * np.array([3000.0, 3000.0, 800.0, 800.0, 200.0, 200.0, 200.0]),
            kv=1.0 * np.array([80.0, 80.0, 40.0, 40.0, 8.0, 8.0, 8.0]),
        )

    def applicable(self, state, tic):
        return True

    def goal_reached(self, state, tic):
        return False

    def close(self):
        pygame.quit()


if __name__ == "__main__":

    robot = robot_client.Flexiv_client(render=False, create_sim_server=False)

    urdf = "/home/quim/code/FlexivPy/FlexivPy/assets/flexiv_rizon10s_kinematics.urdf"
    meshes = "/home/quim/code/FlexivPy/FlexivPy/assets/meshes/"

    pin_robot = pin.RobotWrapper.BuildFromURDF(urdf, meshes)
    frame_id = pin_robot.model.getFrameId("flange")
    dt_control = 0.01

    controller = KeyboardEndEff(pin_robot, frame_id, approx_dt=dt_control)
    easy_controllers.run_controller(robot, controller, dt=dt_control, max_time=100)
