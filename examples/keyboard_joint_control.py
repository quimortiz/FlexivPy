import pygame
import numpy as np

import FlexivPy.robot.interface as interface
import numpy as np
import pinocchio as pin
import easy_controllers

from FlexivPy.robot.dds.flexiv_messages import (
    FlexivCmd,
)



class KeyboardJointControl:
    def __init__(self, pin_robot, approx_dt=0.01):
        self.robot = pin_robot
        self.approx_dt = approx_dt

        pygame.init()
        self.screen = pygame.display.set_mode(
            (100, 100)
        )  # Small window just to capture events

        # Control parameters
        self.joint_speed = 0.05
        self.q_last_key_press = None
        self.des_q = None

    def setup(self, s):
        self.q_last_key_press = np.array(s.q)
        self.tic_last_press = 0.0
        self.des_q = np.array(s.q)

    def get_control(self, state, tic):
        q = np.array(state.q)
        dq = np.zeros_like(q)  # Joint velocity changes

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Get all key states
        keys = pygame.key.get_pressed()

        if any(keys) and tic - self.tic_last_press > 0.1:
            self.tic_last_press = tic
            self.q_last_key_press = q

            if keys[pygame.K_q]:
                raise Exception("Quitting")

            # Control each joint with specific keys (s,d,f,g,h,j,k) to go down and (w,e,r,t,y,u,i) to go up
            if keys[pygame.K_s]:
                dq[0] -= self.joint_speed  # Joint 1 down
            if keys[pygame.K_w]:
                dq[0] += self.joint_speed  # Joint 1 up
            if keys[pygame.K_d]:
                dq[1] -= self.joint_speed  # Joint 2 down
            if keys[pygame.K_e]:
                dq[1] += self.joint_speed  # Joint 2 up
            if keys[pygame.K_f]:
                dq[2] -= self.joint_speed  # Joint 3 down
            if keys[pygame.K_r]:
                dq[2] += self.joint_speed  # Joint 3 up
            if keys[pygame.K_g]:
                dq[3] -= self.joint_speed  # Joint 4 down
            if keys[pygame.K_t]:
                dq[3] += self.joint_speed  # Joint 4 up
            if keys[pygame.K_h]:
                dq[4] -= self.joint_speed  # Joint 5 down
            if keys[pygame.K_y]:
                dq[4] += self.joint_speed  # Joint 5 up
            if keys[pygame.K_j]:
                dq[5] -= self.joint_speed  # Joint 6 down
            if keys[pygame.K_u]:
                dq[5] += self.joint_speed  # Joint 6 up
            if keys[pygame.K_k]:
                dq[6] -= self.joint_speed  # Joint 7 down
            if keys[pygame.K_i]:
                dq[6] += self.joint_speed  # Joint 7 up

        # Limit the velocity to max_dq
        self.des_q += dq

        error = self.des_q - q
        print("Error: ", error)
        velocity = 1.0 * (self.des_q - q)

        des_q_next = q + self.approx_dt * velocity

        return FlexivCmd(
            q=des_q_next,
            dq=velocity,
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

    robot = interface.Flexiv_client(render=False, create_sim_server=False)

    urdf = "/home/quim/code/FlexivPy/FlexivPy/assets/flexiv_rizon10s_kinematics.urdf"
    meshes = "/home/quim/code/FlexivPy/FlexivPy/assets/meshes/"

    pin_robot = pin.RobotWrapper.BuildFromURDF(urdf, meshes)
    dt_control = 0.01

    controller = KeyboardJointControl(pin_robot, approx_dt=dt_control)
    easy_controllers.run_controller(robot, controller, dt=dt_control, max_time=100)
