import numpy as np
import pinocchio as pin


class Controller_static_q0:
    def __init__(self, robot_model, des_q):
        self.robot_model = robot_model
        self.des_q = des_q

    def get_control(self, state, robot_model):
        cmd = {
            "tau_ff": np.zeros(7),
            "q": self.des_q,
            "dq": np.zeros(7),
            "kp": 10.0 * np.ones(7),
            "kv": 5.0 * np.ones(7),
        }
        return cmd


class Controller_joint_PD:
    def __init__(self, robot_model):
        self.robot_model = robot_model
        self.q1 = np.array([0.0, -0.75, 0.0, 1.5, 0.0, 0.75, 0.0])
        self.q2 = np.array([0.5, -0.5, 0.2, 1.0, 0.2, 0.75, 0.5])
        self.current_goal = self.q1
        self.goal_tolerance = 0.1
        self.kp = 5.0
        self.kv = 2.0
        self.send_torque_directly = False
        self.pd_on_acceleration = False

    def get_control(self, state, robot_model):
        """ """
        print('distnace to goal:', np.linalg.norm(self.current_goal - state["q"]))
        print(self.current_goal - state["q"])

        if (
            np.linalg.norm(self.current_goal - self.q1) < 1e-6
            and np.linalg.norm(state["q"] - self.q1) < self.goal_tolerance
        ):
            print("q1 reached -- switching to q2")
            self.current_goal = self.q2

        elif (
            np.linalg.norm(self.current_goal - self.q2) < 1e-6
            and np.linalg.norm(state["q"] - self.q2) < self.goal_tolerance
        ):
            print("q2 reached -- switching to q1")
            self.current_goal = self.q1

        if self.send_torque_directly:
            u = self.kp * (self.current_goal - state["q"]) - self.kv * state["dq"]
            cmd = {
                "tau_ff": u,
                "q": np.zeros(7),
                "dq": np.zeros(7),
                "kp": np.ones(7),
                "kv": np.ones(7),
            }
        elif self.pd_on_acceleration:
            des_q_dotdot = (
                self.kp * (self.current_goal - state["q"]) - self.kv * state["dq"]
            )

            # compute desired torque using the robot model

            # Compute the joint-space inertia matrix

            # Compute the Coriolis and centrifugal effects
            tau = pin.rnea(
                robot_model.model,
                robot_model.data,
                state["q"],
                state["dq"],
                des_q_dotdot,
            )
            g = pin.computeGeneralizedGravity(
                robot_model.model, robot_model.data, state["q"]
            )

            tau_ff = tau - g
            cmd = {
                "tau_ff": tau_ff,
                "q": np.zeros(7),
                "dq": np.zeros(7),
                "kp": np.zeros(7),
                "kv": np.zeros(7),
            }

        else:
            cmd = {
                "tau_ff": np.zeros(7),
                "q": self.current_goal,
                "dq": np.zeros(7),
                "kp": self.kp * np.ones(7),
                "kv": self.kv * np.ones(7),
            }

        return cmd

class Controller_joint_example:

    def __init__(self, robot_model, q0):
        """
        """
        self.robot_model = robot_model
        self.kLoopPeriod = 0.001;
        self.kSineAmp = 0.2;
        self.kSineFreq = 0.2;
        self.q0 = q0
        self.loop_counter = 0
        self.kp = 100

    def get_control(self,state,robot_model):
        target_pos = self.q0 + min(1.0 , np.exp(0.0001 * self.loop_counter) - 1. ) * self.kSineAmp * np.sin(2 * np.pi * self.kSineFreq * self.loop_counter * self.kLoopPeriod)
        self.loop_counter += 1

        cmd = {
            "tau_ff": np.zeros(7),
            "q": target_pos,
            "dq": np.zeros(7),
            "kp": self.kp *  np.ones(7),
            "kv":  np.ones(7),
            "mode": 1,
        }

        return cmd
            
