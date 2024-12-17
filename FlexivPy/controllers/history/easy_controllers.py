import numpy as np
import pinocchio as pin
from numpy.linalg import solve
import time
from scipy.optimize import minimize
from FlexivPy.robot.dds.flexiv_messages import (
    FlexivCmd,
)
import numpy as np
from FlexivPy.controllers.utils import *


class EndEffPose2:
    def __init__(self, robot, s, oMdes, frame_id, tic):
        self.robot = robot
        self.oMdes = oMdes
        self.tic = tic
        self.frame_id = frame_id
        self.damp = 1e-6
        self.kv = 80.0
        self.kv_damping = 5.0
        self.max_error = 0.2  # gains are very agressive!
        self.q0 = s["q"].copy()

        error = lambda q: frame_error(q, self.robot, self.frame_id, self.oMdes)
        jac_error = lambda q: frame_error_jac(q, self.robot, self.frame_id, self.oMdes)

        min_res = minimize(error, jac=jac_error, x0=s["q"], method="BFGS")
        self.q_des = min_res.x

        assert error(self.q_des) < 1e-6

        self.max_displacement = 0.005

        self.coefficients = fit_3rd_order_polynomial(
            self.q0, np.zeros(7), self.q_des, np.zeros(7)
        )

        fixed_time_scaling = False

        if fixed_time_scaling:
            self.motion_time = 3  # 3 seconds

        else:
            self.max_vel = 0.5  # .5 rad / s
            # print("we use max vel to compute the time scaling")

            max_vel, time_max_vel = polynomial_max_velocity(self.coefficients)

            rs = np.zeros(7)
            for i in range(7):
                r = max_vel[i] / self.max_vel
                print(
                    "joint:",
                    i,
                    "max vel:",
                    max_vel[i],
                    "time:",
                    time_max_vel[i],
                    "r: ",
                    r,
                )
                rs[i] = r
            rmax = np.max(rs)
            print("max ratio is: ", rmax)
            print(
                "to get a max vel of ", self.max_vel, "we need to scale time by", rmax
            )
            self.motion_time = rmax

    def is_applicable(self, state):
        return True

    def error(self, s):
        q = s["q"]
        data = self.robot.data
        self.robot.framePlacement(q, self.frame_id, update_kinematics=True)
        iMd = data.oMf[self.frame_id].actInv(self.oMdes)
        err = pin.log(iMd).vector  # in joint frame
        return np.linalg.norm(err)

    def get_control(self, state, tic):
        if (tic - self.tic) > self.motion_time:
            return None
            # assert False

        use_polynomials = True
        if not use_polynomials:
            q = state["q"]
            distance = np.linalg.norm(self.q_des - q)
            print("current distance is", distance)
            # if distance > .2:

            if distance > self.max_displacement:
                q_des = q + self.max_displacement * (self.q_des - q) / distance
            else:
                q_des = self.q_des
            assert np.linalg.norm(q_des - q) < self.max_displacement + 1e-8

            desired_v = np.zeros(7)

            return {
                "tau_ff": np.zeros(7),
                "q": q_des,
                "dq": desired_v,
                "kp": 3 * np.array([3000.0, 3000.0, 800.0, 800.0, 200.0, 200.0, 200.0]),
                "kv": 4 * np.array([80.0, 80.0, 40.0, 40.0, 8.0, 8.0, 8.0]),
                "mode": 2,
            }

        else:
            q = state["q"]
            distance = np.linalg.norm(self.q_des - q)
            print("current distance is", distance)

            distance = np.linalg.norm(self.q_des - q)

            t_i = (tic - self.tic) / self.motion_time
            p, pdot, _ = evaluate_polynomial(self.coefficients, t_i)
            pdot /= self.motion_time

            return {
                "tau_ff": np.zeros(7),
                "q": p,
                "dq": pdot,
                "kp": 0.4
                * np.array([3000.0, 3000.0, 800.0, 800.0, 200.0, 200.0, 200.0]),
                "kv": 0.8 * np.array([80.0, 80.0, 40.0, 40.0, 8.0, 8.0, 8.0]),
                "mode": 2,
            }


class TaskSpaceImpedance:
    def __init__(self, robot, s):

        frame_id = robot.model.getFrameId("flange")
        print("frame id is ", frame_id)
        T = robot.framePlacement(s["q"], frame_id, update_kinematics=True)
        print("T is ", T)
        p = T.translation
        self.desired_pos = p
        print("desired_pos:", self.desired_pos)
        self.kp_task_space = np.array([4000, 10, 4000])
        self.kv_task_space = np.array([400, 10, 400])
        self.robot = robot
        self.frame_id = frame_id
        self.kImpedanceKd = np.array([80.0, 80.0, 40.0, 40.0, 8.0, 8.0, 8.0])
        self.kv_joint = 0.1 * self.kImpedanceKd
        self.kangular_vel = 10

    def get_control(self, state, robot_model):

        q = state["q"]
        dq = state["dq"]
        F_control = np.zeros(6)

        # get the end effector postion

        p = self.robot.framePlacement(
            q, self.frame_id, update_kinematics=True
        ).translation

        print("p:", p)

        v = self.robot.frameVelocity(
            q,
            dq,
            self.frame_id,
            update_kinematics=True,
            reference_frame=pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )

        F_control[:3] = (
            self.kp_task_space * (self.desired_pos - p) - self.kv_task_space * v.linear
        )

        F_control[3:] += -self.kangular_vel * v.angular

        # print('F_control:', F_control)

        # Compute the Jacobian in the world frame
        J = pin.computeFrameJacobian(
            self.robot.model,
            self.robot.data,
            np.array(q),
            self.frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )

        # Compute the joint torques (tau) using the transpose of the Jacobian
        tau = J.T @ F_control

        # print('tau:', tau)

        # tau += np.random.normal(0, 0.2, 7)
        # tau[4] += 1.

        cmd = {
            "tau_ff": tau,
            "q": state["q"],
            "dq": np.zeros(7),
            "kp": np.zeros(7),
            "kv": self.kv_joint,
            "mode": 2,
        }

        return cmd


class ForceController:
    # continue here!!
    # check notes on manipulation of russ tedrake.
    def __init__(self, robot, frame_id, desired_f, desired_pos, desired_R):
        self.robot = robot
        self.frame_id = frame_id
        self.desired_R = desired_R
        self.desired_pos = desired_pos
        self.kforce = 0.0001
        self.max_position_error = 0.001
        self.desired_f = desired_f

    def get_control(self, state, tic):
        current_f = state["ft_sensor"][2]
        q = state["q"]

        data = self.robot.data
        self.robot.framePlacement(q, self.frame_id, update_kinematics=True)

        current_pos = data.oMf[self.frame_id].translation
        force_error = self.desired_f - current_f

        print("current force:", current_f)
        print("current position:", current_pos)

        approach = "pos_error_and_solve_ik"

        if approach == "pos_error_and_solve_ik":

            position_error_z = self.kforce * force_error

            if abs(position_error_z) > self.max_position_error:
                position_error_z = np.sign(position_error_z) * self.max_position_error

            # force small ->  force error is positive -> desired_pos is lower
            desired_pos = current_pos + np.array([0, 0, position_error_z])
            print("current_pos:", current_pos)
            print("desired_pos:", desired_pos)
            desired_pos[:2] = self.desired_pos[:2].copy()

            oMdes = pin.SE3(self.desired_R, desired_pos)

            error = lambda q: frame_error(q, self.robot, self.frame_id, oMdes)
            jac_error = lambda q: frame_error_jac(q, self.robot, self.frame_id, oMdes)

            min_res = minimize(error, jac=jac_error, x0=q, method="BFGS")
            q_des = min_res.x

            assert error(q_des) < 1e-6

            cmd = {
                "tau_ff": np.zeros(7),
                "q": q_des,
                "dq": np.zeros(7),
                "kp": 2.0
                * np.array([3000.0, 3000.0, 800.0, 800.0, 200.0, 200.0, 200.0]),
                "kv": 4.0 * np.array([80.0, 80.0, 40.0, 40.0, 8.0, 8.0, 8.0]),
                "mode": 2,
            }
        else:
            raise NotImplementedError
        # return cmd
        return cmd


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
        self.motion = time
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
        print("current position is", q)
        print("current velocity is", state.dq)
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


class GoJointConfigurationSlow:
    def __init__(
        self, qgoal, error_goal=0.01, max_dq=0.005, max_v=0.1, control_mode="torque"
    ):
        self.qgoal = qgoal
        self.error_goal = error_goal
        self.max_dq = max_dq
        self.kp = 1.5 * np.array([3000.0, 3000.0, 800.0, 800.0, 200.0, 200.0, 200.0])
        self.kv = 4.0 * np.array([80.0, 80.0, 40.0, 40.0, 8.0, 8.0, 8.0])
        self.control_mode = control_mode
        self.max_v = max_v

    def setup(self, s):
        pass

    def get_control(self, state, tic):

        if self.control_mode == "torque":
            q = np.array(state.q)
            if np.linalg.norm(q - self.qgoal) > self.error_goal:
                qgoal = q + self.max_dq * (self.qgoal - q) / np.linalg.norm(
                    q - self.qgoal
                )
            else:
                qgoal = self.qgoal

            return FlexivCmd(q=qgoal, kp=self.kp, kv=self.kv)
        elif self.control_mode == "position":
            raise NotImplementedError
        elif self.control_mode == "velocity":

            v = (
                self.max_v
                * (self.qgoal - state.q)
                / max(np.linalg.norm(self.qgoal - state.q), 1e-6)
            )

            return FlexivCmd(dq=v, mode=1)

    def applicable(self, state, tic):
        return True

    def goal_reached(self, state, tic):
        q = np.array(state.q)
        error = np.linalg.norm(q - self.qgoal)
        return error < self.error_goal


class GoJointConfigurationVelocity:
    def __init__(self, qgoal, error_goal=0.01, max_v=0.1, kd=1.0, smooth_velocity=0.9):
        self.kd = kd
        self.qgoal = qgoal
        self.error_goal = error_goal
        self.max_v = max_v
        self.smooth_velocity = smooth_velocity

    def setup(self, s):
        pass

    def get_control(self, state, tic):
        """ """
        v = self.kd * (self.qgoal - state.q)
        if np.linalg.norm(v) > self.max_v:
            v /= np.linalg.norm(v)

        if self.smooth_velocity > 1e-6:
            v = (
                self.smooth_velocity * np.array(state.dq)
                + (1 - self.smooth_velocity) * v
            )

        return FlexivCmd(dq=v, mode=3)

    def applicable(self, state, tic):
        return True

    def goal_reached(self, state, tic):
        q = np.array(state.q)
        error = np.linalg.norm(q - self.qgoal)
        error_v = np.linalg.norm(state.dq)
        return error + error_v < self.error_goal


class GoEndEffectorPose:
    def __init__(self, robot, oMdes, frame_id, error, max_v=0.3, motion_time=None):
        self.robot = robot
        self.oMdes = oMdes
        self.frame_id = frame_id
        self.max_v = max_v
        self.motion_time = motion_time
        self.joint_controller = None

    def setup(self, s):

        self.error = lambda q: frame_error(q, self.robot, self.frame_id, self.oMdes)
        self.jac_error = lambda q: frame_error_jac(
            q, self.robot, self.frame_id, self.oMdes
        )
        min_res = minimize(
            self.error, jac=self.jac_error, x0=np.array(s.q), method="BFGS"
        )
        self.q_des = min_res.x
        assert self.error(self.q_des) < 1e-6

        self.joint_controller = GoJointConfiguration(qdes=self.q_des, max_v=self.max_v)
        self.joint_controller.setup(s)

    def get_control(self, state, tic):
        return self.joint_controller.get_control(state, tic)

    def applicable(self, state, tic):
        return self.joint_controller.applicable(state, tic)

    def goal_reached(self, state, tic):
        # alternative: check the end effector position
        return self.joint_controller.goal_reached(state, tic)


class InverseKinematicsController:
    def __init__(
        self,
        robot,
        oMdes,
        frame_id,
        approx_dt=0.01,
        kvv=2.0,
        kp_scale=1.0,
        kv_scale=1.0,
        goal_error=1e-2,
        max_v=0.3,
        w_weight=0.25,
    ):
        self.approx_dt = approx_dt
        self.goal_error = goal_error
        self.robot = robot
        self.oMdes = oMdes
        self.frame_id = frame_id
        self.max_v = max_v
        self.w_weight = w_weight
        self.joint_controller = None
        self.kp = kp_scale * np.array(
            [3000.0, 3000.0, 800.0, 800.0, 400.0, 400.0, 400.0]
        )
        self.kv = kv_scale * np.array([80.0, 80.0, 40.0, 40.0, 20.0, 20.0, 20.0])
        self.kvv = kvv

    def setup(self, s):
        pass

    def get_control(self, state, tic):

        q = np.array(state.q)
        self.robot.framePlacement(q, self.frame_id, update_kinematics=True)
        iMd = self.robot.data.oMf[self.frame_id].actInv(self.oMdes)
        err = pin.log(iMd).vector  # in joint frame

        J = pin.computeFrameJacobian(
            self.robot.model, self.robot.data, q, self.frame_id
        )  # in joint frame
        J = -np.dot(pin.Jlog6(iMd.inverse()), J)
        damp = 1e-6
        v = self.kvv * -J.T.dot(solve(J.dot(J.T) + damp * np.eye(6), err))

        if np.linalg.norm(v) > self.max_v:
            v /= np.linalg.norm(v)
            v *= self.max_v

        q_des = q + self.approx_dt * v
        cmd = FlexivCmd(q=q_des, dq=v, kp=self.kp, kv=self.kv)
        return cmd

    def applicable(self, state, tic):
        return True

    def goal_reached(self, state, tic):
        q = np.array(state.q)
        self.robot.framePlacement(q, self.frame_id, update_kinematics=True)
        iMd = self.robot.data.oMf[self.frame_id].actInv(self.oMdes)
        err = pin.log(iMd).vector  # in joint frame
        err *= np.array([1.0, 1.0, 1.0, self.w_weight, self.w_weight, self.w_weight])
        print("error is", err)
        return np.linalg.norm(err) < self.goal_error


class JointFloatingHistory:
    # TODO: not working yet, maybe remove?
    def __init__(self, history=100, kv_scale=2.0, kp_scale=0.2):
        self.kv_scale = kv_scale
        self.kv = kv_scale * np.array([80.0, 80.0, 40.0, 40.0, 8.0, 8.0, 8.0])
        self.kp = kp_scale * np.array(
            [3000.0, 3000.0, 800.0, 800.0, 200.0, 200.0, 200.0]
        )
        self.history_size = history
        self.q_history = []

    def setup(self, s):
        self.q_history.clear()
        for _ in range(self.history_size):
            self.q_history.append(np.array(s.q))

    def get_control(self, state, tic):
        if len(self.q_history) >= self.history_size:
            self.q_history.pop(0)  # Remove the oldest item
        self.q_history.append(np.array(state.q))

        # Take the average of the buffer
        self.average_q = np.mean(self.q_history, axis=0)
        print("average q:", self.average_q)

        return FlexivCmd(q=self.average_q, kp=self.kp, kv=self.kv)

    def applicable(self, state, tic):
        """ """
        return True

    def goal_reached(self, state, tic):
        """ """
        return False


class JointFloating:
    def __init__(self, kv_scale=1.0):
        self.kv_scale = kv_scale
        self.kv = kv_scale * np.array([80.0, 80.0, 40.0, 40.0, 8.0, 8.0, 8.0])

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
