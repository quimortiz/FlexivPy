import numpy as np
import pinocchio as pin

from numpy.linalg import solve
import time
from scipy.optimize import minimize
from FlexivPy.robot.dds.flexiv_messages import (
    FlexivCmd,
)

from examples.utils import *

# TODO: small demo of goal reaching tasks, opening and closing the gripper!
# TODO: continue here!!


from enum import Enum


class ControllerStatus(Enum):
    UNKNOWN = 0
    GOAL_REACHED = 1
    NOT_APPLICABLE = 2
    MAX_TIME = 3
    ERROR = 4


def run_controller(
    robot, controller, dt, max_time, sync_sim=False, dt_sim=None, callback=None
):
    s = robot.get_robot_state()
    controller.setup(s)
    tic_start = time.time()
    exit_status = ControllerStatus.UNKNOWN
    counter = 0
    while True:
        tic = time.time()
        s = robot.get_robot_state()

        elapsed_time = tic - tic_start if not sync_sim else counter * dt

        if not controller.applicable(s, elapsed_time):
            print("controller is not applicable")
            exit_status = ControllerStatus.NOT_APPLICABLE
            break

        if controller.goal_reached(s, elapsed_time):
            exit_status = ControllerStatus.GOAL_REACHED
            print("goal reached")
            break

        if elapsed_time > max_time:
            exit_status = ControllerStatus.MAX_TIME
            print("max time reached")
            break

        cmd = controller.get_control(s, elapsed_time)

        robot.set_cmd(cmd)

        if callback is not None:
            callback(robot, cmd, elapsed_time)

        if sync_sim:
            if dt_sim is None:
                raise ValueError("error!")
            else:
                num_steps = int(dt // dt_sim)
                for _ in range(num_steps):
                    robot.step()

        time.sleep(max(0, dt - (time.time() - tic)))
        # We also apply this in simulation because we don't want to go faster than realtime.
        counter += 1
    # except:
    #     exit_status = ControllerStatus.ERROR
    return exit_status












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
    ):
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

        cmd = FlexivCmd(q=p, dq=pdot, kp=self.kp, kv=self.kv)
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


class GoJointConfigurationSlow:
    def __init__(self, qgoal, error_goal=0.01, max_dq=0.005):
        self.qgoal = qgoal
        self.error_goal = error_goal
        self.max_dq = max_dq
        self.kp = 1.5 * np.array([3000.0, 3000.0, 800.0, 800.0, 200.0, 200.0, 200.0])
        self.kv = 4.0 * np.array([80.0, 80.0, 40.0, 40.0, 8.0, 8.0, 8.0])

    def setup(self, s):
        pass

    def get_control(self, state, tic):
        q = np.array(state.q)
        if np.linalg.norm(q - self.qgoal) > self.error_goal:
            qgoal = q + self.max_dq * (self.qgoal - q) / np.linalg.norm(q - self.qgoal)
        else:
            qgoal = self.qgoal

        return FlexivCmd(q=qgoal, kp=self.kp, kv=self.kv)

    def applicable(self, state, tic):
        return True

    def goal_reached(self, state, tic):
        q = np.array(state.q)
        error = np.linalg.norm(q - self.qgoal)
        return error < self.error_goal


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
        min_res = minimize(self.error, jac=self.jac_error, x0=np.array(s.q), method="BFGS")
        self.q_des = min_res.x
        assert self.error(self.q_des) < 1e-6

        self.joint_controller = GoJointConfiguration(
            qdes=self.q_des, max_v=self.max_v
        )
        self.joint_controller.setup(s)

    def get_control(self, state, tic):
        return self.joint_controller.get_control(state, tic)

    def applicable(self, state, tic):
        return self.joint_controller.applicable(state, tic)

    def goal_reached(self, state, tic):
        # alternative: check the end effector position
        return self.joint_controller.goal_reached(state, tic)


class InverseKinematicsController:
    def __init__(self, robot, oMdes, frame_id, 
                 approx_dt=0.01,kvv=2.,
                 kp_scale=1.0, kv_scale=1.0,
                 goal_error=1e-2, max_v=0.3,
                 w_weight=.25) :
        self.approx_dt = approx_dt
        self.goal_error = goal_error
        self.robot = robot
        self.oMdes = oMdes
        self.frame_id = frame_id
        self.max_v = max_v
        self.w_weight = w_weight
        self.joint_controller = None
        self.kp = kp_scale * np.array([3000.0, 3000.0, 800.0, 800.0, 400.0, 400.0, 400.0])
        self.kv = kv_scale * np.array([80.0, 80.0, 40.0, 40.0, 20.0, 20.0, 20.0])
        self.kvv = kvv

    def setup(self, s):
        pass


    def get_control(self, state, tic):

        q = np.array(state.q)
        self.robot.framePlacement(q, self.frame_id, update_kinematics=True)
        iMd = self.robot.data.oMf[self.frame_id].actInv(self.oMdes)
        err = pin.log(iMd).vector  # in joint frame

        J = pin.computeFrameJacobian(self.robot.model, self.robot.data, q, self.frame_id)  # in joint frame
        J = -np.dot(pin.Jlog6(iMd.inverse()), J)
        damp = 1e-6
        v = self.kvv *  -J.T.dot(solve(J.dot(J.T) + damp * np.eye(6), err))


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
        err *= np.array([1., 1., 1., self.w_weight, self.w_weight, self.w_weight])
        print("error is", err)
        return np.linalg.norm(err) < self.goal_error




class JointFloating:
    def __init__(self, kv_scale=1.):
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


class OpenGripper:
    def __init__(self, stay_here=True, max_error_stay_here=1e-1):
        self.stay_here = stay_here
        self.kp = 1.0 * np.array([3000.0, 3000.0, 800.0, 800.0, 200.0, 200.0, 200.0])
        self.kv = 1.0 * np.array([80.0, 80.0, 40.0, 40.0, 8.0, 8.0, 8.0])
        self.max_error_stay_here = max_error_stay_here

    def setup(self, s):
        self.q = np.array(s.q)

    def get_control(self, state, tic):
        if not self.stay_here:
            return FlexivCmd(g_cmd="open")
        else:
            return FlexivCmd(g_cmd="open", q=self.q, kp=self.kp, kv=self.kv)

    def applicable(self, state, tic):
        """ """
        if self.stay_here:
            if np.linalg.norm(self.q - state.q) > self.max_error_stay_here:
                return False
        return True

    def goal_reached(self, state, tic):
        """ """
        if state.g_state == "open":
            return True
        else:
            return False


class CloseGripper:
    def __init__(self, stay_here=True, max_error_stay_here=1e-1):
        self.stay_here = stay_here
        self.kp = 1.0 * np.array([3000.0, 3000.0, 800.0, 800.0, 200.0, 200.0, 200.0])
        self.kv = 1.0 * np.array([80.0, 80.0, 40.0, 40.0, 8.0, 8.0, 8.0])
        self.max_error_stay_here = max_error_stay_here

    def setup(self, s):
        self.q = np.array(s.q)

    def get_control(self, state, tic):
        if not self.stay_here:
            return FlexivCmd(g_cmd="close")
        else:
            return FlexivCmd(q=self.q, g_cmd="close", kp=self.kp, kv=self.kv)

    def applicable(self, state, tic):
        """ """
        if self.stay_here:
            if np.linalg.norm(self.q - state.q) > self.max_error_stay_here:
                return False
        return True

    def goal_reached(self, state, tic):
        """ """

        if state.g_state == "closed" or state.g_state == "holding":
            return True
        else:
            return False


class StopGripper:
    def __init__(self, stay_here=True):
        self.stay_here = stay_here
        self.kp = 1.0 * np.array([3000.0, 3000.0, 800.0, 800.0, 200.0, 200.0, 200.0])
        self.kv = 1.0 * np.array([80.0, 80.0, 40.0, 40.0, 8.0, 8.0, 8.0])

    def setup(self, s):
        self.q = np.array(s.q) 

    def get_control(self, state, tic):
        # TODO: check that this is running in the robot.
        if not self.stay_here:
            return FlexivCmd(g_cmd="stop")
        else:
            return FlexivCmd(g_cmd="stop", q=self.q, kp=self.kp, kv=self.kv)

    def applicable(self, state, tic):
        """ """
        if self.stay_here:
            if np.linalg.norm(self.q - state.q) > self.max_error_stay_here:
                return False
        return True



