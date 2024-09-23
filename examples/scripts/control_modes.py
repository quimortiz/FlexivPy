import FlexivPy.robot.sim.sim_robot as sim_robot
import FlexivPy.robot.interface as interface
import numpy as np
import time
import argparse
import pinocchio as pin
import easy_controllers
import pinocchio as pin
import argparse
import os
from pinocchio.robot_wrapper import RobotWrapper
import subprocess
import cv2





argp = argparse.ArgumentParser(description="FlexivPy")
argp.add_argument("--control_mode", type=int, default=2, help="control mode: 1 Pos&Vel, 2 Torque+PD, 3 Vel")
args = argp.parse_args()

ref_kp = np.array([3000.0, 3000.0, 800.0, 800.0, 200.0, 200.0, 200.0])
ref_kv = np.array([80.0, 80.0, 40.0, 40.0, 8.0, 8.0, 8.0])

kp_scale = 1.0
kv_scale = 1.0


ASSETS_PATH = "FlexivPy/assets/"
urdf = os.path.join(ASSETS_PATH, "flexiv_rizon10s_kinematics.urdf")
meshes_dir = os.path.join(ASSETS_PATH, "meshes")
pin_model = RobotWrapper.BuildFromURDF(urdf, meshes_dir)
frame_id_flange = pin_model.model.getFrameId("flange")



robot = interface.Flexiv_client()

qhome = np.array([0.0, -0.698, 0.000, 1.571, -0.000, 0.698, -0.000])
q1 = np.array([1.0, -0.698, 0.000, 1.571, -0.000, 0.698, -0.000])


if args.control_mode == 1:


        make_controller = lambda g: easy_controllers.GoJointConfiguration(
            qdes=g,
            max_v=.2,
            max_extra_time_rel=0.2,
            error_running=2 * 1, 
        control_mode="position",
    )

        status = easy_controllers.run_controller(
            robot,
                    make_controller(qhome),
            dt=0.01,
            max_time=10.,
        )
        print("status", status)

        status = easy_controllers.run_controller(
        robot, make_controller(q1),
                        dt=0.01,
                        max_time=10)

        print("status", status)


if args.control_mode == 2:



        make_controller = lambda g: easy_controllers.GoJointConfiguration(
            qdes=g,
            max_v=.2,
            max_extra_time_rel=0.2,
            error_running=2 * 1e-2,
            error_goal=5 * 1e-3,
            min_motion_time=1.0,
            control_mode="torque")

        status = easy_controllers.run_controller(
        robot,
            make_controller(qhome),
                dt=0.01,
                max_time=10.) 
        print("status", status)

        status = easy_controllers.run_controller(
        robot,
            make_controller(q1),
                dt=0.01,
                max_time=10.) 


elif args.control_mode == 3:
    status = easy_controllers.run_controller(
        robot,
        easy_controllers.GoJointConfigurationVelocity(qgoal=qhome, error_goal=0.01, max_v = .5, kd = 1.5), 
        dt=0.01,
        max_time=10.,
    )

    s = robot.get_robot_state()

    status = easy_controllers.run_controller(
        robot,
        easy_controllers.GoJointConfigurationVelocity(qgoal=q1, error_goal=0.01, max_v = .5, kd = 1.5), 
        dt=0.01,
        max_time=10.,
    )
