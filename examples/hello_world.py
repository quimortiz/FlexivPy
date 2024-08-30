import FlexivPy.robot.sim.sim_robot as sim_robot
import FlexivPy.robot.robot_client as robot_client
import FlexivPy.robot.model.model_robot as model_robot
import numpy as np
import time
import yaml
import argparse
import pinocchio as pin
import easy_controllers
import pinocchio as pin
from FlexivPy.robot.dds.flexiv_messages import (
    FlexivCmd,
    FlexivState,
    EnvState,
    EnvImage,
)
import argparse
import os
from pinocchio.robot_wrapper import RobotWrapper


argp = argparse.ArgumentParser(description="FlexivPy")
argp.add_argument("--mode", type=str, default="sim", help="mode: real, sim, sim_async")

args = argp.parse_args()

if args.mode not in ["sim", "real", "sim_async"]:
    raise ValueError("mode not recognized")


ref_kp = np.array([3000.0, 3000.0, 800.0, 800.0, 200.0, 200.0, 200.0])
ref_kv = np.array([80.0, 80.0, 40.0, 40.0, 8.0, 8.0, 8.0])

kp_scale = 1.0
kv_scale = 1.0

if args.mode in ["sim", "sim_async"]:
    # in mujoco lower gains work better!
    kp_scale = 0.2
    kv_scale = 0.2


ASSETS_PATH = "FlexivPy/assets/"
urdf = os.path.join(ASSETS_PATH, "flexiv_rizon10s_kinematics.urdf")
meshes_dir = os.path.join(ASSETS_PATH, "meshes")
pin_model = RobotWrapper.BuildFromURDF(urdf, meshes_dir)


if args.mode == "sim":
    robot = sim_robot.FlexivSim(
        render=True,
        q0=np.array([0.0, -0.698, 0.000, 1.571, -0.000, 0.698, -0.000]),
        pin_model=pin_model,
    )
elif args.mode == "sim_async":
    base_path = ""
    config = "FlexivPy/config/robot.yaml"
    xml_path = "FlexivPy/assets/mjmodel.xml"
    joints = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]

    cmd = [
        "python",
        base_path + "FlexivPy/robot/sim/sim_robot_async.py",
        "--render",
        "--config",
        config,
        "--xml_path",
        xml_path,
        "--urdf",
        urdf,
        "--meshes_dir",
        meshes_dir,
        "--joints",
    ] + joints

    robot = robot_client.Flexiv_client(cmd)
elif args.mode == "real":
    # I can also start the server here if i provide a cmd,
    # similar to sim async.
    robot = robot_client.Flexiv_client()


try:

    print("current state of the robot")
    print(robot.get_robot_state())
    print("we can also get images and object poses if available!")
    env_image = robot.get_env_image()
    obj_poses = robot.get_env_state()
    if env_image is None:
        print("images are not available!")
    if obj_poses is None:
        print("object poses are not available!")
    # TODO: mass do not match!! -- fix this!!
    status = easy_controllers.run_controller(
        robot,
        easy_controllers.GoJointConfiguration(
            qdes=np.array([0.0, -0.698, 0.000, 1.571, -0.000, 0.698, -0.000]),
            max_v=0.2,
            max_extra_time_rel=0.2,
            kp_scale=kp_scale,
            kv_scale=kv_scale,
        ),
        dt=0.005,
        max_time=120,
        sync_sim=args.mode == "sim",
        dt_sim=robot.dt if args.mode == "sim" else None,
    )
    print("status is:", status)

    A = 4
    w = 0.5
    tic_start = time.time()
    max_time = 10
    dt = 0.01
    counter = 0
    # you can write the control loop by hand!
    qref = np.array(robot.get_robot_state().q)
    while True:
        tic = time.time()
        elapsed_time = tic - tic_start if args.mode != "sim" else counter * dt

        s = robot.get_robot_state()

        tau_ff = np.zeros(7)
        tau_ff[0] = A * np.sin(w * elapsed_time)

        add_pd_in_robot_server = True  # preferred, because PD law is evaluate at 1KHz

        movement_kp = np.copy(ref_kp)
        movement_kp[0] = 0.0

        if add_pd_in_robot_server:
            cmd = FlexivCmd(
                tau_ff=tau_ff, q=qref, kp=kp_scale * movement_kp, kv=kv_scale * ref_kv
            )
        else:
            tau_ff += -kv_scale * ref_kv * np.array(s.dq)
            tau_ff += kp_scale * movement_kp * (qref - np.array(s.q))
            cmd = FlexivCmd(tau_ff=tau_ff)

        robot.set_cmd(cmd)

        if args.mode == "sim":
            num_steps = int(dt // robot.dt)
            print("num_steps", num_steps)
            for i in range(num_steps):
                robot.step()

        # we apply this also in sync simulation because we do not want to go faster than realtime
        time.sleep(max(0, dt - (time.time() - tic)))

        if args.mode != "sim" and tic - tic_start > max_time:
            break
        if args.mode == "sim" and counter * dt > max_time:
            break

        counter += 1

    # lets go back home
    print("trying to go back home")
    status = easy_controllers.run_controller(
        robot,
        easy_controllers.GoJointConfiguration(
            qdes=np.array([0.0, -0.698, 0.000, 1.571, -0.000, 0.698, -0.000]),
            max_v=0.2,
            max_extra_time_rel=0.2,
            kv_scale=kv_scale,
            kp_scale=kp_scale,
        ),
        sync_sim=args.mode == "sim",
        dt_sim=robot.dt if args.mode == "sim" else None,
        dt=0.005,
        max_time=120,
    )
    print("status is", status)


finally:
    print("inside finally!")
    print("note: closing the mujoco simulator sometimes throws a seg. fault")
    robot.close()
    print("done")
