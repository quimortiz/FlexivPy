import FlexivPy.robot.sim.sim_robot as sim_robot
import FlexivPy.robot.robot_client as robot_client
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

# Run with:
# PYTHONPATH=. python examples/move_camera.py --mode=sim
# PYTHONPATH=. python examples/move_camera.py --mode=sim_async
# PYTHONPATH=. python examples/move_camera.py --mode=real




def view_image(image):
    cv2.imshow(f"tmp-see_images", image)
    while True:
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    cv2.destroyWindow("tmp-see_images")


class Callback:
    def __init__(self):
        self.tic_last_image = -1
        self.imgs = []
        self.dt = .1
    def __call__(self, robot, cmd, tic):
        if tic - self.tic_last_image > self.dt and robot.get_env_image() is not None:
            self.tic_last_image = tic
            self.imgs.append(robot.get_env_image())



argp = argparse.ArgumentParser(description="FlexivPy")
argp.add_argument("--mode", type=str, default="sim", help="mode: real, sim, sim_async")
args = argp.parse_args()

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
frame_id_flange = pin_model.model.getFrameId("flange")


camera_proc = None

if args.mode == "sim":
    robot = sim_robot.FlexivSim(
        render=True,
        q0=np.array([0.0, -0.698, 0.000, 1.571, -0.000, 0.698, -0.000]),
        pin_model=pin_model,
        render_images=True,
        camera_name="camera_link7",
    )



if args.mode == "sim_async":
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
        "--render_images",
        "--camera_name", "camera_link7", 
        "--joints",
    ] + joints

    robot = robot_client.Flexiv_client(cmd)

# todo: run this on the real robot.

if args.mode == "real":
    robot = robot_client.Flexiv_client()
    cmd_camera = [ "python",  "examples/image_publisher.py", "--camera_id", "8" ]
    camera_proc = subprocess.Popen(cmd_camera)


try :

    time.sleep(2.) # small wait to start receiving images


    status = easy_controllers.run_controller(
        robot,
        easy_controllers.GoJointConfigurationSlow(
            qgoal=np.array([0.0, -0.698, 0.000, 1.571, -0.000, 0.698, -0.000]),
            max_dq=0.01),
        sync_sim=args.mode == "sim", dt_sim=robot.dt if args.mode == "sim" else None,
        dt=0.01,
        max_time=120,
    )

    s = robot.get_robot_state()
    print("current state of the robot")
    print(s)
    print("we can also get images and object poses if available!")
    env_image = robot.get_env_image()
    obj_poses = robot.get_env_state()

    T = pin_model.framePlacement( np.array(s.q), frame_id_flange, update_kinematics=True)
    print("T of link flange is \n", T)
    print("the position of the flange is \n", T.translation)
    print("the orientation of the flange is \n", T.rotation)


    if env_image is None:
        print("no image is available!")
    else:
        view_image(env_image)

    callback = Callback()
    print('starting controller')
    status = easy_controllers.run_controller(
        robot,
        easy_controllers.GoJointConfiguration(
            qdes=np.array([0.5, -0.698, 0.000, 1.571, -0.000, 0.698, -0.000]),
            max_v=0.2,
            max_extra_time_rel=0.2,
            kp_scale=kp_scale,
            kv_scale=kv_scale,
        ),
        dt=0.01,
        max_time=120,
        sync_sim=args.mode == "sim",
        dt_sim=robot.dt if args.mode == "sim" else None,
        callback=callback
    )


    print("controller done!\n Status is ", status)

    print("current state of the robot")
    print(robot.get_robot_state())
    print("we can also get images and object poses if available!")
    env_image = robot.get_env_image()
    obj_poses = robot.get_env_state()

    view_image(env_image)

    input("Press Enter to continue...")

    print("we have stored all imgs")
    print("len imgs")
    print(len(callback.imgs))

    for img in callback.imgs:
        view_image(img)


    # we can also move camera to desired position and get image
    # Method A
    s = robot.get_robot_state()
    T = pin_model.framePlacement( np.array(s.q), frame_id_flange, update_kinematics=True)

    p  = T.translation
    p += np.array([0.1, 0.1, 0.1])
    oMdes = pin.SE3(T.rotation,p )
    controller = easy_controllers.GoEndEffectorPose(
        robot=pin_model,
        frame_id = frame_id_flange,
        error = 1e-2,
        oMdes  =oMdes
    )

    status = easy_controllers.run_controller(
        robot,
        controller, 
        sync_sim=args.mode == "sim",
        dt_sim=robot.dt if args.mode == "sim" else None,
        dt=0.01,
        max_time=120,
    )
    # Method B
    print('status is', status)

    view_image(robot.get_env_image())

    s = robot.get_robot_state()
    T = pin_model.framePlacement( np.array(s.q), frame_id_flange, update_kinematics=True)

    p  = T.translation
    p -= np.array([0.2, 0.2, 0.2])
    rpy = pin.rpy.rpyToMatrix(np.array([0,0,1.]))
    desM = T.rotation @ rpy
    print('desM is')
    print(desM)
    oMdes = pin.SE3(desM,p )
    controller = easy_controllers.InverseKinematicsController(
        robot = pin_model,  oMdes = oMdes,
        approx_dt = 0.01,
        frame_id = frame_id_flange, kp_scale= 1.2 * kp_scale, kv_scale = 1.2 * kv_scale)

    status = easy_controllers.run_controller(
        robot,
        controller, 
        sync_sim=args.mode == "sim",
        dt_sim=robot.dt if args.mode == "sim" else None,
        dt=0.01,
        max_time=120,
    )


    print('status is', status)

    view_image(robot.get_env_image())

finally:
    print("closing the robot")
    time.sleep(1.)
    robot.close()

    if camera_proc is not None:
        camera_proc.terminate()
        camera_proc.wait()
