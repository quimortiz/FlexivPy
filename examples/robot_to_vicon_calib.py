import FlexivPy.robot.sim.sim_robot as sim_robot
import FlexivPy.robot.robot_client as robot_client
import FlexivPy.robot.model.model_robot as model_robot
import easy_controllers
import yaml
import numpy as np
import time
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
import os
import pickle
from SimpleHandEye.solvers import OpenCVSolver
import motioncapture
import sys


from vicon_env_state import ViconEnvState

solver = OpenCVSolver()






ASSETS_PATH = "FlexivPy/assets/"
urdf = os.path.join(ASSETS_PATH, "flexiv_rizon10s_kinematics.urdf")
meshes_dir = os.path.join(ASSETS_PATH, "meshes")

pin_model = RobotWrapper.BuildFromURDF(urdf, meshes_dir)
frame_id_flange = pin_model.model.getFrameId("flange")


robot = robot_client.Flexiv_client()



generate_trajectory = False


if generate_trajectory:

    status = easy_controllers.run_controller(
        robot,
        easy_controllers.GoJointConfigurationSlow(
            qgoal=np.array([0.0, -0.698, 0.000, 1.571, -0.000, 0.698, -0.000]), max_dq=2*0.01
        ),
        sync_sim=False,
        dt_sim=None,
        dt=0.01,
        max_time=120,
    )




    controller = easy_controllers.JointFloating()


    last_elapsed_time = 0
    store_every_s = 1.
    states  = []

    class StateRecorder:
        def __init__(self):
            self.states = []
            self.last_elapsed_time = 0
            self.store_every_s = .1
        def callback(self, robot, cmd, elapsed_time):
            if elapsed_time - self.last_elapsed_time > self.store_every_s:
                self.last_elapsed_time = elapsed_time
                self.states.append(robot.get_robot_state())


    state_recorder = StateRecorder()

    status = easy_controllers.run_controller(robot, controller,  dt=.005 , max_time = 60, callback = state_recorder.callback)


    # lets store the data to a file

    qs = [ s.q for s in state_recorder.states] 

    with open('/tmp/data.yaml', 'w') as f:
        yaml.dump(qs, f)

    sys.exit(0)


else: 
    file = "data/data_robot_vicon_calib_v2.yaml"
    env_state = ViconEnvState(D_ref_frame_wrt_vicon_frame = {})
    # go home
    qhome = np.array([0.0, -0.698, 0.000, 1.571, -0.000, 0.698, -0.000])
    status = easy_controllers.run_controller(
        robot,
        easy_controllers.GoJointConfiguration(
                qdes=qhome, max_v=0.5, max_extra_time_rel=1.0
            ),
        sync_sim=False,
        dt_sim=None,
        dt=0.01,
        max_time=120,
    )
    with open(file, 'r') as f:
        qs = yaml.safe_load(f)


    num_points = 20
    one_every = len(qs) // num_points

    waypoints = qs[::one_every]

    robot_ts = []
    vicon_ts = []
    for q in waypoints:
        status = easy_controllers.run_controller(
            robot,
            easy_controllers.GoJointConfiguration(
                qdes=q, max_v=0.3, max_extra_time_rel=1.0,
                  error_goal=5 * 1e-2,
            ),
            dt=0.01,
            max_time=30,
        )

        status = easy_controllers.run_controller(
            robot,
            easy_controllers.Stay(),
            dt=0.01,
            max_time=2.,
        )

        s = robot.get_robot_state()
        T = pin_model.framePlacement(np.array(s.q), frame_id_flange, update_kinematics=True)
        robot_ts.append(T)

        Tvicon = env_state.get_state().get("end_eff_flexiv",None)
        vicon_ts.append(Tvicon)


        if status != easy_controllers.ControllerStatus.GOAL_REACHED:
            pass
            # print("warning: Error")
            # break
        # 
    print("done")
    # lets save with pickle.

    all_data = { "robot_ts": robot_ts, "vicon_ts": vicon_ts }
    with open("/tmp/ts.pkl", "wb") as f:
        pickle.dump(all_data, f)


    A_list =  [ T.np for T in robot_ts]
    B_list = [ T.np for T in vicon_ts]
    X, Y = solver.solve(A_list, B_list)

    print("X, ef_T_marker", X)
    print("Y, Base_T_viconW", Y)

   