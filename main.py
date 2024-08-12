import FlexivPy.robot.sim.sim_robot as sim_robot
import FlexivPy.robot.robot_client as robot_client
import FlexivPy.robot.model.model_robot as model_robot
import numpy as np
import time


import argparse

argp = argparse.ArgumentParser(description="FlexivPy")
argp.add_argument("--mode", type=str, default="sim", help="mode: real, sim, sim_async")
argp.add_argument("--render", action="store_true", help="render the simulation")
args = argp.parse_args()

if args.mode not in ["sim", "real", "sim_async"]:
    raise ValueError("mode not recognized")


robot_model = model_robot.FlexivModel()

server_process = None
if args.mode == "sim_async":
    robot = robot_client.Flexiv_client(render=args.render, create_server=True)

elif args.mode == "sim":
    robot = sim_robot.FlexivSim(render=args.render)

elif args.mode == "real":
    robot = robot_client.Flexiv_client(create_server=False)

for i in range(100000):
    tic = time.time()
    q = robot.getJointStates()
    cmd = {
        "tau_ff": np.zeros(7),
        "q": np.zeros(7),
        "dq": np.zeros(7),
        "kp": np.zeros(7),
        "kv": np.zeros(7),
    }
    robot.set_cmd(cmd)

    if args.mode == "sim":
        robot.step()  # note: in sim_async and real this does nothing!

    toc = time.time()

    if (toc - tic) > robot.dt:
        print(f"warning: loop time {toc-tic} is greater than dt")

    time.sleep(max(0, robot.dt - (toc - tic)))
