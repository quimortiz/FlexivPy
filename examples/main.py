import FlexivPy.robot.sim.sim_robot as sim_robot
import FlexivPy.robot.robot_client as robot_client
import FlexivPy.robot.model.model_robot as model_robot
import numpy as np
import time
import yaml
import argparse
import pinocchio as pin

argp = argparse.ArgumentParser(description="FlexivPy")
argp.add_argument("--mode", type=str, default="sim", help="mode: real, sim, sim_async")
argp.add_argument("--render", action="store_true", help="render the simulation")
argp.add_argument("--config", type=str, default="FlexivPy/config/robot.yaml", help="config file")
args = argp.parse_args()

if args.mode not in ["sim", "real", "sim_async"]:
    raise ValueError("mode not recognized")

with open(args.config, "r") as stream:
    config = yaml.safe_load(stream)

robot_model = model_robot.FlexivModel(
    render=False, 
    q0=config.get("q0", None),
)

server_process = None
if args.mode == "sim_async":
    robot = robot_client.Flexiv_client(render=args.render, create_server=True,
                                       server_config_file=args.config)

elif args.mode == "sim":
    robot = sim_robot.FlexivSim(render=args.render, q0=config.get('q0'),
                                pin_model=robot_model.robot)
    robot.reset_state(config.get('q0'), np.zeros(7))

elif args.mode == "real":
    robot = robot_client.Flexiv_client(create_server=False)

des_q = np.array(config.get("q0"))



for i in range(100000):
    tic = time.time()
    s = robot.getJointStates() # in Flexiv_client with simulation server, s may be None

    # NOTE: gravity compesation is running inside the simulator!!
    cmd = {
        "tau_ff": np.zeros(7),
        "q": des_q,
        "dq": np.zeros(7),
        "kp": 10. * np.ones(7),
        "kv": 5. * np.ones(7),
    }
    
    robot.set_cmd(cmd)

    if args.mode == "sim":
        robot.step()  # note: in sim_async and real this does nothing!

    toc = time.time()

    if (toc - tic) > robot.dt:
        print(f"warning: loop time {toc-tic} is greater than dt")

    time.sleep(max(0, robot.dt - (toc - tic)))
