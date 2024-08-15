import FlexivPy.robot.sim.sim_robot as sim_robot
import FlexivPy.robot.robot_client as robot_client
import FlexivPy.robot.model.model_robot as model_robot
import numpy as np
import time
import yaml
import argparse
import pinocchio as pin
import easy_controllers

argp = argparse.ArgumentParser(description="FlexivPy")
argp.add_argument("--mode", type=str, default="sim", help="mode: real, sim, sim_async")
argp.add_argument("--render", action="store_true", help="render the simulation")
argp.add_argument(
    "--config", type=str, default="FlexivPy/config/robot.yaml", help="config file"
)
argp.add_argument("--create_sim_server", action="store_true", help="create a server")

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
    robot = robot_client.Flexiv_client(
        render=args.render,
        create_sim_server=args.create_sim_server,
        server_config_file=args.config,
    )

elif args.mode == "sim":
    robot = sim_robot.FlexivSim(
        render=args.render, q0=config.get("q0"), pin_model=robot_model.robot
    )
    robot.reset_state(config.get("q0"), np.zeros(7))

elif args.mode == "real":
    robot = robot_client.Flexiv_client( render=False, create_sim_server=False)


simulation_time_s = 50


warn_time_dt = False


print_state_every = 500

# controller = easy_controllers.Controller_static_q0(robot_model, config.get("q0", None))


# controller = easy_controllers.Controller_joint_example(robot_model, config.get("q0", None))

controller = easy_controllers.Controller_torque_example(robot_model, config.get("q0", None))


# controller = easy_controllers.Controller_joint_PD(robot_model)
# controller.pd_on_acceleration = True
# lets wait until we receive information from the robot

print("waiting for robot to be ready")

max_wait_time = 10

tic_wait = time.time()
robot_ready  = False
while  time.time() - tic_wait < max_wait_time:
    robot_ready = robot.is_ready()
    if robot_ready:
        break
    time.sleep(0.1)

if  not robot_ready:
    raise ValueError("robot is not ready")
else:
    print("robot is ready!")

try:
    for i in range(1000 * simulation_time_s):

        tic = time.time()
        s = robot.getJointStates()

        cmd = controller.get_control(s, robot_model.robot)
        robot.set_cmd(cmd)

        if args.mode == "sim":
            robot.step()  # note: in sim_async and real this does nothing!

        toc = time.time()

        if (toc - tic) > robot.dt and warn_time_dt:
            print(f"warning: loop time {toc-tic} is greater than dt")

        time.sleep(max(0, robot.dt - (toc - tic)))

    robot.close()
except Exception as e:
    # 'e' contains the exception details
    print(f"An error occurred: {e}")

    print("Error in the loop")
    # lets kill the async server if it was created
    robot.close()
