from FlexivPy.sim.mujoco import FlexivMujocoSim
from FlexivPy.controllers.taskspace import RRTController
import numpy as np
from FlexivPy.controllers.runners import blocking_runner

if __name__ == "__main__":
    # robot = robot_client.Flexiv_client(render=False, create_sim_server=False)
    robot = FlexivMujocoSim(render=True)
    dt_control = 0.01
    controller = RRTController(approx_dt=dt_control)
    goal = np.array([0.5, -0.698, 0.000, 1.571, -0.000, 0.698, -0.000])
    start = robot.get_robot_state().q
    controller.compute_path(start, goal)
    blocking_runner(robot, controller, dt=dt_control, timeout=100, sync_sim=True, dt_sim=0.001)
    robot.close()
