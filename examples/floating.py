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




robot = robot_client.Flexiv_client( render=False, create_sim_server=False)
controller = easy_controllers.JointFloating()

status = easy_controllers.run_controller(robot, controller,  dt=.005 , max_time = 120)
print("status", status)

