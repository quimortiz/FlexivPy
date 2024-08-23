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


controller = easy_controllers.GoJointConfiguration(
        qdes = np.array([ 0.0, -0.698, 0.000, 1.571, -0.000, 0.698, -0.000 ]) , 
        max_v = .3, 
        max_extra_time_rel = 1.)

status = easy_controllers.run_controller(robot, controller,  dt=.005 , max_time = 120)


controller = easy_controllers.OpenGripper()
status = easy_controllers.run_controller(robot, controller,  dt=.005 , max_time = 10)

controller = easy_controllers.CloseGripper()
status = easy_controllers.run_controller(robot, controller,  dt=.005 , max_time = 10)

time.sleep(1)
controller = easy_controllers.OpenGripper()
status = easy_controllers.run_controller(robot, controller,  dt=.005 , max_time = 10)





