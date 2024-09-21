import FlexivPy.robot.interface as interface
import numpy as np
import time
import easy_controllers


robot = interface.Flexiv_client()

controller = easy_controllers.GoJointConfiguration(
    qdes=np.array([0.0, -0.698, 0.000, 1.571, -0.000, 0.698, -0.000]),
    max_v=0.3,
    max_extra_time_rel=1.0,
)

status = easy_controllers.run_controller(robot, controller, dt=0.005, max_time=120)


controller = easy_controllers.OpenGripper()
status = easy_controllers.run_controller(robot, controller, dt=0.005, max_time=10)

controller = easy_controllers.CloseGripper()
status = easy_controllers.run_controller(robot, controller, dt=0.005, max_time=10)

time.sleep(1)
controller = easy_controllers.OpenGripper()
status = easy_controllers.run_controller(robot, controller, dt=0.005, max_time=10)
