import FlexivPy.robot.interface as interface
import easy_controllers

robot = interface.Flexiv_client()
controller = easy_controllers.JointFloating()
status = easy_controllers.run_controller(robot, controller, dt=0.005, max_time=120)
