import FlexivPy.robot.robot_client as robot_client
import easy_controllers

robot = robot_client.Flexiv_client()
controller = easy_controllers.JointFloating()
status = easy_controllers.run_controller(robot, controller, dt=0.005, max_time=120)
