import FlexivPy.robot.sim.sim_robot as sim_robot
import FlexivPy.robot.robot_client as robot_client
import FlexivPy.robot.model.model_robot as model_robot
import easy_controllers

robot = robot_client.Flexiv_client()
controller = easy_controllers.JointFloating()
status = easy_controllers.run_controller(robot, controller,  dt=.005 , max_time = 120)


