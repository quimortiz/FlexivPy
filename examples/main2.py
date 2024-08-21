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




# import pickle
# with open('array.pkl', 'rb') as f:
#     states = pickle.load(f)
#
# import pdb; pdb.set_trace()

robot = robot_client.Flexiv_client( render=False, create_sim_server=False)

# controller = easy_controllers.GoJointConfigurationSlow (
#                  qgoal = np.array([ 0.000, -0.698, 0.000, 1.371, -0.000, 0.698, -0.000 ]) , 
#                  error_goal = .01 ,
#                  max_dx = 0.002)
# easy_controllers.run_controller(robot, controller,  dt=.01 , max_time = 10)
#
# controller = easy_controllers.GoHomeDefault()

# controller = easy_controllers.GoJointConfiguration(
#         qdes = np.array([ 0.000, -0.698, 0.000, 1.571, -0.000, 0.698, -0.000 ]) , 
#         max_v = .2 , 
#         max_extra_time_rel = .1)
#
# easy_controllers.run_controller(robot, controller,  dt=.01 , max_time = 10)
#
# input('Press enter to continue')

controller = easy_controllers.JointFloating()

# robot_model = model_robot.FlexivModel(
#     render=False,
#     q0 = np.array( [ 0.000, -0.698, 0.000, 1.571, -0.000, 0.698, -0.000 ]),
#     # urdf="/home/quim/code/FlexivPy/FlexivPy/assets/r10s_with_capsules.urdf"
#     urdf="/home/quim/code/FlexivPy/FlexivPy/assets/flexiv_rizon10s_kinematics.urdf"
# )



states = []
try:
    dt = 0.01
    max_time = 3 * 60
    s = robot.get_robot_state()
    controller.setup(s)
    tic_start = time.time()
    while True:
        tic = time.time()
        s = robot.get_robot_state()
        states.append(s)
        
        if not controller.applicable(s , tic - tic_start):
           print("controller is not applicable")
           break
        
        if controller.goal_reached(s, tic - tic_start):
            print("goal reached")
            break

        if time.time() - tic_start > max_time:
            print("max time reached")
            break
        
        cmd = controller.get_control(s, tic - tic_start)
        # TODO: should I break if the controller returns None?
        if cmd:
            robot.set_cmd(cmd)

        time.sleep(max(0, dt - (time.time() - tic)))
except:
    print('exception!')



import pickle
with open('__array.pkl', 'wb') as f:
    pickle.dump(states, f)

# easy_controllers.run_controller(robot, controller,  dt=.002 , max_time = 120)

#
# model = robot_model.robot.model
# data = robot_model.robot.data
#
#
# for frame in model.frames:
#     print(frame.name)
#
# frame_id = model.getFrameId( "flange" )
#
# print("joint_id", frame_id)
#
# print("q", q)
# robot_model.robot.framePlacement(q, frame_id, update_kinematics=True)
#
#
# Tdes = np.array([[-1.,0.,0.],[0.,1.,0.],[0.,0.,-1.]])
# R = pin.rpy.rpyToMatrix(.0,.0,.0)
# Tdes = Tdes @ R
# pdes = p0 + np.array([.01,-0.1,0.])
# oMdes = pin.SE3(Tdes, pdes)
#
#
# controller = easy_controllers.GoEndEffectorPose
#     (
#                  robot, oMdes,  frame_id, error=1e-2 ,  max_v = .3)
#
# # todo: wait until the robot is ready!! -- as main.py
# easy_controllers.run_controller(robot, controller,  dt=.005 , max_time = 10)
#
#
