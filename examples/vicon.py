import motioncapture
import argparse
import numpy as np
import time
import pinocchio as pin
import FlexivPy.robot.robot_client as robot_client
import easy_controllers
from pinocchio.robot_wrapper import RobotWrapper
import os
import sys


# create a small python viewer



# p_robot_frame = T_vicon_to_robot @ p_vicon
# R = np.eye(3)
# p = np.array([0.48, 0, 0])
# T_vicon_to_robot = pin.SE3(R, p)
# T_cube_vicon_to_ref = pin.SE3()
#
# # D_vicon_frame_to_ref_frame = {"cube2": pin.SE3.Identity() }         

from vicon_env_state import ViconEnvState


p_black = np.array([-.05, 0, -.03])
# p_black = np.array([.0, 0, .0])
R_black = np.eye(3)

T_black = pin.SE3(R_black, p_black)


p_yellow = np.array([-.055, 0, .0])
# p_yellow = np.array([.055, 0, .0])
# p_yellow = np.array([.0, 0, .0])
R_yellow  = np.eye(3)
T_yellow = pin.SE3(R_yellow, p_yellow)

D_ref_frame_wrt_vicon_frame = {"box_black":T_black, 
                              "box_yellow": T_yellow }         


Base_T_viconW = np.array( [[ 9.99952235e-01,  9.46723556e-03,  2.42897816e-03,  2.89813540e-01],
      [-9.46543818e-03,  9.99954920e-01 ,-7.50407225e-04, -1.23062361e-02],
      [-2.43597295e-03 , 7.27380039e-04,  9.99996768e-01  ,7.88964024e-04],
      [ 0.00000000e+00  ,0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])

# second trial!
# Y, Base_T_viconW [[ 9.99954198e-01  9.22798942e-03  2.53875336e-03  2.89738982e-01]
#  [-9.22578055e-03  9.99957054e-01 -8.80399757e-04 -1.22710448e-02]
#  [-2.54676865e-03  8.56937452e-04  9.99996390e-01  7.51411479e-04]
# [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]

# Base_T_viconW = np.array( [[ 9.99901417e-01,  1.34438268e-02,  4.05205743e-03,  2.89027280e-01],
#  [-1.34392108e-02,  9.99909012e-01, -1.16426401e-03, -1.13614676e-02],
#  [-4.06734091e-03 , 1.10969278e-03,  9.99991113e-01,  8.41028579e-04],
#  [ 0.00000000e+00  ,0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])


Base_T_viconW = np.array( [[ 9.99840687e-01,  1.77210742e-02,  2.13643729e-03,  2.89607673e-01],
 [-1.77168764e-02,  9.99841107e-01, -1.96799840e-03, -1.12452615e-02],
 [-2.17097287e-03,  1.92983388e-03,  9.99995781e-01, -7.68970272e-04],
 [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])




vicon_wrt_robot = pin.SE3(Base_T_viconW[:3,:3], Base_T_viconW[:3,3])

# vicon_wrt_robot = pin.SE3(np.eye(3), np.array([0.29, 0, 0]) )

# Base_T_viconW =  pin.SE3(np.eye(3), np.array([0.3, 0, 0]))


# Base_T_viconW.translation = # pin.SE3(R_yellow, p_yellow)




env_state = ViconEnvState(
    T_vicon_to_robot = vicon_wrt_robot,
    D_ref_frame_wrt_vicon_frame = D_ref_frame_wrt_vicon_frame  )


# import 

# class QEnvState:
#
#     def __init__(
#         self, system_type="vicon", hostname="192.168.2.70", T_vicon_to_robot=pin.SE3.Identity()
#     ):
#         self.mc = motioncapture.connect(system_type, {"hostname": hostname})
#         self.state = None
#         self.T_vicon_to_robot = T_vicon_to_robot
#         self.D_vicon_frame_to_ref_frame = D_vicon_frame_to_ref_frame
#         for i in range(1000):
#             self.mc.waitForNextFrame()
#             time.sleep(.001)
#        
#
#     def get_state(self):
#         self.mc.waitForNextFrame()        
#         D = {}
#         for name, obj in self.mc.rigidBodies.items():
#             quat = pin.Quaternion(w=obj.rotation.w, x=obj.rotation.x, y=obj.rotation.y, z=obj.rotation.z)
#             pos = obj.position
#             T = pin.SE3(quat, pos)
#
#             if name in self.D_vicon_frame_to_ref_frame:
#                 T = self.D_vicon_frame_to_ref_frame[name] * T # TODO: check the math here!
#             T = self.T_vicon_to_robot * T # transform to the robot frame
#             D[name] = T
#            
#             
#         return D


# get the state of the environment
print("getting state of the environment")
state = env_state.get_state()
print(state['box_yellow'].translation)
print(state['box_yellow'].rotation)
print(state['box_black'].translation)
print(state['box_black'].rotation)

z_offset = 0.3

# sys.exit(0)
# input("Press Enter to continue...")
# state = env_state.get_state()
# print(state)
# print(state['cube2'].translation)
# print(state['cube2'].rotation)

# input("Press Enter to continue...")
# state = env_state.get_state()
# print(state)
# print(state['cube2'].translation)
# print(state['cube2'].rotation)

# input("Press Enter to continue...")
# state = env_state.get_state()
# print(state)
# print(state['cube2'].translation)
# print(state['cube2'].rotation)


robot = robot_client.Flexiv_client()


time.sleep(2.0)  # small wait to start receiving images


# qhome = np.array([0.0, -0.698, 0.000, 1.571, -0.000, 0.698, -0.000])
#
# controller_status = easy_controllers.run_controller(
#     robot,
#     easy_controllers.GoJointConfiguration(
#         qdes=qhome, max_v=0.5, max_extra_time_rel=1.0
#     ),
#     dt=0.002,
#     max_time=30,
# )


ASSETS_PATH = "FlexivPy/assets/"
urdf = os.path.join(ASSETS_PATH, "flexiv_rizon10s_kinematics.urdf")
meshes_dir = os.path.join(ASSETS_PATH, "meshes")

s = robot.get_robot_state()
pin_model = RobotWrapper.BuildFromURDF(urdf, meshes_dir)
frame_id_flange = pin_model.model.getFrameId("flange")


print("current state of the robot")
print(s)
print("we can also get images and object poses if available!")
env_image = robot.get_env_image()
obj_poses = robot.get_env_state()

T = pin_model.framePlacement(np.array(s.q), frame_id_flange, update_kinematics=True)
print("T of link flange is \n", T)
print("the position of the flange is \n", T.translation)
print("the orientation of the flange is \n", T.rotation)


oMdes = pin.SE3( T.rotation , state['box_black'].translation)
oMdes.translation[2] += z_offset

kp_scale = 1.0
kv_scale = 1.0

print("oMdes is ", oMdes)

Rdes = np.array( [[-1. , 0., 0. ],
                 [ 0.  , 1.,  0.],
                 [ 0. , 0. , -1.]] )

controller = easy_controllers.InverseKinematicsControllerVicon(
     robot=pin_model,
     oMdes=oMdes,
     approx_dt=0.01,
     frame_id=frame_id_flange,
     kp_scale=1.2 * kp_scale,
     kv_scale=1.2 * kv_scale,
     vicon = env_state, 
     kvv = 2*10.,
    w_weight = 1.,
     Rdes = Rdes)




status = easy_controllers.run_controller(
    robot,
    controller,
    sync_sim=False,
    dt_sim=None,
    dt=0.01,
    max_time=np.inf,
)
print(status)

