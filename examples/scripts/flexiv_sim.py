import pinocchio as pin
import numpy as np
from FlexivPy.robot.model.pinocchio import FlexivModel
from FlexivPy.sim.MuJoCo import FlexivSimMujoco as FlexivSim
from FlexivPy.robot.dds.flexiv_messages import FlexivCmd

model = FlexivModel()
robot = FlexivSim(render=True, dt=0.002, mode="position")
