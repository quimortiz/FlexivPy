from FlexivPy.robot.model.model_robot import FlexivModel
import numpy as np

robot = FlexivModel(render=True)

q = np.array([
    0,
    -.75,
    0,
    1.5,
    0,
    .75,
    0 ])

robot.display(q)

input("Press Enter to continue..." )
