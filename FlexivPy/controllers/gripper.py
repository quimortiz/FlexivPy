
from FlexivPy.robot.dds.flexiv_messages import (
    FlexivCmd,
)
import numpy as np
from FlexivPy.controllers.utils import *

class OpenGripper:
    def __init__(self, stay_here=True, max_error_stay_here=1e-1):
        self.stay_here = stay_here
        self.kp = 1.0 * np.array([3000.0, 3000.0, 800.0, 800.0, 200.0, 200.0, 200.0])
        self.kv = 1.0 * np.array([80.0, 80.0, 40.0, 40.0, 8.0, 8.0, 8.0])
        self.max_error_stay_here = max_error_stay_here

    def setup(self, s):
        self.q = np.array(s.q)

    def get_control(self, state, tic):
        if not self.stay_here:
            return FlexivCmd(g_cmd="open")
        else:
            return FlexivCmd(g_cmd="open", q=self.q, kp=self.kp, kv=self.kv)

    def applicable(self, state, tic):
        """ """
        if self.stay_here:
            if np.linalg.norm(self.q - state.q) > self.max_error_stay_here:
                return False
        return True

    def goal_reached(self, state, tic):
        """ """
        if state.g_state == "open":
            return True
        else:
            return False


class CloseGripper:
    def __init__(self, stay_here=True, max_error_stay_here=1e-1):
        self.stay_here = stay_here
        self.kp = 1.0 * np.array([3000.0, 3000.0, 800.0, 800.0, 200.0, 200.0, 200.0])
        self.kv = 1.0 * np.array([80.0, 80.0, 40.0, 40.0, 8.0, 8.0, 8.0])
        self.max_error_stay_here = max_error_stay_here

    def setup(self, s):
        self.q = np.array(s.q)

    def get_control(self, state, tic):
        if not self.stay_here:
            return FlexivCmd(g_cmd="close")
        else:
            return FlexivCmd(q=self.q, g_cmd="close", kp=self.kp, kv=self.kv)

    def applicable(self, state, tic):
        """ """
        if self.stay_here:
            if np.linalg.norm(self.q - state.q) > self.max_error_stay_here:
                return False
        return True

    def goal_reached(self, state, tic):
        """ """

        if state.g_state == "closed" or state.g_state == "holding":
            return True
        else:
            return False


class StopGripper:
    def __init__(self, stay_here=True):
        self.stay_here = stay_here
        self.kp = 1.0 * np.array([3000.0, 3000.0, 800.0, 800.0, 200.0, 200.0, 200.0])
        self.kv = 1.0 * np.array([80.0, 80.0, 40.0, 40.0, 8.0, 8.0, 8.0])

    def setup(self, s):
        self.q = np.array(s.q) 

    def get_control(self, state, tic):
        # TODO: check that this is running in the robot.
        if not self.stay_here:
            return FlexivCmd(g_cmd="stop")
        else:
            return FlexivCmd(g_cmd="stop", q=self.q, kp=self.kp, kv=self.kv)

    def applicable(self, state, tic):
        """ """
        if self.stay_here:
            if np.linalg.norm(self.q - state.q) > self.max_error_stay_here:
                return False
        return True



