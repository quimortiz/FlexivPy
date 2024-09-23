import time
from FlexivPy.controllers.utils import *
from enum import Enum
import numpy as np
from threading import Thread

class ControllerStatus(Enum):
    UNKNOWN = 0
    GOAL_REACHED = 1
    NOT_APPLICABLE = 2
    MAX_TIME = 3
    ERROR = 4

def blocking_runner(
    robot, 
    controller, 
    timeout=60, 
    callback=None
    ):

    s = robot.get_robot_state()
    if hasattr(controller,'setup'):
        controller.setup(s)
    else:
        print('The controller.setup() is not implemented.')
        
    tic_start = time.time()
    exit_status = ControllerStatus.UNKNOWN
    counter = 0
    while True:
        tic = time.time()
        s = robot.get_robot_state()
        elapsed_time = tic - tic_start if not hasattr(robot, 'step') else counter * controller.dt

        if not controller.applicable(s, elapsed_time):
            print("controller is not applicable")
            exit_status = ControllerStatus.NOT_APPLICABLE
            break

        if controller.goal_reached(s, elapsed_time):
            exit_status = ControllerStatus.GOAL_REACHED
            print("goal reached")
            break

        if elapsed_time > timeout:
            exit_status = ControllerStatus.MAX_TIME
            print("max time reached")
            break

        cmd = controller.get_control(s, elapsed_time)

        robot.set_cmd(cmd)

        if callback is not None:
            callback(robot, cmd, elapsed_time)
        
        if hasattr(robot, 'step'):
            num_steps = int(controller.dt // robot.dt) if controller.dt>=robot.dt else 1
            for _ in range(num_steps):
                robot.step()  
        time.sleep(max(0, controller.dt - (time.time() - tic)))
        counter += 1
    return exit_status

class NonBlockingRunner:
    def __init__(self,
                 robot, 
                 controller, 
                 timeout=60, 
                 callback=None):
        s = robot.get_robot_state()
        if hasattr(controller,'setup'):
            controller.setup(s)
        else:
            print('The controller.setup() is not implemented.')
        pass
        self.robot= robot
        self.controller= controller
        self.timeout = timeout
        self.callback = callback
        self.running = True
        self.exit_status = ControllerStatus.UNKNOWN
        self.thread = Thread(target=self.loop)
        self.thread.start()

    def loop(self):
        tic_start = time.time()
        counter = 0
        while self.running:
            tic = time.time()
            s = self.robot.get_robot_state()
            elapsed_time = tic - tic_start if not hasattr(self.robot, 'step') else counter * self.controller.dt

            if not self.controller.applicable(s, elapsed_time):
                print("controller is not applicable")
                self.exit_status = ControllerStatus.NOT_APPLICABLE
                self.running=False
                break

            if self.controller.goal_reached(s, elapsed_time):
                self.exit_status = ControllerStatus.GOAL_REACHED
                print("goal reached")
                self.running=False
                break

            if elapsed_time > self.timeout:
                exit_status = ControllerStatus.MAX_TIME
                print("max time reached")
                self.running=False
                break

            cmd = self.controller.get_control(s, elapsed_time)

            self.robot.set_cmd(cmd)

            if self.callback is not None:
                self.callback(self.robot, cmd, elapsed_time)
            
            if hasattr(self.robot, 'step'):
                num_steps = int(self.controller.dt // self.robot.dt) if self.controller.dt>=self.robot.dt else 1
                for _ in range(num_steps):
                    self.robot.step()  
            time.sleep(max(0, self.controller.dt - (time.time() - tic)))
            counter += 1
    def close(self):
        self.running=False
        self.thread.join()