import time
import mujoco
import mujoco.viewer
import numpy as np
from scipy.spatial.transform import Rotation
import os
import yaml

import FlexivPy.robot.sim.sim_robot as sim_robot
import FlexivPy.robot.model.model_robot as model_robot
import numpy as np
import time


from enum import auto
from typing import TYPE_CHECKING, Optional
from dataclasses import dataclass

import cyclonedds.idl as idl
import cyclonedds.idl.annotations as annotate
import cyclonedds.idl.types as types
from cyclonedds.idl.types import int64, float32, array
from cyclonedds.idl.types import int64, float32, sequence
import argparse

# root module import for resolving types
# import vehicles


import time
import random

from cyclonedds.core import Qos, Policy
from cyclonedds.domain import DomainParticipant
from cyclonedds.pub import Publisher, DataWriter
from cyclonedds.topic import Topic
from cyclonedds.util import duration

from cyclonedds.core import Listener, Qos, Policy
from cyclonedds.domain import DomainParticipant
from cyclonedds.topic import Topic
from cyclonedds.sub import Subscriber, DataReader
from cyclonedds.util import duration
import time
from FlexivPy.robot.dds.flexiv_messages import FlexivCmd, FlexivState


ASSETS_PATH = "assets/"


class FlexivSim_dds_server:

    def __init__(self, dt, render, max_time, q0, pin_model):

        self.pin_model = pin_model
        self.render = render
        self.dt = dt
        self.max_time = max_time
        self.domain_participant = DomainParticipant()
        self.topic_state = Topic(self.domain_participant, "FlexivState", FlexivState)
        self.publisher = Publisher(self.domain_participant)
        self.writer = DataWriter(self.publisher, self.topic_state)

        # I also have to create a subscriber to receive the data

        self.topic_cmd = Topic(self.domain_participant, "FlexivCmd", FlexivCmd)
        self.subscriber = Subscriber(self.domain_participant)
        self.reader = DataReader(self.subscriber, self.topic_cmd)

        # create a simulated robot
        self.robot = sim_robot.FlexivSim(
            render=self.render, dt=self.dt, q0=q0, pin_model=self.pin_model
        )

        self.stop_dt = 0.01  # [s] if i don't receive a cmd in this time, stop the robot

        self.default_cmd = {
            "tau_ff": np.zeros(7),
            "q": np.zeros(7),
            "dq": np.zeros(7),
            "kp": np.zeros(7),
            "kv": 10.0 * np.ones(7),
        }

    def run(self):
        time_start = time.time()
        warning_send = False
        good_satus_send = False
        history_dt = []
        warning_dt_send = False

        # we start with the default cmd
        self.robot.set_cmd(self.default_cmd)
        time_last_cmd = time.time()

        while time.time() - time_start < self.max_time:

            tic = time.time()

            if time.time() - time_last_cmd > self.stop_dt:
                time_last_cmd = time.time()
                self.robot.set_cmd(self.default_cmd)
                if not warning_send:
                    print("no cmd recieved in time! -- using default cmd")
                    warning_send = True
                    good_satus_send = False

            # last_sample = None
            # samples = self.reader.read()
            # if len(samples):
            #     last_sample = samples[-1]
            #     self.reader.take(N=len(samples))

            # last_sample = self.reader.take()
            # if last_sample:
            #     while True:
            #         a = self.reader.take()
            #         if not a:
            #             break
            #         else:
            #             last_sample = a
            #     last_sample = last_sample[0]

            last_sample = None
            max_int = 65535
            samples = self.reader.take(N=max_int)

            if len(samples):
                last_sample = samples[-1]

            if last_sample and type(last_sample) == FlexivCmd:
                cmd = last_sample
                time_last_cmd = time.time()
                warning_send = False
                if not good_satus_send:
                    print("cmd received")
                    good_satus_send = True
                self.robot.set_cmd(
                    {
                        "tau_ff": cmd.tau_ff,
                        "q": cmd.q,
                        "dq": cmd.dq,
                        "kp": cmd.kp,
                        "kv": cmd.kv,
                    }
                )

            self.robot.step()

            state = self.robot.getJointStates()
            msg_out = FlexivState(
                q=state["q"], dq=state["dq"], tau=np.zeros(7), timestamp=""
            )
            self.writer.write(msg_out)

            toc = time.time()
            elapsed_time = toc - tic
            history_dt.append(elapsed_time)
            if elapsed_time > self.dt:
                if not warning_dt_send:
                    warning_dt_send = True
                    print(
                        f"warning! elapsed time:  {elapsed_time} is greater than dt: {self.dt}"
                    )

            time.sleep(max(0, self.dt - elapsed_time))

        print("mean dt", np.mean(history_dt))
        print("median dt", np.median(history_dt))
        print("max dt", np.max(history_dt))
        print("min dt", np.min(history_dt))


if __name__ == "__main__":

    argp = argparse.ArgumentParser()
    argp.add_argument("--render", action="store_true", help="render the simulation")
    argp.add_argument(
        "--config", type=str, default="FlexivPy/config/robot.yaml", help="config file"
    )

    args = argp.parse_args()

    # load the config file
    with open(args.config, "r") as stream:
        config = yaml.safe_load(stream)

    q0 = config.get("q0", None)
    if q0:
        q0 = np.array(q0)

    # I need a pinocchio robot

    robot_model = model_robot.FlexivModel(
        render=False,
        q0=config.get("q0", None),
    )

    sim = FlexivSim_dds_server(
        dt=0.001, render=args.render, max_time=100.0, q0=q0, pin_model=robot_model.robot
    )
    sim.run()
