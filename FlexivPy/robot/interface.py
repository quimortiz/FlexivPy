import time
import numpy as np
import os

from cyclonedds.domain import DomainParticipant
from cyclonedds.topic import Topic
from cyclonedds.sub import Subscriber, DataReader
import time

from cyclonedds.domain import DomainParticipant
from cyclonedds.pub import Publisher, DataWriter
from cyclonedds.topic import Topic


import subprocess
import time

from FlexivPy.robot.dds.flexiv_messages import (
    FlexivCmd,
    FlexivState,
    EnvState,
    EnvImage,
)
from FlexivPy.robot.dds.subscriber import get_last_msg
import cv2
from typing import List


class FlexivDDSClient:
    def __init__(self, create_server_cmd: List[str] = []):

        self.create_server_cmd = create_server_cmd
        self.domain_participant = DomainParticipant()

        self.topic_cmd = Topic(self.domain_participant, "FlexivCmd", FlexivCmd)
        self.publisher = Publisher(self.domain_participant)
        self.writer = DataWriter(self.publisher, self.topic_cmd)

        self.topic_state = Topic(self.domain_participant, "FlexivState", FlexivState)
        self.subscriber = Subscriber(self.domain_participant)
        self.reader = DataReader(self.subscriber, self.topic_state)

        self.topic_env_state = Topic(self.domain_participant, "EnvState", EnvState)
        self.subscriber_env = Subscriber(self.domain_participant)
        self.reader_env = DataReader(self.subscriber_env, self.topic_env_state)

        self.topic_env_image = Topic(self.domain_participant, "EnvImage", EnvImage)
        self.subscriber_env_image = Subscriber(self.domain_participant)
        self.reader_env_image = DataReader(
            self.subscriber_env_image, self.topic_env_image
        )

        self.warning_step_msg_send = False

        self.warning_no_joint_states_dt = 0.1
        self.warning_no_joint_states_send = False

        self.warning_no_env_states_dt = 0.1
        self.warning_no_env_states_send = False

        self.warning_no_env_image_dt = 0.2
        self.warning_no_env_image_send = 0.2

        self.max_wait_time_first_msg = 20
        self.server_process = None

        if len(create_server_cmd):
            print("Starting server with command: ", " ".join(create_server_cmd))
            self.server_process = subprocess.Popen(
                create_server_cmd, env=os.environ.copy()
            )
            time.sleep(0.01)

        # create a smiluation in another process
        self.last_state = None
        self.last_env_state = None
        self.last_img = None

        tic = time.time()
        self.time_last_state = tic
        self.time_last_env_state = tic
        self.time_last_img = tic

        tic = time.time()
        print("waiting to receive the first message from the robot...")
        while True:
            if self.is_ready():
                print("Robot is ready!")
                break
            if time.time() - tic > self.max_wait_time_first_msg:
                raise Exception("Robot is not ready! -- Start the server first!")

    def is_ready(self):
        return self.get_robot_state() is not None

    def set_cmd(self, cmd):
        """ """
        self.writer.write(cmd)

    def close(self):
        """ """
        if self.server_process is not None:
            print("closing the server")
            self.server_process.terminate()  # Terminate the process
            self.server_process.wait()  # Wait for the process to fully close

    def get_env_image(self):
        tic = time.time()
        msg = get_last_msg(self.reader_env_image, EnvImage)
        if msg:
            np_array = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            frame = cv2.imdecode(np_array, cv2.IMREAD_COLOR)
            self.last_img = frame
            self.time_last_img = tic
            self.warning_no_env_image_send = False
        else:
            if (
                tic - self.time_last_img > self.warning_no_env_image_dt
                and not self.warning_no_env_image_send
            ):
                print(
                    f"warning: client did not recieve env image in  last {self.warning_no_env_image_dt} [s]"
                )
                self.warning_no_env_image_send = True
        return self.last_img

    def get_env_state(self):
        """ """
        tic = time.time()
        msg = get_last_msg(self.reader_env, EnvState)
        if msg:
            self.last_env_state = dict(zip(msg.names, msg.poses))
            self.time_last_env_state = tic
            self.warning_no_joint_states_send = False
        else:
            if (
                tic - self.time_last_env_state > self.warning_no_env_states_dt
                and not self.warning_no_joint_states_send
            ):
                print(
                    f"warning: client did not recieve env states in  last {self.warning_no_joint_states_dt} [s]"
                )
                self.warning_no_env_states_send = True
        return self.last_env_state

    def get_robot_state(self):
        """ """
        msg = get_last_msg(self.reader, FlexivState)
        if msg:
            self.last_state = msg
            self.time_last_state = time.time()
            self.warning_no_joint_states_send = False
            return self.last_state

        else:
            tic = time.time()
            if (
                tic - self.time_last_state > self.warning_no_joint_states_dt
                and not self.warning_no_joint_states_send
            ):
                print(
                    f"warning: client did not recieve joint states in  last {self.warning_no_joint_states_dt} [s]"
                )
                self.warning_no_joint_states_send = True
            return self.last_state
