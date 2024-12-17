import numpy as np
import time
from FlexivPy.robot.dds.subscriber import get_last_msg
import cv2
from cyclonedds.domain import DomainParticipant
from cyclonedds.topic import Topic
from cyclonedds.sub import Subscriber, DataReader
import time

from cyclonedds.domain import DomainParticipant
from cyclonedds.topic import Topic
from FlexivPy.robot.dds.flexiv_messages import EnvImage
from datetime import datetime
import matplotlib.pyplot as plt


class CameraDDSClient:
    def __init__(self):

        self.domain_participant = DomainParticipant(42)

        self.topic_env_image = Topic(self.domain_participant, "EnvImage", EnvImage)
        self.subscriber_env_image = Subscriber(self.domain_participant)
        self.reader_env_image = DataReader(
            self.subscriber_env_image, self.topic_env_image
        )

        self.warning_no_env_image_dt = 0.2
        self.warning_no_env_image_send = 0.2

        self.max_wait_time_first_msg = 2

        # create a smiluation in another process
        self.last_img = None

        tic = time.time()

        self.time_last_img = tic

        tic = time.time()
        print("waiting to receive the image ...")
        while True:
            if self.is_ready():
                print("Camera is ready!")
                break
            if time.time() - tic > self.max_wait_time_first_msg:
                raise Exception("Camera is not ready! -- Start the server first!")
            time.sleep(0.1)

    def is_ready(self):
        return self.get_env_image() is not None

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
                self.warning_no_env_image_send = False

        return self.last_img
