import time
import numpy as np
from datetime import datetime
import time

from cyclonedds.domain import DomainParticipant
from cyclonedds.pub import Publisher, DataWriter
from cyclonedds.topic import Topic

from cyclonedds.domain import DomainParticipant
from cyclonedds.topic import Topic
from cyclonedds.sub import Subscriber, DataReader
import time
from threading import Thread
from FlexivPy.robot.dds.flexiv_messages import (
    FlexivCmd,
    FlexivState,
    EnvImage,
    EnvState)

import cv2

def view_image(image):
    cv2.imshow(f"tmp-async", image)
    while True:
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    cv2.destroyWindow("tmp-async")

class AsyncSimManager:
    def __init__(self, 
                 simulator, 
                 timeout):
        
        self.CV2 = None
        self.dt = simulator.dt
        self.timeout = timeout
        self.domain_participant = DomainParticipant(10)
        self.topic_state = Topic(self.domain_participant, "FlexivState", FlexivState)
        self.publisher = Publisher(self.domain_participant)
        self.writer = DataWriter(self.publisher, self.topic_state)

        self.topic_env = Topic(self.domain_participant, "EnvState", EnvState)
        self.publisher_env = Publisher(self.domain_participant)
        self.writer_env = DataWriter(self.publisher_env, self.topic_env)
        # I also have to create a subscriber to receive the data
        self.topic_cmd = Topic(self.domain_participant, "FlexivCmd", FlexivCmd)
        self.subscriber = Subscriber(self.domain_participant)
        self.reader = DataReader(self.subscriber, self.topic_cmd)
        self.simulator = simulator
        if self.simulator.camera_renderer is not None:
            import cv2
            self.CV2 = cv2
            self.topic_state_image = Topic(
                self.domain_participant, "EnvImage", EnvImage
            )
            self.publisher_image = Publisher(self.domain_participant)
            self.writer_image = DataWriter(self.publisher_image, self.topic_state_image)
        self.stop_dt = 0.15  # [s] if i don't receive a cmd in this time, stop the robot
        self.running = True 
        self.simulation_thread=Thread(target=self.run())
        self.simulation_thread.start()

    def compute_default_command(self):
        return FlexivCmd(
            kv=10 * np.ones(7),
        )

    def run(self):
        time_start = time.time()
        warning_send = False
        good_satus_send = False
        history_dt = []
        warning_dt_send = False

        # we start with the default cmd
        self.simulator.set_cmd(self.compute_default_command())
        time_last_cmd = time.time()

        time_last_img = time.time()

        while (time.time() - time_start < self.timeout) and self.running:

            tic = time.time()
            now = datetime.now()  # TODO: avoid calling tic twice
            timestamp = now.strftime("%Y-%m-%d %H:%M:%S.%f")[:-4]

            if tic - time_last_cmd > self.stop_dt:
                time_last_cmd = time.time()
                self.simulator.set_cmd(self.compute_default_command())
                if not warning_send:
                    print("no cmd recieved in time! -- using default cmd")
                    warning_send = True
                    good_satus_send = False

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
                self.simulator.set_cmd(cmd)
                # Modify

            self.simulator.step()

            state = self.simulator.get_robot_state()
            msg_out = state
            self.writer.write(msg_out)

            env_state = self.simulator.get_env_state()
            msg_out = EnvState(
                names=list(env_state.keys()),
                poses=list(env_state.values()),
                timestamp=timestamp,
            )

            self.writer_env.write(msg_out)

            if self.simulator.camera_renderer is not None:
                if tic - time_last_img > self.simulator.camera_render_dt:
                    if self.simulator.last_camera_image is not None:
                        _, buffer = self.CV2.imencode(
                            ".jpg", self.simulator.last_camera_image
                        )
                        image_bytes = buffer.tobytes()
                        # Create an ImageData object and publish it
                        now = datetime.now()
                        timestamp = now.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                        image_data = EnvImage(data=image_bytes, timestamp=timestamp)
                        self.writer_image.write(image_data)
                        time_last_img = tic

                    else:
                        print("no image to write!")

            toc = time.time()
            elapsed_time = toc - tic
            if elapsed_time > self.dt:
                if not warning_dt_send:
                    warning_dt_send = True
                    print(
                        f"warning! elapsed time:  {elapsed_time} is greater than dt: {self.dt}"
                    )

            time.sleep(max(0, self.dt - elapsed_time))

    def terminate(self):
        self.running = False
        self.simulation_thread.join()
        self.simulator.close()
        time.sleep(0.5)