import time
import mujoco
import mujoco.viewer
import numpy as np
from scipy.spatial.transform import Rotation
import os

from cyclonedds.core import Listener, Qos, Policy
from cyclonedds.domain import DomainParticipant
from cyclonedds.topic import Topic
from cyclonedds.sub import Subscriber, DataReader
from cyclonedds.util import duration
import time

from cyclonedds.core import Qos, Policy
from cyclonedds.domain import DomainParticipant
from cyclonedds.pub import Publisher, DataWriter
from cyclonedds.topic import Topic
from cyclonedds.util import duration
from datetime import datetime


import subprocess
import time

from FlexivPy.robot.dds.flexiv_messages import FlexivCmd, FlexivState, EnvState, EnvImage
from FlexivPy.robot.dds.subscriber import get_last_msg
import cv2


class Flexiv_client:
    def __init__(
        self, dt=0.001, render=False, render_images=True, create_sim_server=False, server_config_file="",
        xml_path=None, urdf=None, meshes_dir=None, joints=None, 
        base_path="", gravity_comp=True, has_gripper=False

    ):

        self.dt = dt
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
        self.reader_env_image = DataReader(self.subscriber_env_image, self.topic_env_image)

        self.warning_step_msg_send = False

        self.warning_no_joint_states = .1
        self.warning_no_env_states = .1
        self.warning_no_env_image = .2

        self.create_sim_server = create_sim_server
        self.server_process = None

        if self.create_sim_server:

            cmd = ["python", base_path +  "FlexivPy/robot/sim/sim_robot_async.py"]
            if render:
                cmd += ["--render"]
            if server_config_file:
                cmd += ["--config", server_config_file]
            if render_images:
                cmd += ["--render_images"]
            if xml_path:
                cmd += ["--xml_path", xml_path]
            if urdf:
                cmd += ["--urdf", urdf]
            if meshes_dir:
                cmd += ["--meshes_dir", meshes_dir]
            if joints:
                cmd += ["--joints"] + joints

            if not gravity_comp:
                cmd += ["--no_gravity_comp"]
            if has_gripper:
                cmd += ["--has_gripper"]

            print("starting the server")
            print(cmd)
            self.server_process = subprocess.Popen(cmd, env=os.environ.copy())

            time.sleep(0.01)

        # create a smiluation in another process
        self.last_state = None
        self.last_env_state = None
        self.last_img = None

        tic = time.time()
        self.time_last_state = tic
        self.time_last_env_state = tic
        self.time_last_img = tic




        print("waiting for robot to be ready...")
        while not self.is_ready():
            time.sleep(0.05)
        print("robot is ready!")

    def is_ready(self):
        return self.get_robot_state() is not None

    def set_cmd(self, cmd):
        """ """
        # create the dds message
        # Get the current time
        now = datetime.now()

        # Format the time as a string with up to milliseconds
        timestamp_str = now.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

        # print("Timestamp with milliseconds:", timestamp_str)
        msg_out = FlexivCmd(
            tau_ff=cmd["tau_ff"],
            q=cmd["q"],
            dq=cmd["dq"],
            kp=cmd["kp"],
            kv=cmd["kv"],
            g_cmd=cmd.get("g_cmd",""),
            timestamp=timestamp_str,
            mode=cmd["mode"],
        )

        self.writer.write(msg_out)

    def step(self):
        """ """
        if not self.warning_step_msg_send:
            self.warning_step_msg_send = True
            print(
                "WARNING: In the client the step runs asynchronusly! \n Wee keep the function here to use same interface!"
            )

    def close(self):
        """ """
        print("closing the robot!")
        if self.create_sim_server:
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
        else:
            if tic - self.time_last_img > self.warning_no_env_image:
                pass
                # print(f"warning: client did not recieve env image in  last {self.warning_no_joint_states} [s]")
        return self.last_img


        

    def get_env_state(self):
        """
        """
        tic = time.time()
        msg = get_last_msg(self.reader_env, EnvState)
        if msg:
            self.last_env_state = dict(zip(msg.names, msg.poses))
            self.time_last_env_state = tic
        else:
            if tic - self.time_last_env_state > self.warning_no_env_states:
                pass
                # print(f"warning: client did not recieve env states in  last {self.warning_no_joint_states} [s]")
        return self.last_env_state


    def get_robot_state(self):
        """ 
        """
        msg = get_last_msg(self.reader, FlexivState)
        if msg:
            self.last_state = {"q": np.array(msg.q), "dq": np.array(msg.dq),
                               "tau": np.array(msg.tau), 
                               "ft_sensor": np.array(msg.ft_sensor),
                               "timestamp": msg.timestamp,
                                "g_state" : msg.g_state,
                               "g_moving" : msg.g_moving,
                               "g_force": msg.g_force,
                               "g_width": msg.g_width }
            self.time_last_state = time.time()
            return self.last_state

        else:
            tic = time.time()
            if tic - self.time_last_state > self.warning_no_joint_states:
                print(f"warning: client did not recieve joint states in  last {self.warning_no_joint_states} [s]")
            return self.last_state
