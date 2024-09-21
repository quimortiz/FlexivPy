# This viewer will subscribe to the dds topic and display the robot state online.
import FlexivPy.robot.model.pinocchio as pinocchio
import numpy as np
import time


from cyclonedds.domain import DomainParticipant
from cyclonedds.topic import Topic
from cyclonedds.sub import Subscriber, DataReader
import time

from cyclonedds.domain import DomainParticipant
from cyclonedds.topic import Topic
from FlexivPy.robot.dds.flexiv_messages import FlexivState


domain_participant = DomainParticipant()
topic_state = Topic(domain_participant, "FlexivState", FlexivState)
subscriber = Subscriber(domain_participant)
reader = DataReader(subscriber, topic_state)

robot_model = pinocchio.FlexivModel(render=True)

# TODO: wrap this in a nice class that servers as example for other users
# E.g. timeoutime, Hz, ...

stop_dt = 100.0  # if i don't receive a cmd in this time, stop the robot
hz = 60
dt = 1 / hz
max_time = 100

wait_time = 10
start_wait = time.time()
receiving_data = False
print("waiting for data...")
while time.time() - start_wait < wait_time:
    last_sample = reader.take()
    if len(last_sample) and type(last_sample[0]) == FlexivState:
        receiving_data = True
        break
    time.sleep(0.01)

if not receiving_data:
    raise Exception("No state received in {} seconds".format(wait_time))
print("data received")

time_last_msg = time.time()
time_start = time.time()

while time.time() - time_start < max_time:

    if time.time() - time_last_msg > stop_dt:
        raise Exception(f"No state received in {stop_dt} seconds")

    last_msg = reader.take()

    if last_msg:
        while True:
            a = reader.take()
            if not a:
                break
            else:
                last_msg = a
    if last_msg:
        state = last_msg[0]
        if type(state) == FlexivState:
            time_last_msg = time.time()
            robot_model.display(np.array(state.q))

    time.sleep(dt)
