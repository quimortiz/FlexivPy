from collections import deque
import time
import time
from typing import List
from cyclonedds.domain import DomainParticipant
from cyclonedds.topic import Topic
from cyclonedds.sub import Subscriber, DataReader
import time
from cyclonedds.domain import DomainParticipant
from cyclonedds.topic import Topic
from copy import deepcopy


def get_last_msg(reader, topic_type):
    """ """
    last_msg = reader.take()

    if last_msg:
        while True:
            a = reader.take()
            if not a:
                break
            else:
                last_msg = a
    if last_msg:
        msg = last_msg[0]
        if type(msg) == topic_type:
            return msg
        else:
            return None

    else:
        return None


class SubscriberNode:
    def __init__(
        self,
        topic_names: List[str],
        topic_types,
        dt: float = 0.1,
        max_time_s: float = float("inf"),
        warning_dt: float = 0.1,
        messages_to_keep: int = 1000,
        repeat_last_msg_if_not_new=True,
    ):

        self.repeat_last_msg_if_not_new = repeat_last_msg_if_not_new

        if len(topic_names) != len(topic_types):
            raise ValueError("topic_names and topic_types must have the same length")

        self.topic_names = topic_names
        self.topic_types = topic_types
        self.dt = dt
        self.max_time_s = max_time_s
        self.warning_dt = warning_dt
        self.messages_to_keep = messages_to_keep

        self.domain_participant = DomainParticipant()

        self.subscribers = []
        self.readers = []
        self.message_queue = deque(maxlen=self.messages_to_keep)

        self.message_queue.append([None for _ in range(len(self.topic_names))])

        for topic_name, topic_type in zip(self.topic_names, self.topic_types):
            topic = Topic(self.domain_participant, topic_name, topic_type)
            subscriber = Subscriber(self.domain_participant)
            reader = DataReader(subscriber, topic)
            self.subscribers.append(subscriber)
            self.readers.append(reader)

    def do(self):
        """
        Here you can do something with the messages!
        """
        raise NotImplementedError("You need to implement this method in child class")

    def run(self):
        """ """
        time_start = time.time()
        time_last_msgs = [time.time() for _ in range(len(self.topic_names))]
        warning_msg_send = [False for _ in range(len(self.topic_names))]
        warning_slow_send = False

        while time.time() - time_start < self.max_time_s:
            tic_loop = time.time()

            all_msgs = []
            for i in range(len(self.topic_names)):
                if tic_loop - time_last_msgs[i] > self.warning_dt:
                    if not warning_msg_send[i]:
                        warning_msg_send[i] = True
                        print(
                            f"No state received in {self.warning_dt} seconds for topic {self.topic_names[i]}"
                        )

                msg = get_last_msg(self.readers[i], self.topic_types[i])

                if msg:
                    time_last_msgs[i] = time.time()
                    warning_msg_send[i] = False
                    all_msgs.append(msg)
                else:
                    if self.repeat_last_msg_if_not_new:
                        old_msg = deepcopy(self.message_queue[-1][i])
                        if old_msg:
                            all_msgs.append(old_msg)
                        else:
                            all_msgs.append(None)
                    else:
                        all_msgs.append(None)
            self.message_queue.append(all_msgs)

            toc_loop = time.time()

            self.do()

            if (toc_loop - tic_loop) > self.dt:
                if not warning_slow_send:
                    warning_slow_send = True
                    print("warning: loop time {toc_loop-tic_loop} is greater than dt")

            time.sleep(max(0, self.dt - (toc_loop - tic_loop)))

    def close(self):
        """ """
