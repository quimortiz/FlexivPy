import matplotlib.pyplot as plt
from collections import deque
import numpy as np
import time
import pathlib


import FlexivPy.robot.model.model_robot as model_robot
import numpy as np
import time
from typing import List


from cyclonedds.domain import DomainParticipant
from cyclonedds.topic import Topic
from cyclonedds.sub import Subscriber, DataReader
import time

from cyclonedds.domain import DomainParticipant
from cyclonedds.topic import Topic
from FlexivPy.robot.dds.flexiv_messages import FlexivState, FlexivCmd



import math


import os
from datetime import datetime

# Get current date and time

class BufferedLogger:
    def __init__(self, filename, buffer_size=100):
        self.filename = filename
        self.buffer = []
        self.buffer_size = buffer_size

    def log(self, data):
        self.buffer.append(data)
        # Write to the file when buffer reaches the buffer_size limit
        if len(self.buffer) >= self.buffer_size:
            self.flush()

    def flush(self):
        with open(self.filename, 'a') as f:
            f.write('\n'.join(self.buffer) + '\n')
        self.buffer.clear()

    def close(self):
        # Flush remaining data
        self.flush()



def get_last_msg(reader, topic_type):
    """

    """
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






class SubscriberNode():

    def __init__(self, topic_names: str, topic_types,  dt: float = .1, max_time_s: float = float('inf'), warning_dt : float = .1, messages_to_keep: int =  1000):

        self.topic_names = topic_names
        self.topic_types = topic_types
        self.dt = dt
        self.max_time_s = max_time_s
        self.warning_dt = warning_dt
        self.messages_to_keep = messages_to_keep

        self.domain_participant = DomainParticipant()

        self.subscribers = []
        self.readers = []
        self.message_queue = [ deque(maxlen=self.messages_to_keep)   for _ in range(len(self.topic_names))]


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
        """

        """
        time_start = time.time()
        time_last_msgs  = [time.time() for _ in range(len(self.topic_names))]
        warning_msg_send = [False for _ in range(len(self.topic_names))]
        warning_slow_send = False

        while time.time() - time_start < self.max_time_s:
            tic_loop = time.time()

            for i in range(len(self.topic_names)):
                if tic_loop - time_last_msgs[i] > self.warning_dt:
                    if not warning_msg_send[i]:
                        warning_msg_send[i] = True
                        print(f"No state received in {self.warning_dt} seconds for topic {self.topic_names[i]}")

                msg = get_last_msg(self.readers[i], self.topic_types[i])

                if msg:
                    time_last_msgs[i] = time.time()
                    warning_msg_send[i] = False
                    self.message_queue[i].append(msg)

            toc_loop = time.time()


            self.do()

            if (toc_loop - tic_loop) > self.dt:
                if not warning_slow_send:
                    warning_slow_send = True
                    print('warning: loop time {toc_loop-tic_loop} is greater than dt')

            time.sleep(max(0, self.dt - (toc_loop - tic_loop)))


class QPlotter( SubscriberNode ):

    def __init__(self, topic_names: List[str], topic_types: List[str],  dt: float = 1./20., max_time_s: float = float('inf'), warning_dt : float = .1, messages_to_keep: int =  1000):

        super().__init__(topic_names=topic_names, topic_types=topic_types, dt=dt, max_time_s=max_time_s, warning_dt=warning_dt, messages_to_keep=messages_to_keep)


        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.lines = [self.ax.plot([], [], label=f"q{i}", linestyle='-', marker='o', alpha=.5)[0] for i in range(7)]
        max_t = dt * messages_to_keep
        self.ax.set_xlim(0, 1.1 * max_t) 
        self.ax.set_ylim(-3.2, 3.2)  # Adjust based on the expected range of robot states
        self.ax.legend()


    def do(self):
        if len(self.message_queue[0]):

            x = self.dt * np.arange(len(self.message_queue[0]))
            for i in range(7):
                y = np.array( [msg.q[i] for msg in self.message_queue[0]])
                self.lines[i].set_data(x[:len(y)], y)
            self.ax.draw_artist(self.ax.patch)
            for line in self.lines:
                self.ax.draw_artist(line)
            self.fig.canvas.flush_events()


class QUPlotter( SubscriberNode ):

    def __init__(self, topic_names: List[str], topic_types: List[str],  dt: float = 1./20., max_time_s: float = float('inf'), warning_dt : float = .1, messages_to_keep: int =  1000):

        super().__init__(topic_names=topic_names, topic_types=topic_types, dt=dt, max_time_s=max_time_s, warning_dt=warning_dt, messages_to_keep=messages_to_keep)


        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.lines_q = [self.ax.plot([], [], label=f"q{i}", linestyle='-', marker='o', alpha=.5)[0] for i in range(7)]
        # Create the lines for q* and pair their colors with the corresponding q lines
        self.lines_qstar = []
        for i in range(7):
            line_qstar = self.ax.plot([], [], label=f"q*{i}", linestyle=':', alpha=.5)[0]
            line_qstar.set_color(self.lines_q[i].get_color())  # Pair the color with q line
            self.lines_qstar.append(line_qstar)

        max_t = dt * messages_to_keep
        self.ax.set_xlim(0, 1.1 * max_t) 
        self.ax.set_ylim(-3.2, 3.2)  # Adjust based on the expected range of robot states

        # add legend
        self.ax.legend()



    def do(self):
        if len(self.message_queue[0]):

            x = self.dt * np.arange(len(self.message_queue[0]))
            for i in range(7):
                y = np.array( [msg.q[i] for msg in self.message_queue[0]])
                self.lines_q[i].set_data(x[:len(y)], y)
                y = np.array( [msg.q[i] for msg in self.message_queue[1]])
                self.lines_qstar[i].set_data(x[:len(y)], y)

            self.ax.draw_artist(self.ax.patch)
            for line in self.lines_q:
                self.ax.draw_artist(line)
            for line in self.lines_qstar:
                self.ax.draw_artist(line)
            self.fig.canvas.flush_events()


class Logger( SubscriberNode ):

    def __init__(self, topic_names: List[str], topic_types: List[str],  dt: float = 1./50., max_time_s: float = float('inf'), warning_dt : float = .1):

        super().__init__(topic_names=topic_names, topic_types=topic_types, dt=dt, max_time_s=max_time_s, warning_dt=warning_dt, messages_to_keep=1)

        current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

        pathlib.Path(f"logs/{current_time}").mkdir(parents=True, exist_ok=True)
        filename = [ f"logs/{current_time}/log_{i}.txt" for i in topic_names]

        self.loggers = [BufferedLogger(f) for f in filename]


    def do(self):
        for i, logger in enumerate(self.loggers):
            if len(self.message_queue[i]) == 1:
                msg = self.message_queue[i][0]
                logger.log(str(msg))
            elif len(self.message_queue[i])>1:
                raise Exception("Logger should only keep one message")

    def close(self):
        for logger in self.loggers:
            logger.close()







if __name__ == "__main__":

    # plotter = QPlotter(topic_names= ["FlexivState"] , messages_to_keep=100,  topic_types=[FlexivState])
    # plotter.run()

    logger = Logger(topic_names= ["FlexivState", "FlexivCmd"] , topic_types=[FlexivState, FlexivCmd])
    try:
        logger.run()
    finally:
        logger.close()

