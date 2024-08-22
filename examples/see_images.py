from FlexivPy.robot.dds.subscriber import SubscriberNode


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
from FlexivPy.robot.dds.flexiv_messages import  EnvImage

import rerun as rr
from FlexivPy.robot.dds.subscriber import SubscriberNode

import cv2


def view_image(image):
    cv2.imshow(f"tmp-see_images", image)
    while True:
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    cv2.destroyWindow("tmp-see_images")

class ImagePlotter( SubscriberNode ):

    def __init__(self) :

        super().__init__(topic_names=
                         ['EnvImage'],
                         topic_types= [EnvImage],
                         dt=.05,
                         max_time_s=100, 
                         warning_dt=.2,
                         messages_to_keep=1)



    def do(self):
        if len(self.message_queue) == 1:
            msg = self.message_queue[0][0]
            if msg is not None:
                print('message received')
                print(msg)
                np_array = np.frombuffer(bytes(msg.data), dtype=np.uint8)
                frame = cv2.imdecode(np_array, cv2.IMREAD_COLOR)
                cv2.imshow("Frame", frame)
                key = cv2.waitKey(int(1000*self.dt))
                # view_image(frame)





            else:
                print("message is None")
        elif len(self.message_queue) > 1:
            raise ValueError("Too many messages in queue -- we only want one")

        else: 
            print("No messages in queue")


    def close(self):
        cv2.destroyAllWindows()

if __name__ == "__main__" :

    img_plotter = ImagePlotter()
    img_plotter.run()
