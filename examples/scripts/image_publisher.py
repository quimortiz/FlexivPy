import time
from FlexivPy.robot.dds.flexiv_messages import EnvImage
import argparse

ASSETS_PATH = "FlexivPy/assets/"


import cv2

from cyclonedds.domain import DomainParticipant
from cyclonedds.topic import Topic
from cyclonedds.pub import Publisher, DataWriter
import time

from cyclonedds.domain import DomainParticipant
from cyclonedds.topic import Topic
from FlexivPy.robot.dds.flexiv_messages import EnvImage


from datetime import datetime


class ImagePublisher:
    def __init__(self, dt=0.05, camera_id=0):
        self.domain_participant = DomainParticipant()
        self.topic_state_image = Topic(self.domain_participant, "EnvImage", EnvImage)
        self.publisher_image = Publisher(self.domain_participant)
        self.writer_image = DataWriter(self.publisher_image, self.topic_state_image)
        self.dt = dt
        self.camera_id = camera_id
        self.cap = cv2.VideoCapture(self.camera_id)

    def run(self):
        tic_start = time.time()
        while True:
            tic = time.time()
            ret, frame = self.cap.read()
            if not ret:
                print("failed to read frame")

            # Convert frame to a byte array
            _, buffer = cv2.imencode(".jpg", frame)
            image_bytes = buffer.tobytes()

            # Create an ImageData object and publish it
            now = datetime.now()  # TODO: avoid calling tic twice
            timestamp = now.strftime("%Y-%m-%d %H:%M:%S.%f")[:-4]
            image_data = EnvImage(data=image_bytes, timestamp=timestamp)
            self.writer_image.write(image_data)

            # Wait to maintain the frame rate
            elapsed_time = time.time() - tic
            time.sleep(max(0, self.dt - elapsed_time))

    def close(self):
        self.cap.release()


if __name__ == "__main__":

    argsp = argparse.ArgumentParser()
    argsp.add_argument("--camera_id", type=int, default=0)
    argsp.add_argument("--dt", type=float, default=0.05)
    args = argsp.parse_args()

    image_publisher = ImagePublisher(camera_id=args.camera_id, dt=args.dt)
    try:
        image_publisher.run()
    finally:
        image_publisher.close()
