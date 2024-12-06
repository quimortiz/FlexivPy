from FlexivPy.vision import RealSenseCamera
import time
from FlexivPy.robot.dds.flexiv_messages import EnvImage

from datetime import datetime
import numpy as np


import cv2

from cyclonedds.domain import DomainParticipant
from cyclonedds.topic import Topic
from cyclonedds.pub import Publisher, DataWriter
import time

from cyclonedds.domain import DomainParticipant
from cyclonedds.topic import Topic
from FlexivPy.robot.dds.flexiv_messages import EnvImage


  

class ImagePublisher:
    def __init__(self, realsense_camera: RealSenseCamera, dt=0.01):
        self.domain_participant = DomainParticipant(10)
        self.topic_state_image = Topic(self.domain_participant, "EnvImage", EnvImage)
        self.publisher_image = Publisher(self.domain_participant)
        self.writer_image = DataWriter(self.publisher_image, self.topic_state_image)
        self.dt = dt
        self.realsense_camera = realsense_camera

    def run(self):
        new_width = 320
        new_height = 240
        while True:
            tic = time.time()

            image = self.realsense_camera.color_frame

            if image is not None:
                image = cv2.resize(
                    image, (new_width, new_height), interpolation=cv2.INTER_LINEAR
                )
                # change from RGB to 
                # Convert frame to a byte array
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                _, buffer = cv2.imencode(".png", image)
                image_bytes = buffer.tobytes()

                # Create an ImageData object and publish it
                now = datetime.now()  # TODO: avoid calling tic twice
                timestamp = now.strftime("%Y-%m-%d %H:%M:%S.%f")[:-4]
                image_data = EnvImage(data=image_bytes, timestamp=timestamp)
                self.writer_image.write(image_data)

                # Wait to maintain the frame rate
            elapsed_time = time.time() - tic
            time.sleep(max(0, self.dt - elapsed_time))


class ImagePublisher2:
    def __init__(self, dt=0.05, camera_id=0):
        self.domain_participant = DomainParticipant(10)
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
    # publish images!
    camera = RealSenseCamera(VGA=True, camera_serial_no="234222302193")
    time.sleep(1)

    class Camera:
        def __init__(self):
            self.color_frame = np.random.randint(
                low=0, high=256, size=(320, 240, 3), dtype=np.uint8
            )

        def close(self):
            pass

    # camera = Camera()

    img_publisher = ImagePublisher(camera)

    try:
        img_publisher.run()
    finally:
        #img_publisher.close()
        camera.close()
