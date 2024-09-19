import os
import numpy as np
import rerun as rr
from pinocchio.robot_wrapper import RobotWrapper
import FlexivPy.robot.vis.rerunio as rerunio

from FlexivPy.robot.dds.flexiv_messages import FlexivState, FlexivCmd, EnvImage
import cv2
from FlexivPy.robot.dds.subscriber import SubscriberNode


class Rerun_full_log(SubscriberNode):
    def __init__(self):
        prefix = "Flexiv"

        ASSETS_PATH = "FlexivPy/assets/"

        urdf = os.path.join(ASSETS_PATH, "flexiv_rizon10s_kinematics.urdf")
        meshes_dir = os.path.join(ASSETS_PATH, "meshes")

        robot = RobotWrapper.BuildFromURDF(urdf, meshes_dir)

        self.urdf_logger = rerunio.Robot_logger_rerunio(robot, prefix)

        q0 = np.array([0.000, -0.698, 0.000, 1.571, -0.000, 0.698, -0.000])
        self.urdf_logger.log(q=q0, log_meshes=True)

        super().__init__(
            topic_names=["FlexivState", "FlexivCmd", "EnvImage"],
            topic_types=[FlexivState, FlexivCmd, EnvImage],
            dt=0.05,
            max_time_s=100,
            warning_dt=0.2,
            messages_to_keep=1,
        )

    def do(self):
        if len(self.message_queue) == 1:
            if self.message_queue[0][0]:
                self.urdf_logger.log(
                    q=np.array(self.message_queue[0][0].q), log_meshes=False
                )
                for i in range(7):
                    rr.log(f"joint/q{i}", rr.Scalar(self.message_queue[0][0].q[i]))

            if self.message_queue[0][1]:
                for i in range(7):
                    rr.log(
                        f"ctrl/tau_ff{i}", rr.Scalar(self.message_queue[0][1].tau_ff[i])
                    )

            if self.message_queue[0][2]:
                np_array = np.frombuffer(
                    bytes(self.message_queue[0][2].data), dtype=np.uint8
                )
                frame = cv2.imdecode(np_array, cv2.IMREAD_COLOR)
                rr.log("image", rr.Image(frame))

            else:
                print("message is None")
        elif len(self.message_queue) > 1:
            raise ValueError("Too many messages in queue -- we only want one")

        else:
            print("No messages in queue")

    def close(self):
        pass


if __name__ == "__main__":
    rr.init("Visualize Robot!", spawn=True)
    full_log = Rerun_full_log()
    full_log.run()
