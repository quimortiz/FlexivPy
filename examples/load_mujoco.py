from FlexivPy.robot.model.model_robot import FlexivModel
import numpy as np
import FlexivPy.robot.sim.sim_robot as sim_robot
import mujoco
import cv2


def view_image(image):
    cv2.imshow(f"tmp", image)
    while True:
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    cv2.destroyWindow("tmp")


q0 = np.array([0, -0.75, 0, 1.5, 0, 0.75, 0])
robot_model = FlexivModel(render=False, q0=q0)

robot = sim_robot.FlexivSim(render=True, q0=q0, pin_model=robot_model.robot)


renderer = mujoco.Renderer(robot.model, 480, 640)
renderer.update_scene(robot.data)

camera_names = ["static_camera"]
for camera in camera_names:
    print(f"camera {camera}")
    renderer.update_scene(robot.data, camera=camera)
    pixels = renderer.render()
    image = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
    view_image(image)


input("Press Enter to continue...")
robot.close()
