# create a simulation robot using mujoco


import time
import mujoco
import mujoco.viewer
import numpy as np
from scipy.spatial.transform import Rotation
import os
import yaml
import pinocchio as pin

ASSETS_PATH = "FlexivPy/assets/"


import cv2

def view_image(image):
    cv2.imshow(f"tmp sim robot", image)
    while True:
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    cv2.destroyWindow("tmp")





class FlexivSim:
    def __init__(
        self,
        render=False,
        dt=0.001,
        xml_path=None,
        q0=None,
        gravity_comp=True,
        kv_damping=0.01,
        pin_model=None,
        render_images=False,
        object_names = [],
        joints = None

    ):

        if joints is None:
            self.joints = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
        else: 
            self.joints = joints


        assert pin_model is not None
        self.CV2 = None

        if xml_path is None:
            self.model = mujoco.MjModel.from_xml_path(
                os.path.join(ASSETS_PATH, "mjmodel.xml")
            )
        else:
            self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.simulated = True
        self.data = mujoco.MjData(self.model)
        self.dt = dt
        _render_dt = 1.0 / 30.0
        self.render_ds_ratio = max(1, _render_dt // dt)
        self.camera_render_dt = 1.0 / 10.
        self.camera_render_ds_ratio = max(1, self.camera_render_dt // dt)
        print("camera_render_ds_ratio", self.camera_render_ds_ratio)

        self.kv_damping = kv_damping
        self.object_names = object_names

        if render:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self.render = True
        else:
            self.render = False

        self.model.opt.gravity[2] = -9.81
        self.model.opt.timestep = dt
        self.renderer = None
        self.render = render
        self.step_counter = 0
        self.cmd = None
        self.pin_model = pin_model

        self.gravity_comp = gravity_comp

        # lets check that the model is correct!
        pin.computeGeneralizedGravity(
            self.pin_model.model, self.pin_model.data, np.zeros(7)
        )

        # self.reset()
        mujoco.mj_forward(self.model, self.data)
        if self.render:
            self.viewer.sync()

        self.q0 = q0
        if self.q0 is not None:
            self.reset_state_robot(q0, np.zeros(7))
            if self.render:
                self.viewer.sync()

        self.last_camera_image = None
        if render_images:
            import cv2
            self.CV2 = cv2
            # TODO: adpat this code to include more camera if desired!
            self.camera_renderer = mujoco.Renderer(self.model, 480, 640)
            self.camera_renderer.update_scene(self.data)
            pixels = self.camera_renderer.render()
            self.last_camera_image = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR).copy()
        else:
            self.camera_renderer = None


        print("robot sim is ready!")


    def get_robot_joints(self):
        return np.array([self.data.joint(j).qpos[0] for j in self.joints])


    def get_robot_vel(self):
        return np.array([self.data.joint(j).qvel[0] for j in self.joints])
    
    def get_gripper_state(self):
        if self.data.ctrl[-1] == 255:
            if self.data.joint('2f_85_left_driver_joint').qpos > 0.75 and self.data.joint('2f_85_right_driver_joint').qpos > 0.75 and \
                self.data.joint('2f_85_left_driver_joint').qvel < 0.5 and self.data.joint('2f_85_right_driver_joint').qvel < 0.5:
                    return 'closed'
            elif self.data.joint('2f_85_left_driver_joint').qpos > 0.1 and self.data.joint('2f_85_right_driver_joint').qpos > 0.1 and \
                self.data.joint('2f_85_left_driver_joint').qvel < 0.5 and self.data.joint('2f_85_right_driver_joint').qvel < 0.5:
                return 'holding'

            else:
                return 'open'
        else:
            return 'open'

    def set_robot_joints(self, q):
        for i, j in enumerate(self.joints):
            self.data.joint(j).qpos[0] = q[i]

    def set_robot_vel(self, dq):
        for i, j in enumerate(self.joints):
            self.data.joint(j).qvel[0] = dq[i]


    def read_config(self, file):
        with open(file, "r") as stream:
            try:
                self.config = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)

    def reset_state_robot(self, q, dq):
        self.set_robot_joints(q)
        self.set_robot_vel(dq)
        mujoco.mj_forward(self.model, self.data)

    def step(self):
        self.step_counter += 1
        self.set_u()
        mujoco.mj_step(self.model, self.data)
        # Render every render_ds_ratio steps (60Hz GUI update)
        # TODO: is this necessary?
        if self.render and (self.step_counter % self.render_ds_ratio) == 0:
            self.viewer.sync()

        if self.camera_renderer and (self.step_counter % self.camera_render_ds_ratio ) == 0:
            self.camera_renderer.update_scene(self.data, camera="static_camera")
            pixels = self.camera_renderer.render()
            self.last_camera_image =  self.CV2.cvtColor(pixels, self.CV2.COLOR_RGB2BGR).copy()
            # # lets show the image
            # self.CV2.imshow("image in sim robot", self.last_camera_image)
            # input('heere is the rendered image')
            # view_image(self.last_camera_image)
            # time.sleep(0.0001)




    def set_u(self):
        cmd = self.cmd
        tau = (
            cmd["tau_ff"]
            + cmd["kp"] * (cmd["q"] - self.get_robot_joints())
            + cmd["kv"] * (cmd["dq"] - self.get_robot_vel())
        )

        if self.gravity_comp:
            tau += pin.computeGeneralizedGravity(
                self.pin_model.model, self.pin_model.data, 
                self.get_robot_joints())

        if self.kv_damping > 0:
            tau -= self.kv_damping * self.get_robot_vel()

        if cmd['g_cmd'] == 'close':
            tau = np.append(tau, 255)
        else:
            tau = np.append(tau, 0)

        self.data.ctrl = tau

    def set_cmd(self, cmd):
        """
        cmd: dictionary with keys 'q', 'dq', 'kp', 'kv', 'tau_ff'
        """
        self.cmd = cmd

    def get_robot_state(self):
        return {"q": self.get_robot_joints(), "dq": self.get_robot_vel(), "g_state": self.get_gripper_state()}

    def get_env_image(self):
        return self.last_camera_image
        
    def get_env_state(self):
        # 
        D = {}
        for obj in self.object_names:
            # q = data.joint("cube_j").qpos
            D[obj] = np.concatenate([self.data.get_body_xpos(obj), self.data.get_body_xquat(obj)])
        return D


    def is_ready(self):
        return self.get_robot_state() is not None

    def getPose(self):
        return self.data.qpos[:3], self.data.qpos[3:7]

    def close(self):
        if self.render:
            self.viewer.close()
