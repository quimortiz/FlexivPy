# create a simulation robot using mujoco


import mujoco
import mujoco.viewer
import numpy as np
import os
import yaml
import pinocchio as pin
import FlexivPy.robot.dds.flexiv_messages as flexiv_messages

from FlexivPy import ASSETS_PATH
import time

import cv2


def view_image(image):
    cv2.imshow(f"tmp sim robot", image)
    while True:
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    cv2.destroyWindow("tmp")


class FlexivMujocoSim:
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
        object_names=[],
        joint_names=None,
        has_gripper=False,
        camera_name = "static_camera", 
        gravity = np.array([0., 0., -9.81])
    ):
        self.dt = dt
        self.camera_name = camera_name
        self.has_gripper = has_gripper
        self.gravity = gravity
        self.render = render
        self.step_counter = 0
        if q0 is not None:
            self.q0 = q0
        else:
            self.q0 = np.array([0.0, -0.698, 0.000, 1.571, -0.000, 0.698, -0.000])
        # initialize the command with a zero buffer
        self.cmd = flexiv_messages.FlexivCmd()
        # Get all the joint_names if the joint list is not provided by the user
        if joint_names is None:
            self.joint_names = [
                "joint1",
                "joint2",
                "joint3",
                "joint4",
                "joint5",
                "joint6",
                "joint7",
            ]
        else:
            self.joint_names = joint_names

        self.kv_damping = kv_damping
        self.object_names = object_names
        self.CV2 = None
        self.gravity_comp = gravity_comp

        # Load a default scene if the scene XML is not provided by the
        if xml_path is None:
            self.model = mujoco.MjModel.from_xml_path(
                os.path.join(ASSETS_PATH, "mjmodel.xml")
            )
        else:
            self.model = mujoco.MjModel.from_xml_path(xml_path)
        
        self.data = mujoco.MjData(self.model)    
        self.model.opt.gravity = self.gravity.squeeze()
        self.model.opt.timestep = self.dt    
        # Enable camera simulation?
        self.last_camera_image = None
        if render_images:
            import cv2
            self.CV2 = cv2
            # TODO: adpat this code to include more camera if desired!
            self.camera_renderer = mujoco.Renderer(self.model, 480, 640)
            self.camera_renderer.update_scene(self.data, camera=self.camera_name)
            pixels = self.camera_renderer.render()
            self.last_camera_image = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR).copy()
            self.camera_render_dt = 1.0 / 10.0
            self.camera_render_ds_ratio = max(1, self.camera_render_dt // dt)
            print("camera_render_ds_ratio", self.camera_render_ds_ratio)
        else:
            self.camera_renderer = None
        # Enable live rendering?
        if self.render:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            _render_dt = 1.0 / 30.0
            self.render_ds_ratio = max(1, _render_dt // dt)
        
        # Load a default piochhio model if it's not provided by the user
        if pin_model is not None:
            self.pin_model = pin_model
            try:
                # lets check that the model is correct!
                pin.computeGeneralizedGravity(
                    self.pin_model.model, self.pin_model.data, self.q0
                )
            except:
                assert False, 'The provided Pinocchio model is not right.'
        else:
            urdf = os.path.join(ASSETS_PATH, "flexiv_rizon10s_kinematics.urdf")
            meshes_dir = os.path.join(ASSETS_PATH, "meshes")
            from pinocchio.robot_wrapper import RobotWrapper
            self.pin_model = RobotWrapper.BuildFromURDF(urdf, meshes_dir)

        self.reset_state_robot(self.q0, np.zeros(7))
        # Step once to update the visualizer
        self.step()
        if self.render:
            self.viewer.sync()
        print("robot sim is ready!")

    def get_gravity(self):
        return pin.computeGeneralizedGravity(
                self.pin_model.model, self.pin_model.data, self.get_robot_joints()
            )

    def get_robot_joints(self):
        return np.array([self.data.joint(j).qpos[0] for j in self.joint_names])

    def get_robot_vel(self):
        return np.array([self.data.joint(j).qvel[0] for j in self.joint_names])

    def get_gripper_state(self):
        if self.data.ctrl[-1] == 255:
            if (
                self.data.joint("2f_85_left_driver_joint").qpos > 0.75
                and self.data.joint("2f_85_right_driver_joint").qpos > 0.75
                and self.data.joint("2f_85_left_driver_joint").qvel < 0.5
                and self.data.joint("2f_85_right_driver_joint").qvel < 0.5
            ):
                return "closed"
            elif (
                self.data.joint("2f_85_left_driver_joint").qpos > 0.1
                and self.data.joint("2f_85_right_driver_joint").qpos > 0.1
                and self.data.joint("2f_85_left_driver_joint").qvel < 0.5
                and self.data.joint("2f_85_right_driver_joint").qvel < 0.5
            ):
                return "holding"

            else:
                return "open"
        else:
            return "open"

    def set_robot_joints(self, q):
        for i, j in enumerate(self.joint_names):
            self.data.joint(j).qpos[0] = q[i]

    def set_robot_vel(self, dq):
        for i, j in enumerate(self.joint_names):
            self.data.joint(j).qvel[0] = dq[i]

    def read_config(self, file):
        with open(file, "r") as stream:
            try:
                self.config = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)

    def reset_state_robot(self, q, dq):
        self.step_counter = 0
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

        if (
            self.camera_renderer
            and (self.step_counter % self.camera_render_ds_ratio) == 0
        ):
            self.camera_renderer.update_scene(self.data, camera=self.camera_name)
            pixels = self.camera_renderer.render()
            self.last_camera_image = self.CV2.cvtColor(
                pixels, self.CV2.COLOR_RGB2BGR
            ).copy()
            # # lets show the image
            # self.CV2.imshow("image in sim robot", self.last_camera_image)
            # input('heere is the rendered image')
            # view_image(self.last_camera_image)
            # time.sleep(0.0001)

    def set_u(self):
        cmd = self.cmd
        tau = (
            np.array(cmd.tau_ff)
            + np.array(cmd.kp) * (np.array(cmd.q) - self.get_robot_joints())
            + np.array(cmd.kv) * (np.array(cmd.dq) - self.get_robot_vel())
        )

        if not cmd.tau_ff_with_gravity:
            tau += self.get_gravity()

        if self.kv_damping > 0:
            tau -= self.kv_damping * self.get_robot_vel()

        if self.has_gripper:
            if cmd.g_cmd == "close":
                tau = np.append(tau, 255)
            else:
                tau = np.append(tau, 0)

        self.data.ctrl = tau

    def set_cmd(self, cmd):
        """
        cmd: dictionary with keys 'q', 'dq', 'kp', 'kv', 'tau_ff'
        """
        self.cmd = cmd

    def get_robot_tau(self):
        return self.data.ctrl[:7]

    def get_robot_state(self):
        return flexiv_messages.FlexivState(
            q=self.get_robot_joints(),
            dq=self.get_robot_vel(),
            g_state=self.get_gripper_state(),
            tau=self.get_robot_tau(),
            g_moving=False,
            g_force=-1,
            g_width=-1,
        )

    def get_env_image(self):
        return self.last_camera_image

    def get_env_state(self):
        #
        D = {}
        for obj in self.object_names:
            # q = data.joint("cube_j").qpos
            D[obj] = np.concatenate(
                [self.data.get_body_xpos(obj), self.data.get_body_xquat(obj)]
            )
        return D

    def is_ready(self):
        return self.get_robot_state() is not None

    def getPose(self):
        return self.data.qpos[:3], self.data.qpos[3:7]

    def close(self):
        if self.render:
            self.viewer.sync()
            self.viewer.close()
        # wait for the viewer to close
        time.sleep(0.5)
