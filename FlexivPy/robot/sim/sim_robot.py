# create a simulation robot using mujoco


import time
import mujoco
import mujoco.viewer
import numpy as np
from scipy.spatial.transform import Rotation
import os
import yaml
import pinocchio as pin
import FlexivPy.robot.dds.flexiv_messages as flexiv_messages

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
        joints = None,
        has_gripper=False,
        disturbances_path = None

    ):

        self.has_gripper = has_gripper
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

        if not disturbances_path is None:
            self.disturbances = load_yaml_disturbances(disturbances_path)
        else:
            self.disturbances = []
        
        print("robot sim is ready!")


    def get_robot_joints(self):
        return np.array([self.data.joint(j).qpos[0] for j in self.joints])


    def get_robot_vel(self):
        return np.array([self.data.joint(j).qvel[0] for j in self.joints])
    
    def get_gripper_state(self):
        if self.data.ctrl[-1] == 255:
            if self.data.joint('robot_0_2f_85_left_driver_joint').qpos > 0.75 and self.data.joint('robot_0_2f_85_right_driver_joint').qpos > 0.75 and \
                self.data.joint('robot_0_2f_85_left_driver_joint').qvel < 0.5 and self.data.joint('robot_0_2f_85_right_driver_joint').qvel < 0.5:
                    return 'closed'
            elif self.data.joint('robot_0_2f_85_left_driver_joint').qpos > 0.1 and self.data.joint('robot_0_2f_85_right_driver_joint').qpos > 0.1 and \
                self.data.joint('robot_0_2f_85_left_driver_joint').qvel < 0.5 and self.data.joint('robot_0_2f_85_right_driver_joint').qvel < 0.5:
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

        if self.disturbances:
            disturbance_time, object_name, placement, is_relative = self.disturbances[0]
            if self.data.time >= disturbance_time:
                self.update_object_placement(object_name, placement, is_relative) 
                self.disturbances =  self.disturbances[1:]

    def set_u(self):
        cmd = self.cmd
        tau =  (
            np.array(cmd.tau_ff)
            + np.array(cmd.kp) * (np.array(cmd.q) - self.get_robot_joints())
            + np.array(cmd.kv) * (np.array(cmd.dq) - self.get_robot_vel()) )

        if not cmd.tau_ff_with_gravity:
            tau += pin.computeGeneralizedGravity(
                self.pin_model.model, self.pin_model.data, 
                self.get_robot_joints())

        if self.kv_damping > 0:
            tau -= self.kv_damping * self.get_robot_vel()

        if self.has_gripper:
            if cmd.g_cmd == 'close':
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
            q = self.get_robot_joints(),
            dq = self.get_robot_vel(),
            g_state = self.get_gripper_state(),
            tau = self.get_robot_tau(),
            g_moving = False,
                g_force= -1,
                g_width= -1 )



    def get_env_image(self):
        return self.last_camera_image
        
    def get_env_state(self):
        object_placement_dict = {}
        for object_name in self.object_names:
            object_trans = self.data.joint(object_name + "_joint").qpos[:3]
            object_quat_pin = np.concatenate([self.data.joint(object_name + "_joint").qpos[4:], [self.data.joint(object_name + "_joint").qpos[3]]])
            object_placement_dict[object_name] = np.concatenate([object_trans, object_quat_pin])
        return object_placement_dict


    def is_ready(self):
        return self.get_robot_state() is not None

    def getPose(self):
        return self.data.qpos[:3], self.data.qpos[3:7]

    def close(self):
        if self.render:
            self.viewer.close()

    def update_object_placement(self, object_name: str, placement: pin.SE3, is_relative: bool):
        """
        Update the placement of an object in the environment.

        Parameters
        ----------
        object_name : str
            The name of the object whose placement is to be updated.
        placement : pinocchio.SE3
            The desired placement of the object.

        Raises
        ------
        ValueError
            If the object specified by object_name is not found in the simulator.
        """
        if is_relative:
            object_trans = self.data.joint(object_name + "_joint").qpos[:3]
            object_quat_pin = pin.Quaternion(np.concatenate([self.data.joint(object_name + "_joint").qpos[4:], [self.data.joint(object_name + "_joint").qpos[3]]]))
            current_object_placement_pin = pin.SE3(object_quat_pin, object_trans )
            new_object_placement_pin = current_object_placement_pin * placement
        else:
            new_object_placement_pin = placement

        object_joint_data = self.data.joint(object_name + "_joint")

        object_joint_data.qpos[:3] = new_object_placement_pin.translation

        quat = pin.Quaternion(new_object_placement_pin.rotation).coeffs()
        object_joint_data.qpos[3] = quat[3]
        object_joint_data.qpos[4:] = quat[:3]

        mujoco.mj_forward(self.model, self.data)

def load_yaml_disturbances(disturbances_path: str):

    with open(disturbances_path, 'r') as file:
        disturbances_yaml = yaml.safe_load(file)

    disturbances_yaml = disturbances_yaml.get('disturbances', [])
    disturbances = []
    for disturbance in disturbances_yaml:
        object_name = disturbance.get('object_name')
        time = disturbance.get('time')
        translation = np.array(disturbance.get('translation'), dtype=float)
        rotation = pin.utils.rpyToMatrix(np.array(disturbance.get('rotation'), dtype=float))
        placement = pin.SE3(rotation, translation)
        is_relative = disturbance.get('is_relative')

        disturbances.append((time, object_name, placement, is_relative))

    return disturbances