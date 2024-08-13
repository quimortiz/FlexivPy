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


class FlexivSim:
    def __init__(
        self,
        render=False,
        dt=0.001,
        xml_path=None,
        q0=None,
        gravity_comp=True,
        kv_damping=1.0,
        pin_model=None,
    ):

        assert pin_model is not None

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
        self.kv_damping = kv_damping

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
            print("q0 is", q0)
            self.reset_state(q0, np.zeros(7))
            if self.render:
                self.viewer.sync()

        print("robot sim is ready!")

    def read_config(self, file):
        with open(file, "r") as stream:
            try:
                self.config = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)

    def reset_state(self, q, dq):
        self.data.qpos = q
        self.data.qvel = dq
        mujoco.mj_forward(self.model, self.data)

    def step(self):
        self.step_counter += 1
        self.set_u()
        mujoco.mj_step(self.model, self.data)
        # Render every render_ds_ratio steps (60Hz GUI update)
        # TODO: is this necessary?
        if self.render and (self.step_counter % self.render_ds_ratio) == 0:
            self.viewer.sync()

    def set_u(self):
        cmd = self.cmd
        tau = (
            cmd["tau_ff"]
            + cmd["kp"] * (cmd["q"] - self.data.qpos)
            + cmd["kv"] * (cmd["dq"] - self.data.qvel)
        )

        if self.gravity_comp:
            tau += pin.computeGeneralizedGravity(
                self.pin_model.model, self.pin_model.data, self.data.qpos
            )

        if self.kv_damping > 0:
            tau -= self.kv_damping * self.data.qvel

        self.data.ctrl = tau

    def set_cmd(self, cmd):
        """
        cmd: dictionary with keys 'q', 'dq', 'kp', 'kv', 'tau_ff'
        """
        self.cmd = cmd

    def getJointStates(self):
        return {"q": self.data.qpos, "dq": self.data.qvel}

    def is_ready(self):
        return self.getJointStates() is not None

    def getPose(self):
        return self.data.qpos[:3], self.data.qpos[3:7]

    def close(self):
        if self.render:
            self.viewer.close()