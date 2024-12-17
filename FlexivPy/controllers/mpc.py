from dataclasses import dataclass
import crocoddyl
import pinocchio
import numpy as np


@dataclass
class Mpc_cfg:
    gripperPose_run: float = 1.0
    gripperPose_terminal: float = 1e3
    qReg: float = 1.0
    vReg: float = 1.0
    uReg: float = 1e-1
    vel_terminal: float = 1e2
    dt: float = 0.05
    horizon: int = 20


class Mpc_generator:
    def __init__(self, robot, x0, Tdes, mpc_cfg: Mpc_cfg = Mpc_cfg()):
        """

        Notes:

        After solving and optimization problem, you can access information with:
        solver.problem.terminalData.differential.multibody.pinocchio.oMf[robot_model.getFrameId("link7")].translation.T,




        """

        self.mpc_cfg = mpc_cfg
        self.robot = robot
        state = crocoddyl.StateMultibody(robot)
        actuation = crocoddyl.ActuationModelFull(state)
        x0 = x0

        nu = 7

        runningCostModel = crocoddyl.CostModelSum(state)
        terminalCostModel = crocoddyl.CostModelSum(state)

        # uResidual = crocoddyl.ResidualModelControlGrav(state, nu)
        # uResidual = crocoddyl.ResidualModelJointEffort(state, nu)
        uResidual = crocoddyl.ResidualModelJointAcceleration(state, nu)
        xResidual = crocoddyl.ResidualModelState(state, x0, nu)

        framePlacementResidual = crocoddyl.ResidualModelFramePlacement(
            state,
            self.robot.getFrameId("link7"),
            pinocchio.SE3(Tdes),
            nu,
        )

        goalTrackingCost = crocoddyl.CostModelResidual(state, framePlacementResidual)
        # xRegCost = crocoddyl.CostModelResidual(state, xResidual)

        velRegCost = crocoddyl.CostModelResidual(
            state,
            crocoddyl.ActivationModelWeightedQuad(
                np.concatenate([np.zeros(7), np.ones(7)])
            ),
            xResidual,
        )

        qRegCost = crocoddyl.CostModelResidual(
            state,
            crocoddyl.ActivationModelWeightedQuad(
                np.concatenate([np.ones(7), np.zeros(7)])
            ),
            xResidual,
        )

        uRegCost = crocoddyl.CostModelResidual(state, uResidual)

        # Then let's added the running and terminal cost functions
        runningCostModel.addCost(
            "gripperPose", goalTrackingCost, mpc_cfg.gripperPose_run
        )
        runningCostModel.addCost("qReg", qRegCost, mpc_cfg.qReg)
        runningCostModel.addCost("velReg", velRegCost, mpc_cfg.vReg)

        runningCostModel.addCost("uReg", uRegCost, mpc_cfg.uReg)
        terminalCostModel.addCost(
            "gripperPose", goalTrackingCost, mpc_cfg.gripperPose_terminal
        )
        # terminalCostModel.addCost("velReg", velRegCost, mpc_cfg.vel_terminal)

        runningModel = crocoddyl.IntegratedActionModelEuler(
            crocoddyl.DifferentialActionModelFreeFwdDynamics(
                state, actuation, runningCostModel
            ),
            mpc_cfg.dt,
        )
        terminalModel = crocoddyl.IntegratedActionModelEuler(
            crocoddyl.DifferentialActionModelFreeFwdDynamics(
                state, actuation, terminalCostModel
            ),
            0.0,
        )

        self.problem = crocoddyl.ShootingProblem(
            x0, [runningModel] * mpc_cfg.horizon, terminalModel
        )
        self.solver = crocoddyl.SolverFDDP(self.problem)

        self.solver.setCallbacks(
            [
                crocoddyl.CallbackVerbose(),
                crocoddyl.CallbackLogger(),
            ]
        )

    def update_x0(self, x0):
        self.solver.problem.x0 = x0

        # for k in range(self.solver.problem.T):
        #     self.solver.problem.runningModels[k].differential.costs.costs[
        #         "xReg"
        #     ].cost.residual.reference = x0

    def update_reg(self, qreg, vreg):
        for k in range(self.solver.problem.T):
            self.solver.problem.runningModels[k].differential.costs.costs[
                "qReg"
            ].cost.residual.reference = qreg
            self.solver.problem.runningModels[k].differential.costs.costs[
                "qReg"
            ].cost.residual.reference = vreg

    def update_Tdes(self, Tdes):
        for k in range(self.solver.problem.T):
            self.solver.prxRegoblem.runningModels[k].differential.costs.costs[
                "gripperPose"
            ].cost.residual.reference = Tdes
        self.problem.terminal_model.differential.costs.costs[
            "gripperPose"
        ].cost.residual.reference = Tdes
