"""Slow, torque-driven crawl with a support shift before each swing.

The gait trades speed for a three-foot support polygon. No base state is
overwritten and no external stabilizing force is applied.
"""
import mujoco
import numpy as np
from scipy.optimize import Bounds, LinearConstraint, minimize
from .control import skew
from .model import LEGS, FOOT_RADIUS


def rotation(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.]])


def smooth(u):
    u = np.clip(u, 0., 1.)
    return u*u*(3-2*u)


def contact_forces(feet, com, wrench, supporting):
    ids = np.flatnonzero(supporting)
    n = len(ids)
    A = np.vstack([np.tile(np.eye(3), (1, n)), np.hstack([skew(feet[i]-com) for i in ids])])
    W = np.diag([1., 1., 1., 4., 4., 2.])
    B, w = W @ A, W @ wrench
    H, g = B.T @ B + .002*np.eye(3*n), -B.T @ w
    C = np.zeros((4*n, 3*n))
    for i in range(n):
        C[4*i:4*i+4, 3*i:3*i+3] = [[1,1,-.55],[1,-1,-.55],[-1,1,-.55],[-1,-1,-.55]]
    bounds = Bounds(np.tile([-120.,-120.,0.], n), np.tile([120.,120.,180.], n))
    f = np.linalg.solve(H, -g)
    ok = True
    if not (np.all(C@f <= 0) and np.all(f >= bounds.lb) and np.all(f <= bounds.ub)):
        initial = np.tile([0.,0.,max(0.,wrench[2])/n], n)
        result = minimize(lambda x: .5*x@H@x+g@x, initial, jac=lambda x: H@x+g,
                          method="SLSQP", bounds=bounds,
                          constraints=[LinearConstraint(C, -np.inf, 0)],
                          options={"maxiter": 40, "ftol": 1e-5})
        ok = result.success and np.max(C@result.x) < 1e-4
        f = result.x if ok else initial
    forces = np.zeros((4,3))
    forces[ids] = f.reshape(n,3)
    return forces, ok


class CrawlController:
    PERIOD = .95
    SHIFT = .40
    SWING = .40
    ORDER = (0, 3, 1, 2)
    MAX_SPEED = .06
    MAX_YAW_RATE = .08

    def __init__(self, model, data):
        self.model = model
        self.trunk = model.body("trunk").id
        self.sites = [model.site(f"{leg}_toe").id for leg in LEGS]
        self.mass = model.body_mass.sum()
        self.anchors = data.site_xpos[self.sites].copy()
        self.center = self.anchors.mean(axis=0)
        self.offsets = self.anchors-self.center
        self.hold_xy = data.subtree_com[self.trunk,:2].copy()
        self.target_xy = self.hold_xy.copy()
        self.start_xy = self.hold_xy.copy()
        self.target_yaw = 0.
        self.height = .395
        self.phase = "Standing"
        self.start_time = None
        self.step_number = 0
        self.swing_leg = 0
        self.swing_start = self.anchors[0].copy()
        self.swing_end = self.anchors[0].copy()
        self.supporting = np.ones(4, dtype=bool)
        self.jac = np.zeros((3,model.nv))
        self.jac_com = np.zeros_like(self.jac)
        self.forces = np.zeros((4,3))
        self.failures = 0
        self.completed_steps = 0
        self.fallen = False

    def begin_step(self, data, command):
        self.swing_leg = self.ORDER[self.step_number % 4]
        self.anchors = data.site_xpos[self.sites].copy()
        self.anchors[:,2] = FOOT_RADIUS
        self.start_xy = self.target_xy.copy()
        others = [i for i in range(4) if i != self.swing_leg]
        self.hold_xy = self.anchors[others,:2].mean(axis=0)
        self.swing_start = self.anchors[self.swing_leg].copy()
        # Each foot advances once every four steps.
        advance = np.r_[command[:2] * 4*self.PERIOD, 0.]
        yaw_advance = command[2]*4*self.PERIOD
        center = self.anchors.mean(axis=0)
        offset = self.anchors[self.swing_leg]-center
        self.swing_end = center + rotation(yaw_advance)@offset + advance
        self.swing_end[2] = FOOT_RADIUS
        self.step_yaw = self.target_yaw + command[2]*self.PERIOD
        self.start_time = data.time

    def step(self, data, command):
        command = np.array(command, dtype=float)
        speed = np.linalg.norm(command[:2])
        if speed > self.MAX_SPEED:
            command[:2] *= self.MAX_SPEED/speed
        command[2] = np.clip(command[2], -self.MAX_YAW_RATE, self.MAX_YAW_RATE)
        R = data.xmat[self.trunk].reshape(3,3)
        if data.qpos[2] < .22 or R[2,2] < .65:
            self.fallen = True
        if self.fallen:
            self.phase = "Fallen — reset required"
            data.ctrl[:] = np.clip(-2*data.qvel[6:],-32,32)
            return
        moving = np.linalg.norm(command) > .001 and data.time > 1.
        if self.start_time is None and moving:
            self.begin_step(data, command)
        desired_foot, foot_velocity = None, np.zeros(3)
        self.supporting[:] = True
        if self.start_time is not None:
            elapsed = data.time-self.start_time
            self.target_xy = self.start_xy + smooth(elapsed/self.SHIFT)*(self.hold_xy-self.start_xy)
            self.target_yaw += np.clip(self.step_yaw-self.target_yaw, -.001, .001)
            self.phase = f"Shift support · {LEGS[self.swing_leg]}"
            if self.SHIFT <= elapsed < self.SHIFT+self.SWING:
                u = (elapsed-self.SHIFT)/self.SWING
                self.supporting[self.swing_leg] = False
                desired_foot = self.swing_start + smooth(u)*(self.swing_end-self.swing_start)
                desired_foot[2] += .075*np.sin(np.pi*u)**2
                foot_velocity = (6*u-6*u*u)/self.SWING*(self.swing_end-self.swing_start)
                foot_velocity[2] += .075*np.pi*np.sin(2*np.pi*u)/self.SWING
                self.phase = f"Swing · {LEGS[self.swing_leg]}"
            elif elapsed >= self.SHIFT+self.SWING:
                self.phase = "Touchdown"
            if elapsed >= self.PERIOD:
                self.completed_steps += 1
                self.step_number += 1
                self.start_time = None
                if moving:
                    self.begin_step(data, command)
                else:
                    self.start_xy = self.target_xy.copy()
                    self.hold_xy = data.site_xpos[self.sites,:2].mean(axis=0)
        else:
            self.target_xy += np.clip(self.hold_xy-self.target_xy,-.0005,.0005)
            self.phase = "Standing"
        com = data.subtree_com[self.trunk].copy()
        mujoco.mj_jacSubtreeCom(self.model,data,self.jac_com,self.trunk)
        vel = self.jac_com@data.qvel
        accel = np.array([100,100,180])*np.r_[self.target_xy-com[:2],self.height-data.qpos[2]] - np.array([20,20,27])*vel
        force = self.mass*(accel + [0,0,9.81])
        desired_R = rotation(self.target_yaw)
        E = desired_R @ R.T
        error = .5*np.array([E[2,1]-E[1,2], E[0,2]-E[2,0], E[1,0]-E[0,1]])
        moment = np.array([200,260,150])*error - np.array([20,24,16])*(R@data.qvel[3:6])
        feet = data.site_xpos[self.sites]
        self.forces, ok = contact_forces(feet,com,np.r_[force,moment],self.supporting)
        self.failures += int(not ok)
        torque = data.qfrc_bias[6:].copy() - .8*data.qvel[6:]
        for i, site in enumerate(self.sites):
            mujoco.mj_jacSite(self.model,data,self.jac,None,site)
            torque -= self.jac[:,6:].T@self.forces[i]
            if i == self.swing_leg and desired_foot is not None:
                swing_force = 700*(desired_foot-feet[i]) + 18*(foot_velocity-self.jac@data.qvel)
                torque += self.jac[:,6:].T@swing_force
        data.ctrl[:] = np.clip(torque,-32,32)
