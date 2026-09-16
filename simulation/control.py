"""Centroidal wrench tracking with constrained contact force allocation.

All feet remain in stance. This is a reduced-order whole-body controller,
not a full inverse-dynamics QP or a locomotion policy.
"""
import mujoco
import numpy as np
from scipy.optimize import minimize, LinearConstraint, Bounds
from .model import LEGS, HOME_HEIGHT


def skew(v):
    x, y, z = v
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])


def reference(t: float):
    # Smooth two-second transitions, a low inspection hold, then return home.
    low = .34
    if t < 3:
        return HOME_HEIGHT, 0., "Stabilize"
    if t < 5:
        u = (t - 3) / 2
        s, ds = 3*u*u - 2*u*u*u, 3*u - 3*u*u
        return HOME_HEIGHT + (low - HOME_HEIGHT)*s, (low - HOME_HEIGHT)*ds, "Lower sensor"
    if t < 8:
        return low, 0., "Inspect + reject push"
    if t < 10:
        u = (t - 8) / 2
        s, ds = 3*u*u - 2*u*u*u, 3*u - 3*u*u
        return low + (HOME_HEIGHT - low)*s, (HOME_HEIGHT - low)*ds, "Rise"
    return HOME_HEIGHT, 0., "Recover + hold"


def allocate_forces(feet, com, wrench, previous, mu=.55):
    """Convex QP with unilateral loads and a conservative friction pyramid.

    |fx| + |fy| <= mu*fz fits inside the circular Coulomb cone.
    The objective trades off wrench tracking and force regularization.
    """
    A = np.vstack([np.tile(np.eye(3), (1, 4)), np.hstack([skew(p-com) for p in feet])])
    weights = np.diag([1., 1., 1., 4., 4., 2.])
    B, target = weights @ A, weights @ wrench
    H = B.T @ B + .001*np.eye(12)
    g = -B.T @ target
    C = np.zeros((16, 12))
    for i in range(4):
        C[4*i:4*i+4, 3*i:3*i+3] = [[1,1,-mu],[1,-1,-mu],[-1,1,-mu],[-1,-1,-mu]]
    bounds = Bounds(np.tile([-120., -120., 0.], 4), np.tile([120., 120., 180.], 4))
    # The unconstrained optimum is also the QP optimum when it is feasible.
    # Most stance steps take this exact fast path; SLSQP handles active limits.
    unconstrained = np.linalg.solve(H, -g)
    if np.all(C @ unconstrained <= 0) and np.all(unconstrained >= bounds.lb) and np.all(unconstrained <= bounds.ub):
        return unconstrained, True
    result = minimize(lambda f: .5*f @ H @ f + g @ f, previous,
                      jac=lambda f: H @ f + g, method="SLSQP", bounds=bounds,
                      constraints=[LinearConstraint(C, -np.inf, 0)],
                      options={"ftol": 1e-6, "maxiter": 60})
    feasible = np.all(C @ result.x < 1e-4) and np.all(result.x[2::3] >= -1e-6)
    return (result.x if result.success and feasible else previous.copy()), bool(result.success and feasible)


class StanceController:
    def __init__(self, model, data, mode="whole_body"):
        self.model, self.mode = model, mode
        self.sites = [model.site(f"{leg}_toe").id for leg in LEGS]
        self.trunk = model.body("trunk").id
        self.mass = float(model.body_mass.sum())
        self.home_q = data.qpos[7:].copy()
        self.home_com_xy = data.subtree_com[self.trunk, :2].copy()
        self.forces = np.tile([0., 0., self.mass * 9.81 / 4], 4)
        self.failures = 0
        self.saturated = 0
        self.calls = 0
        self.jac = np.zeros((3, model.nv))
        self.jac_com = np.zeros_like(self.jac)

    def step(self, data):
        m = self.model
        height, vz, _ = reference(data.time)
        desired_q = self.home_q.copy()
        bend = np.arccos(np.clip((height-.026)/.5, .1, .99))
        desired_q[1::3], desired_q[2::3] = -bend, 2*bend
        if self.mode == "joint_pd":
            # Same reference and torque limit, but no body feedback/force allocation.
            torque = 90*(desired_q-data.qpos[7:]) - 5*data.qvel[6:]
        else:
            R = data.xmat[self.trunk].reshape(3, 3)
            com = data.subtree_com[self.trunk].copy()
            mujoco.mj_jacSubtreeCom(m, data, self.jac_com, self.trunk)
            vel = self.jac_com @ data.qvel
            position_error = np.r_[self.home_com_xy-com[:2], height-data.qpos[2]]
            accel = np.array([55, 55, 160])*position_error + np.array([14, 14, 25])*(np.array([0, 0, vz])-vel)
            force = self.mass * (accel + [0, 0, 9.81])
            # Rotation error expressed in world coordinates; freejoint angular velocity is local.
            error = -.5*np.array([R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R[0,1]])
            moment = np.array([180, 230, 100])*error - np.array([18, 22, 12])*(R @ data.qvel[3:6])
            feet = data.site_xpos[self.sites].copy()
            self.forces, success = allocate_forces(feet, com, np.r_[force, moment], self.forces)
            self.failures += int(not success)
            # M*qdd + bias = tau + J^T*f. Quasistatic feedforward neglects M*qdd.
            torque = data.qfrc_bias[6:].copy()
            for i, site in enumerate(self.sites):
                mujoco.mj_jacSite(m, data, self.jac, None, site)
                torque -= self.jac[:, 6:].T @ self.forces[3*i:3*i+3]
            torque += 8*(desired_q-data.qpos[7:]) - 1.2*data.qvel[6:]
        self.saturated += int(np.any(np.abs(torque) > 32))
        self.calls += 1
        data.ctrl[:] = np.clip(torque, -32, 32)
