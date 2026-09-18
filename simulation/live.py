"""Live simulation state. This module is headless and has no network or UI code."""
import math
import mujoco
import numpy as np
from .locomotion import CrawlController, rotation
from .navigation import make_arena, plan_path, segment_clear, ARENA, OBSTACLES, ROBOT_CLEARANCE
from .model import LEGS


class LiveSimulation:
    def __init__(self):
        self.model, self.data = make_arena()
        self.reset()

    def reset(self):
        # Reset is the only runtime action that overwrites generalized positions.
        _, fresh_data = make_arena()
        mujoco.mj_resetData(self.model,self.data)
        self.data.qpos[:] = fresh_data.qpos
        mujoco.mj_forward(self.model,self.data)
        self.controller = CrawlController(self.model,self.data)
        self.tick = 0
        self.mode = "manual"
        self.paused = False
        self.manual = np.zeros(3)
        self.path = []
        self.waypoint = 0
        self.goal = None
        self.message = "Ready. Drive with W/A/S/D or choose a destination."
        self.trail = []
        self.collision_count = 0

    def stop(self, message="Stopping — placing the swing foot before holding."):
        self.manual[:] = 0
        self.path = []
        self.goal = None
        self.mode = "manual"
        self.message = message

    def command(self, event):
        action = event["action"]
        if action == "drive":
            self.manual = np.array([event["forward"],event["strafe"],event["turn"]])
            self.mode, self.path, self.goal = "manual", [], None
            if np.linalg.norm(self.manual) > 0:
                self.message = "Manual drive · release the controls to hold position."
        elif action == "goal":
            center = self.data.site_xpos[self.controller.sites,:2].mean(axis=0)
            try:
                path = plan_path(center,event["target"])
            except ValueError as error:
                self.message = str(error)
                return
            self.path, self.waypoint, self.goal = path, 1, list(event["target"])
            self.manual[:] = 0
            self.mode = "navigate"
            self.message = "Following a route through the known obstacle map."
        elif action == "stop":
            self.stop()
        elif action == "pause":
            self.paused = event["paused"]
            self.manual[:] = 0
            self.message = "Physics paused." if self.paused else "Physics running."
        elif action == "reset":
            self.reset()

    def navigation_command(self):
        c, d = self.controller, self.data
        R = d.xmat[c.trunk].reshape(3,3)
        yaw = math.atan2(R[1,0],R[0,0])
        center = d.site_xpos[c.sites,:2].mean(axis=0)
        if self.mode == "navigate" and self.path:
            delta = np.array(self.path[self.waypoint])-center
            distance = np.linalg.norm(delta)
            if distance < .10:
                if self.waypoint == len(self.path)-1:
                    self.manual[:] = 0
                    self.mode = "arrived"
                    self.message = "Destination reached. Finishing the step and holding."
                    return np.zeros(3)
                self.waypoint += 1
                delta = np.array(self.path[self.waypoint])-center
                distance = np.linalg.norm(delta)
            desired_yaw = math.atan2(delta[1],delta[0])
            yaw_error = math.atan2(math.sin(desired_yaw-yaw),math.cos(desired_yaw-yaw))
            speed = min(.05,distance/(4*c.PERIOD))
            command = np.r_[delta/max(distance,1e-6)*speed,np.clip(.4*yaw_error,-.06,.06)]
        elif self.mode == "arrived":
            return np.zeros(3)
        else:
            local = np.array([self.manual[0],self.manual[1],0.])
            local /= max(1.,np.linalg.norm(local))
            world = rotation(yaw)@local*c.MAX_SPEED
            command = np.r_[world[:2],self.manual[2]*c.MAX_YAW_RATE]
            # Reduce reach when translating and turning simultaneously.
            if abs(command[2]) > .02:
                command[:2] *= .75
        if np.linalg.norm(command[:2]) > 0:
            # Check an entire foothold advance, not just the next physics step.
            if not segment_clear(center,center+command[:2]*4*c.PERIOD,margin=.55):
                command[:2] = 0
                self.message = "Clearance stop. Steer away from the obstacle or wall."
        return command

    def advance(self, steps=10):
        if self.paused:
            return
        for _ in range(steps):
            if self.tick % 5 == 0:
                self.controller.step(self.data,self.navigation_command())
            mujoco.mj_step(self.model,self.data)
            self.tick += 1
        mujoco.mj_forward(self.model,self.data)
        if not np.isfinite(self.data.qpos).all():
            self.paused = True
            self.message = "Physics fault. Reset the simulation."
        if self.controller.fallen:
            self.mode = "fallen"
            self.message = "Robot lost balance. Reset to try again."
        obstacle_ids = {self.model.geom(f"obstacle_{i}").id for i in range(len(OBSTACLES))}
        for contact in self.data.contact[:self.data.ncon]:
            if contact.geom1 in obstacle_ids or contact.geom2 in obstacle_ids:
                self.collision_count += 1
        if self.tick % 50 == 0:
            self.trail.append(self.data.qpos[:2].tolist())
            self.trail = self.trail[-700:]

    def snapshot(self):
        d,c = self.data,self.controller
        R = d.xmat[c.trunk].reshape(3,3)
        loads = np.zeros(4)
        ids = [self.model.geom(f"{leg}_foot").id for leg in LEGS]
        force = np.zeros(6)
        for i in range(d.ncon):
            contact = d.contact[i]
            for j, foot in enumerate(ids):
                if foot in (contact.geom1,contact.geom2):
                    mujoco.mj_contactForce(self.model,d,i,force)
                    loads[j] += max(0.,force[0])
        return {
            "time": float(d.time), "position": d.qpos[:3].tolist(),
            "center": d.site_xpos[c.sites,:2].mean(axis=0).tolist(),
            "yaw": float(math.atan2(R[1,0],R[0,0])),
            "roll": float(np.degrees(math.atan2(R[2,1],R[2,2]))),
            "pitch": float(np.degrees(np.arcsin(np.clip(-R[2,0],-1,1)))),
            "speed": float(np.linalg.norm(d.qvel[:2])),
            "torque": float(np.max(np.abs(d.ctrl))),
            "loads": loads.tolist(), "supporting": c.supporting.tolist(),
            "phase": c.phase, "mode": self.mode, "paused": self.paused,
            "message": self.message, "path": self.path, "waypoint": self.waypoint,
            "goal": self.goal, "trail": self.trail, "steps": c.completed_steps,
            "solverFailures": c.failures, "collisionSamples": self.collision_count,
            "arena": ARENA, "clearance": ROBOT_CLEARANCE, "obstacles": OBSTACLES,
        }
