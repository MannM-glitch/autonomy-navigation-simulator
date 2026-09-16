"""Original, primitive-only quadruped. Metres, kilograms, radians, seconds."""
from pathlib import Path
import mujoco
import numpy as np

LEGS = ("FL", "FR", "RL", "RR")
HIP_LOCATIONS = np.array([[.25, .18, 0], [.25, -.18, 0], [-.25, .18, 0], [-.25, -.18, 0]])
LENGTH = .25
HOME_HEIGHT = .435
FOOT_RADIUS = .026


def model_xml() -> str:
    legs, motors = [], []
    for name, (x, y, _) in zip(LEGS, HIP_LOCATIONS):
        side = 1 if y > 0 else -1
        legs.append(f'''
        <body name="{name}_hip" pos="{x} {y} 0">
          <joint name="{name}_abduction" axis="1 0 0" range="-.7 .7"/>
          <geom type="capsule" fromto="0 0 0 0 {side*.075} 0" size=".035" mass=".3" rgba=".18 .24 .29 1"/>
          <body name="{name}_thigh" pos="0 {side*.075} 0">
            <joint name="{name}_pitch" axis="0 1 0" range="-1.6 1.6"/>
            <geom type="capsule" fromto="0 0 0 0 0 -.25" size=".026" mass=".55" rgba=".9 .57 .16 1"/>
            <geom type="sphere" size=".045" mass=".05" rgba=".12 .17 .21 1"/>
            <body name="{name}_shin" pos="0 0 -.25">
              <joint name="{name}_knee" axis="0 1 0" range=".1 2.6"/>
              <geom type="capsule" fromto="0 0 0 0 0 -.25" size=".018" mass=".3" rgba=".2 .28 .32 1"/>
              <geom type="sphere" size=".033" mass=".05" rgba=".9 .57 .16 1"/>
              <geom name="{name}_foot" type="sphere" pos="0 0 -.25" size=".026" mass=".08" rgba=".07 .1 .12 1" friction=".8 .02 .002"/>
              <site name="{name}_toe" pos="0 0 -.25" size=".008" rgba=".3 .9 .7 1"/>
            </body>
          </body>
        </body>''')
        for joint in ("abduction", "pitch", "knee"):
            motors.append(f'<motor name="{name}_{joint}_motor" joint="{name}_{joint}" ctrlrange="-32 32"/>')
    return f'''<mujoco model="SENTRY whole-body inspection platform">
      <compiler angle="radian"/>
      <option timestep=".002" integrator="implicitfast" cone="elliptic" iterations="50"/>
      <visual><global offwidth="1280" offheight="720"/><quality shadowsize="4096"/><headlight ambient=".35 .35 .35" diffuse=".7 .7 .7"/></visual>
      <statistic center="0 0 .25" extent="1.4"/>
      <default><joint damping=".25" armature=".025"/><geom solref=".008 1" solimp=".9 .95 .001" condim="3"/></default>
      <asset>
        <texture name="grid" type="2d" builtin="checker" rgb1=".12 .17 .19" rgb2=".16 .21 .23" width="512" height="512"/>
        <material name="ground" texture="grid" texrepeat="12 12" reflectance=".08"/>
      </asset>
      <worldbody>
        <light pos="1 -2 4" dir="-.2 .3 -1" diffuse=".9 .9 .85" castshadow="true"/>
        <light pos="-2 1 3" diffuse=".4 .55 .65" castshadow="false"/>
        <geom name="floor" type="plane" size="5 5 .1" material="ground" friction=".8 .02 .002"/>
        <body name="trunk" pos="0 0 {HOME_HEIGHT}">
          <freejoint/>
          <geom name="chassis" type="box" size=".29 .16 .065" mass="8" rgba=".88 .57 .18 1"/>
          <geom type="box" size=".21 .145 .012" pos="0 0 .075" mass=".4" rgba=".15 .22 .26 1"/>
          <geom type="box" size=".035 .11 .045" pos=".305 0 .015" mass=".3" rgba=".13 .2 .24 1"/>
          <geom type="sphere" size=".017" pos=".34 .065 .025" mass=".01" rgba=".25 .95 .8 1"/>
          <geom type="sphere" size=".017" pos=".34 -.065 .025" mass=".01" rgba=".25 .95 .8 1"/>
          <body name="payload" pos="-.07 0 .1">
            <geom type="cylinder" size=".065 .045" mass="1.45" rgba=".2 .28 .33 1"/>
            <geom type="cylinder" pos="0 0 .046" size=".061 .01" mass=".05" rgba=".25 .9 .75 1"/>
          </body>
          {''.join(legs)}
        </body>
      </worldbody>
      <actuator>{''.join(motors)}</actuator>
    </mujoco>'''


def make_model(payload_kg: float = 1.5, friction: float = .8):
    model = mujoco.MjModel.from_xml_string(model_xml())
    model.body_mass[model.body("payload").id] *= payload_kg / 1.5
    model.body_inertia[model.body("payload").id] *= payload_kg / 1.5
    model.geom_friction[:, 0] = friction
    data = mujoco.MjData(model)
    # Recompute derived constants after the payload edit, before initialization.
    mujoco.mj_setConst(model, data)
    angle = np.arccos((HOME_HEIGHT - FOOT_RADIUS) / (2 * LENGTH))
    data.qpos[7:] = np.tile([0, -angle, 2 * angle], 4)
    mujoco.mj_forward(model, data)
    return model, data


if __name__ == "__main__":
    path = Path(__file__).with_name("sentry.xml")
    path.write_text(model_xml(), encoding="utf-8")
    print(path)
