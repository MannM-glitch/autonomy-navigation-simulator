"""Known-map A* navigation for the slow crawl. World coordinates are metres."""
import heapq
import math
import numpy as np
import mujoco
from .model import model_xml, HOME_HEIGHT, FOOT_RADIUS, LENGTH

ARENA = 2.4
ROBOT_CLEARANCE = .65
OBSTACLES = [
    {"x": .95, "y": .35, "hx": .18, "hy": .28, "height": .45},
    {"x": -.85, "y": .85, "hx": .22, "hy": .18, "height": .32},
    {"x": .30, "y": -1.10, "hx": .38, "hy": .14, "height": .30},
]


def clear(point, margin=ROBOT_CLEARANCE):
    x, y = point
    if max(abs(x),abs(y)) > ARENA-margin:
        return False
    for obstacle in OBSTACLES:
        dx = max(abs(x-obstacle["x"])-obstacle["hx"],0.)
        dy = max(abs(y-obstacle["y"])-obstacle["hy"],0.)
        if math.hypot(dx,dy) <= margin:
            return False
    return True


def segment_clear(start, end, margin=ROBOT_CLEARANCE):
    distance = np.linalg.norm(np.array(end)-start)
    return all(clear(np.array(start)+(np.array(end)-start)*t,margin) for t in np.linspace(0,1,max(2,math.ceil(distance/.025)+1)))


def plan_path(start, goal):
    """8-connected A*, then shortcut only collision-free line segments."""
    start, goal = np.array(start), np.array(goal)
    if not clear(start) or not clear(goal):
        raise ValueError("Choose a point in the clear area, away from obstacles and walls.")
    if segment_clear(start, goal):
        return [start.tolist(), goal.tolist()]
    resolution = .10
    origin = -ARENA
    to_cell = lambda p: tuple(np.round((p-origin)/resolution).astype(int))
    to_point = lambda c: np.array(c)*resolution + origin
    first, last = to_cell(start), to_cell(goal)
    if not segment_clear(start,to_point(first)) or not segment_clear(to_point(last),goal):
        raise ValueError("Destination is too close to the clearance boundary; move it slightly.")
    queue, costs, parent = [(0., first)], {first: 0.}, {}
    while queue:
        _, cell = heapq.heappop(queue)
        if cell == last:
            break
        for dx in (-1,0,1):
            for dy in (-1,0,1):
                if dx == dy == 0:
                    continue
                candidate = (cell[0]+dx, cell[1]+dy)
                p = to_point(candidate)
                if not clear(p) or not segment_clear(to_point(cell),p):
                    continue
                cost = costs[cell]+math.hypot(dx,dy)*resolution
                if cost < costs.get(candidate, float("inf")):
                    costs[candidate], parent[candidate] = cost, cell
                    heapq.heappush(queue,(cost+np.linalg.norm(p-to_point(last)),candidate))
    if last not in costs:
        raise ValueError("No route with enough clearance for all four feet.")
    cells = [last]
    while cells[-1] != first:
        cells.append(parent[cells[-1]])
    points = [start.tolist()] + [to_point(c).tolist() for c in reversed(cells)] + [goal.tolist()]
    simplified = [points[0]]
    i = 0
    while i < len(points)-1:
        j = len(points)-1
        while j > i+1 and not segment_clear(points[i],points[j]):
            j -= 1
        simplified.append(points[j])
        i = j
    return simplified


def make_arena():
    obstacles = []
    for i, o in enumerate(OBSTACLES):
        obstacles.append(f'<geom name="obstacle_{i}" type="box" pos="{o["x"]} {o["y"]} {o["height"]/2}" size="{o["hx"]} {o["hy"]} {o["height"]/2}" rgba=".28 .37 .40 1"/>')
    for x,y,sx,sy in ((ARENA+.05,0,.05,ARENA+.1),(-ARENA-.05,0,.05,ARENA+.1),(0,ARENA+.05,ARENA,.05),(0,-ARENA-.05,ARENA,.05)):
        obstacles.append(f'<geom type="box" pos="{x} {y} .12" size="{sx} {sy} .12" rgba=".23 .32 .35 1"/>')
    xml = model_xml().replace('<worldbody>','<worldbody>'+''.join(obstacles))
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    bend = np.arccos((HOME_HEIGHT-FOOT_RADIUS)/(2*LENGTH))
    data.qpos[7:] = np.tile([0,-bend,2*bend],4)
    mujoco.mj_forward(model,data)
    return model, data
