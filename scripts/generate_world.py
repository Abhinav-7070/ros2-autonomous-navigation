#!/usr/bin/env python3

import math
import os

# ======================== TRACK PARAMETERS ========================
L = 18.0
R = 7.0
LANE_W = 3.0
WALL_OFF = 1.0
LINE_W = 0.12
LINE_GAP = 0.12
WALL_T, WALL_H = 0.15, 0.5
DS = 0.6
SPAWN_S = 3.0
GOAL_SPACING = 8.0
MIN_GOAL_CLEARANCE = 2.0

OBSTACLES = [
    (12.0, +0.7, "barrel"),
    (17.5, -0.7, "cone"),
    (23.0, +0.7, "tyre"),
    (28.5, -0.7, "barrel"),
    (34.0, +0.7, "cone"),
    (39.5, -0.7, "tyre"),
    (45.0, +0.7, "barrel"),
    (50.5, -0.7, "cone"),
    (56.0, +0.7, "tyre"),
    (61.5, -0.7, "barrel"),
    (67.0, +0.7, "cone"),
    (72.5, -0.7, "tyre"),
    (77.0, +0.7, "barrel"),
    (81.0, -0.7, "cone"),
]


OBST_RADIUS = {"barrel": 0.30, "tyre": 0.35, "cone": 0.15}
ARC = math.pi * R
TOTAL = 2 * L + 2 * ARC


def pose(s, offset=0.0):
    """Return x, y, heading at arc length s."""
    s %= TOTAL

    if s < L:
        x, y, h = -L / 2 + s, -R, 0.0
    elif s < L + ARC:
        th = -math.pi / 2 + (s - L) / R
        x, y, h = L / 2 + R * math.cos(th), R * math.sin(th), th + math.pi / 2
    elif s < 2 * L + ARC:
        x, y, h = L / 2 - (s - L - ARC), R, math.pi
    else:
        th = math.pi / 2 + (s - 2 * L - ARC) / R
        x, y, h = -L / 2 + R * math.cos(th), R * math.sin(th), th + math.pi / 2

    return x - offset * math.sin(h), y + offset * math.cos(h), h


def loop_boxes(offset, z):
    """Return poses and lengths for boxes following the loop."""
    n = math.ceil(TOTAL / DS)
    pts = [pose(i * TOTAL / n, offset)[:2] for i in range(n)]
    boxes = []

    for i, (x0, y0) in enumerate(pts):
        x1, y1 = pts[(i + 1) % n]
        length = math.hypot(x1 - x0, y1 - y0) * 1.04
        yaw = math.atan2(y1 - y0, x1 - x0)
        boxes.append(((x0 + x1) / 2, (y0 + y1) / 2, z, yaw, length))

    return boxes


def box_xml(name, b, width, height, material, collision=False):
    x, y, z, yaw, length = b
    p = f"{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.4f}"
    size = f"{length:.3f} {width:.3f} {height:.3f}"

    xml = (
        f'<visual name="{name}"><pose>{p}</pose>'
        f'<geometry><box><size>{size}</size></box></geometry>'
        f'<material><script>'
        f'<uri>file://media/materials/scripts/gazebo.material</uri>'
        f'<name>{material}</name></script></material></visual>\n'
    )

    if collision:
        xml += (
            f'<collision name="{name}_collision"><pose>{p}</pose>'
            f'<geometry><box><size>{size}</size></box></geometry></collision>\n'
        )

    return xml


def cylinder_xml(name, radius, height, z, material, collision=False):
    geom = (
        f'<geometry><cylinder><radius>{radius}</radius>'
        f'<length>{height}</length></cylinder></geometry>'
    )
    xml = (
        f'<visual name="{name}"><pose>0 0 {z} 0 0 0</pose>{geom}'
        f'<material><script>'
        f'<uri>file://media/materials/scripts/gazebo.material</uri>'
        f'<name>{material}</name></script></material></visual>\n'
    )

    if collision:
        xml += (
            f'<collision name="{name}_collision">'
            f'<pose>0 0 {z} 0 0 0</pose>{geom}</collision>\n'
        )

    return xml


def obstacle_xml(index, s, offset, kind):
    x, y, _ = pose(s, offset)
    body = ""

    if kind == "barrel":
        body += cylinder_xml("body", 0.30, 0.90, 0.45, "Gazebo/Orange", True)
        body += cylinder_xml("stripe", 0.305, 0.15, 0.55, "Gazebo/White")
    elif kind == "tyre":
        body += cylinder_xml("tyre_lower", 0.35, 0.25, 0.125, "Gazebo/Black", True)
        body += cylinder_xml("tyre_upper", 0.35, 0.25, 0.375, "Gazebo/Black")
        body += cylinder_xml("hub", 0.15, 0.02, 0.51, "Gazebo/Grey")
    else:
        body += cylinder_xml("base", 0.15, 0.20, 0.10, "Gazebo/Orange", True)
        body += cylinder_xml("middle", 0.11, 0.20, 0.30, "Gazebo/White")
        body += cylinder_xml("top", 0.07, 0.20, 0.50, "Gazebo/Orange")

    return (
        f'<model name="obstacle_{index}_{kind}"><static>true</static>'
        f'<pose>{x:.3f} {y:.3f} 0 0 0 0</pose><link name="link">'
        f'{body}</link></model>\n'
    )


def check_layout():
    clearances = []

    for s, offset, kind in OBSTACLES:
        clearance = LANE_W / 2 + abs(offset) - OBST_RADIUS[kind]
        assert clearance >= 1.2, f"Insufficient clearance at s={s}"
        clearances.append(clearance)

    positions = sorted(s for s, _, _ in OBSTACLES)

    for a, b in zip(positions, positions[1:] + [positions[0] + TOTAL]):
        assert b - a >= 4.0, f"Obstacles at {a} and {b} are too close"

    assert min(s for s, _, _ in OBSTACLES) - SPAWN_S >= 8.0
    return min(clearances)


def make_goals():
    obstacle_positions = [s for s, _, _ in OBSTACLES]
    goals = []
    s = SPAWN_S + GOAL_SPACING

    while s < SPAWN_S + TOTAL - GOAL_SPACING / 2:
        if min(
            abs(((s - o + TOTAL / 2) % TOTAL) - TOTAL / 2)
            for o in obstacle_positions
        ) < MIN_GOAL_CLEARANCE:
            s += 1.0
            continue

        goals.append(s)
        s += GOAL_SPACING

    return goals


def build_world():
    half_x = L / 2 + R + 4
    half_y = R + 4

    world = [
        '<?xml version="1.0"?>',
        '<sdf version="1.6">',
        '<world name="navigation_loop">',
        '<include><uri>model://sun</uri></include>',
        '<physics type="ode"><real_time_update_rate>1000</real_time_update_rate>'
        '<max_step_size>0.001</max_step_size></physics>',
        '<scene><ambient>0.6 0.6 0.6 1</ambient><shadows>false</shadows></scene>',
        '<model name="grass"><static>true</static><link name="link">',
        f'<collision name="ground"><geometry><plane><normal>0 0 1</normal>'
        f'<size>{2 * half_x} {2 * half_y}</size></plane></geometry>'
        '<surface><friction><ode><mu>1</mu><mu2>1</mu2></ode></friction>'
        '</surface></collision>',
        f'<visual name="grass_visual"><geometry><plane><normal>0 0 1</normal>'
        f'<size>{2 * half_x} {2 * half_y}</size></plane></geometry>'
        '<material><script><uri>file://media/materials/scripts/gazebo.material'
        '</uri><name>Gazebo/Grass</name></script></material></visual>',
        '</link></model>',
        '<model name="lane_lines"><static>true</static><link name="link">',
    ]

    for edge in (+LANE_W / 2, -LANE_W / 2):
        for d in (+(LINE_W + LINE_GAP) / 2, -(LINE_W + LINE_GAP) / 2):
            for i, b in enumerate(loop_boxes(edge + d, 0.004)):
                world.append(box_xml(
                    f"line_{edge}_{d}_{i}", b, LINE_W, 0.008, "Gazebo/White"
                ))

    world.extend([
        '</link></model>',
        '<model name="boundary_walls"><static>true</static><link name="link">',
    ])

    wall_offset = LANE_W / 2 + LINE_W + LINE_GAP + WALL_OFF

    for side, offset in (("inner", wall_offset), ("outer", -wall_offset)):
        for i, b in enumerate(loop_boxes(offset, WALL_H / 2)):
            world.append(box_xml(
                f"{side}_{i}", b, WALL_T, WALL_H, "Gazebo/Wood", True
            ))

    world.append('</link></model>')

    for i, (s, offset, kind) in enumerate(OBSTACLES):
        world.append(obstacle_xml(i, s, offset, kind))

    world.extend(['</world>', '</sdf>'])
    return "\n".join(world) + "\n"


def build_goals():
    sx, sy, sh = pose(SPAWN_S)
    c, sn = math.cos(sh), math.sin(sh)

    lines = [
        "# Waypoints relative to the intended robot spawn pose.",
        "# Verify the Cartographer map origin before using these goals.",
        "frame_id: map",
        "goals:",
    ]

    for s in make_goals():
        x, y, heading = pose(s)
        dx, dy = x - sx, y - sy
        mx = c * dx + sn * dy
        my = -sn * dx + c * dy
        yaw = math.atan2(math.sin(heading - sh), math.cos(heading - sh))
        lines.append(f"  - {{x: {mx:.2f}, y: {my:.2f}, yaw: {yaw:.3f}}}")

    return "\n".join(lines) + "\n"


if __name__ == "__main__":
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    clearance = check_layout()

    os.makedirs(os.path.join(root, "worlds"), exist_ok=True)
    os.makedirs(os.path.join(root, "config"), exist_ok=True)

    world_path = os.path.join(root, "worlds", "navigation_loop.world")
    goals_path = os.path.join(root, "config", "goals.yaml")

    with open(world_path, "w", encoding="utf-8") as f:
        f.write(build_world())

    with open(goals_path, "w", encoding="utf-8") as f:
        f.write(build_goals())

    x, y, yaw = pose(SPAWN_S)
    print(f"Track length: {TOTAL:.2f} m")
    print(f"Minimum calculated obstacle clearance: {clearance:.2f} m")
    print(f"Intended spawn pose: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")
    print(f"Generated: {world_path}")
    print(f"Generated: {goals_path}")
