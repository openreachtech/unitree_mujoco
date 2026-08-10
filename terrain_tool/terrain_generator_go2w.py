"""Generate unitree_robots/go2w/scene_terrain.xml -- walls only, for the Go2W policies.

Run from this directory:

    python3 terrain_generator_go2w.py

Kept separate from terrain_generator.py so that running either one can never touch the
other robot's scene_terrain.xml. TerrainGenerator itself is imported rather than copied,
so there is still a single implementation of the geometry helpers.
"""

import terrain_generator as tg_mod
from terrain_generator import TerrainGenerator

# terrain_generator.py's own scene.xml is go2-only: it declares <mujoco model="go2 scene">
# and includes go2.xml plus height_scan_mocap.xml, and that mocap file exists only under
# unitree_robots/go2/. Starting Go2W from it would emit a scene that loads the go2 model
# and then fails on the missing include, so the input here is Go2W's own scene.xml.
ROBOT = "go2w"
INPUT_SCENE_PATH = "../unitree_robots/go2w/scene.xml"
OUTPUT_SCENE_PATH = "../unitree_robots/go2w/scene_terrain.xml"

# (x, thickness) -- thickness varies along x within a lane.
THICKNESS_LANES = ((-3.0, 0.30), (-4.5, 0.10))
# One y lane per height, so a lane is a single difficulty.
HEIGHTS = (0.50, 0.60, 0.70)
LANE_SPACING = 1.5
WALL_LENGTH = 1.0


def add_walls(tg):
    """Six walls in a 3 x 2 grid: height along y, thickness along x.

    Driving a lane head-on means crossing the 0.30 m wall first and the 0.10 m one 1.5 m
    later at the same height. That spacing leaves roughly 1.3 m of clear ground between
    their faces -- about two Go2W body lengths -- so the robot can settle before the
    second one.

    No AddFloatingWall: the wheeled base is meant to climb onto and over a wall, so a
    hovering plate with no supporting wall underneath is not a case worth testing here.

    AddWall's argument names, restated because "width" is easy to misread: `width` is the
    wall's *thickness* along x (what the robot has to get across), `length` is its extent
    along y, and `height` is how tall it stands.
    """
    for i, height in enumerate(HEIGHTS):
        lane_y = i * LANE_SPACING
        for x, thickness in THICKNESS_LANES:
            tg.AddWall(init_pos=[x, lane_y, 0.0],
                       yaw=0.0,
                       width=thickness,
                       height=height,
                       length=WALL_LENGTH)


if __name__ == "__main__":
    # TerrainGenerator reads INPUT_SCENE_PATH in __init__ and OUTPUT_SCENE_PATH in Save()
    # from its *own* module globals, so point those at Go2W before constructing it.
    tg_mod.ROBOT = ROBOT
    tg_mod.INPUT_SCENE_PATH = INPUT_SCENE_PATH
    tg_mod.OUTPUT_SCENE_PATH = OUTPUT_SCENE_PATH

    tg = TerrainGenerator()
    add_walls(tg)
    tg.Save()
    print(f"{ROBOT}: wrote {OUTPUT_SCENE_PATH} (from {INPUT_SCENE_PATH})")
