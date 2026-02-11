import os
from typing import List


def parse_spawn_pos(value: str) -> List[float]:
    parts = (value or "0,0,0").split(",")
    return [float(p.strip()) for p in parts]


def load_map(map_path: str) -> None:
    import omni.isaac.core.utils.stage as stage_utils

    stage_utils.add_reference_to_stage(usd_path=map_path, prim_path="/World/Map")


def spawn_robot(robot_path: str, spawn_pos: List[float], world) -> None:
    import omni.isaac.core.utils.stage as stage_utils
    from omni.isaac.core.robots import Robot

    stage_utils.add_reference_to_stage(usd_path=robot_path, prim_path="/World/E1_Robot")
    world.scene.add(Robot(prim_path="/World/E1_Robot", name="e1_robot", position=spawn_pos))
