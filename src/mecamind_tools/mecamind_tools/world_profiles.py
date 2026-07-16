from dataclasses import dataclass
from typing import List


@dataclass(frozen=True)
class RoomSpec:
    name: str
    center_x: float
    center_y: float
    width: float
    depth: float


@dataclass(frozen=True)
class ObstacleSpec:
    name: str
    x: float
    y: float
    width: float
    depth: float
    height: float


@dataclass(frozen=True)
class WorldBlueprint:
    name: str
    outer_width: float
    outer_depth: float
    rooms: List[RoomSpec]
    obstacles: List[ObstacleSpec]


def three_room_blueprint() -> WorldBlueprint:
    rooms = [
        RoomSpec("entry_hall", -2.8, -1.8, 2.4, 2.2),
        RoomSpec("living_room", 0.0, 0.4, 3.4, 2.6),
        RoomSpec("bedroom", 2.5, -1.7, 2.2, 2.0),
    ]
    obstacles = [
        ObstacleSpec("table", -1.8, 1.2, 0.8, 0.8, 0.35),
        ObstacleSpec("cabinet", 1.9, -0.9, 0.6, 1.0, 0.55),
        ObstacleSpec("pillar", 0.2, -2.3, 0.35, 0.35, 0.8),
        ObstacleSpec("chair", 2.8, 1.0, 0.45, 0.45, 0.45),
    ]
    return WorldBlueprint(
        name="three_room_house",
        outer_width=10.0,
        outer_depth=8.0,
        rooms=rooms,
        obstacles=obstacles,
    )


def summarize_blueprint(blueprint: WorldBlueprint) -> str:
    room_names = ", ".join(room.name for room in blueprint.rooms)
    obstacle_names = ", ".join(obj.name for obj in blueprint.obstacles)
    return (
        f"world={blueprint.name}; "
        f"rooms={len(blueprint.rooms)} [{room_names}]; "
        f"obstacles={len(blueprint.obstacles)} [{obstacle_names}]"
    )


def validate_three_room_blueprint(blueprint: WorldBlueprint) -> bool:
    return len(blueprint.rooms) >= 3 and len(blueprint.obstacles) >= 3
