from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ET


@dataclass(frozen=True)
class AxisAlignedBox:
    name: str
    center_x: float
    center_y: float
    width: float
    depth: float
    height: float = 0.0

    @property
    def min_x(self) -> float:
        return self.center_x - self.width * 0.5

    @property
    def max_x(self) -> float:
        return self.center_x + self.width * 0.5

    @property
    def min_y(self) -> float:
        return self.center_y - self.depth * 0.5

    @property
    def max_y(self) -> float:
        return self.center_y + self.depth * 0.5

    def contains(self, x: float, y: float, margin: float = 0.0) -> bool:
        return (
            self.min_x - margin <= x <= self.max_x + margin
            and self.min_y - margin <= y <= self.max_y + margin
        )

    def ray_distance(
        self,
        origin_x: float,
        origin_y: float,
        dx: float,
        dy: float,
        max_range: float,
    ) -> float | None:
        eps = 1e-9

        if abs(dx) < eps:
            if origin_x < self.min_x or origin_x > self.max_x:
                return None
            tx_min = -math.inf
            tx_max = math.inf
        else:
            tx1 = (self.min_x - origin_x) / dx
            tx2 = (self.max_x - origin_x) / dx
            tx_min = min(tx1, tx2)
            tx_max = max(tx1, tx2)

        if abs(dy) < eps:
            if origin_y < self.min_y or origin_y > self.max_y:
                return None
            ty_min = -math.inf
            ty_max = math.inf
        else:
            ty1 = (self.min_y - origin_y) / dy
            ty2 = (self.max_y - origin_y) / dy
            ty_min = min(ty1, ty2)
            ty_max = max(ty1, ty2)

        enter = max(tx_min, ty_min)
        exit = min(tx_max, ty_max)
        if exit < 0.0 or enter > exit or enter > max_range:
            return None
        if enter < 0.0 <= exit:
            return 0.0
        hit = enter if enter >= 0.0 else exit
        if 0.0 <= hit <= max_range:
            return hit
        return None


@dataclass(frozen=True)
class OccupancyGridSpec:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    data: tuple[int, ...]


@dataclass(frozen=True)
class WorldGeometry:
    name: str
    boxes: tuple[AxisAlignedBox, ...]
    min_x: float
    max_x: float
    min_y: float
    max_y: float

    def is_occupied(self, x: float, y: float, margin: float = 0.0) -> bool:
        return any(box.contains(x, y, margin) for box in self.boxes)

    def raycast(self, origin_x: float, origin_y: float, heading: float, max_range: float) -> float:
        dx = math.cos(heading)
        dy = math.sin(heading)
        best = max_range
        for box in self.boxes:
            hit = box.ray_distance(origin_x, origin_y, dx, dy, best)
            if hit is not None and hit < best:
                best = hit
        return best

    def build_occupancy_grid(
        self,
        resolution: float = 0.05,
        padding: float = 0.20,
    ) -> OccupancyGridSpec:
        origin_x = self.min_x - padding
        origin_y = self.min_y - padding
        width = max(1, int(math.ceil((self.max_x - self.min_x + padding * 2.0) / resolution)))
        height = max(1, int(math.ceil((self.max_y - self.min_y + padding * 2.0) / resolution)))
        data: list[int] = []
        for row in range(height):
            y = origin_y + (row + 0.5) * resolution
            for col in range(width):
                x = origin_x + (col + 0.5) * resolution
                data.append(100 if self.is_occupied(x, y) else 0)
        return OccupancyGridSpec(
            width=width,
            height=height,
            resolution=resolution,
            origin_x=origin_x,
            origin_y=origin_y,
            data=tuple(data),
        )


def _parse_pose(pose_text: str | None) -> tuple[float, float]:
    if not pose_text:
        return 0.0, 0.0
    parts = pose_text.split()
    if len(parts) < 2:
        return 0.0, 0.0
    return float(parts[0]), float(parts[1])


def _parse_box_size(link_element: ET.Element) -> tuple[float, float, float] | None:
    size_text = link_element.findtext("collision/geometry/box/size")
    if not size_text:
        size_text = link_element.findtext("visual/geometry/box/size")
    if not size_text:
        return None
    parts = size_text.split()
    if len(parts) < 2:
        return None
    width = float(parts[0])
    depth = float(parts[1])
    height = float(parts[2]) if len(parts) > 2 else 0.0
    return width, depth, height


def load_world_geometry(world_path: str | Path) -> WorldGeometry:
    path = Path(world_path)
    root = ET.parse(path).getroot()
    world_name = root.find("world").get("name", path.stem)

    boxes: list[AxisAlignedBox] = []
    for model in root.findall("world/model"):
        model_name = model.get("name", "model")
        for link in model.findall("link"):
            size = _parse_box_size(link)
            if size is None:
                continue
            width, depth, height = size
            pose_x, pose_y = _parse_pose(link.findtext("pose"))
            link_name = link.get("name", "link")
            boxes.append(
                AxisAlignedBox(
                    name=f"{model_name}/{link_name}",
                    center_x=pose_x,
                    center_y=pose_y,
                    width=width,
                    depth=depth,
                    height=height,
                )
            )

    if not boxes:
        raise ValueError(f"No box geometry found in {path}")

    min_x = min(box.min_x for box in boxes)
    max_x = max(box.max_x for box in boxes)
    min_y = min(box.min_y for box in boxes)
    max_y = max(box.max_y for box in boxes)

    return WorldGeometry(
        name=world_name,
        boxes=tuple(boxes),
        min_x=min_x,
        max_x=max_x,
        min_y=min_y,
        max_y=max_y,
    )
