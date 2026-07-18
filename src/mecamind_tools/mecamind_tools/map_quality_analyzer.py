from __future__ import annotations

import argparse
import json
import math
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import yaml

from .world_geometry import AxisAlignedBox, load_world_geometry


@dataclass(frozen=True)
class OccupancyMap:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    pixels: tuple[int, ...]

    def at(self, row: int, col: int) -> int:
        return self.pixels[row * self.width + col]

    def world_to_cell(self, x: float, y: float) -> tuple[int, int] | None:
        col = int((x - self.origin_x) / self.resolution)
        row = int((y - self.origin_y) / self.resolution)
        if 0 <= row < self.height and 0 <= col < self.width:
            return row, col
        return None

    def cell_to_world(self, row: int, col: int) -> tuple[float, float]:
        return (
            self.origin_x + (col + 0.5) * self.resolution,
            self.origin_y + (row + 0.5) * self.resolution,
        )


def _read_pgm_token(data: bytes, index: int) -> tuple[bytes, int]:
    n = len(data)
    while index < n:
        value = data[index]
        if value == ord("#"):
            while index < n and data[index] not in (10, 13):
                index += 1
        elif value in b" \t\r\n":
            index += 1
        else:
            break

    start = index
    while index < n and data[index] not in b" \t\r\n#":
        index += 1
    return data[start:index], index


def _load_pgm(path: Path) -> tuple[int, int, tuple[int, ...]]:
    data = path.read_bytes()
    magic, index = _read_pgm_token(data, 0)
    width_raw, index = _read_pgm_token(data, index)
    height_raw, index = _read_pgm_token(data, index)
    max_value_raw, index = _read_pgm_token(data, index)

    width = int(width_raw)
    height = int(height_raw)
    max_value = int(max_value_raw)
    if max_value <= 0 or max_value > 65535:
        raise ValueError(f"Unsupported PGM max value: {max_value}")

    if magic == b"P2":
        values = []
        for item in data[index:].split():
            if item.startswith(b"#"):
                continue
            values.append(int(item))
        if len(values) < width * height:
            raise ValueError(f"PGM has too few pixels: {path}")
        return width, height, tuple(values[: width * height])

    if magic == b"P5":
        while index < len(data) and data[index] in b" \t\r\n":
            index += 1
        pixel_count = width * height
        if max_value < 256:
            raw = data[index : index + pixel_count]
            if len(raw) != pixel_count:
                raise ValueError(f"PGM has too few binary pixels: {path}")
            return width, height, tuple(int(v) for v in raw)

        raw = data[index : index + pixel_count * 2]
        if len(raw) != pixel_count * 2:
            raise ValueError(f"PGM has too few 16-bit pixels: {path}")
        values = [
            int.from_bytes(raw[i : i + 2], byteorder="big")
            for i in range(0, len(raw), 2)
        ]
        return width, height, tuple(values)

    raise ValueError(f"Unsupported PGM magic: {magic!r}")


def load_map_yaml(path: str | Path) -> OccupancyMap:
    yaml_path = Path(path).expanduser().resolve()
    with yaml_path.open("r", encoding="utf-8") as stream:
        meta = yaml.safe_load(stream) or {}
    image_path = Path(str(meta["image"]))
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    width, height, pixels = _load_pgm(image_path)
    # map_saver 写出的 PGM 第 0 行对应 y 最大处；这里翻转为第 0 行对应
    # origin（y 最小处），使 world_to_cell 的行号与世界坐标一致。
    flipped: list[int] = []
    for row in range(height - 1, -1, -1):
        flipped.extend(pixels[row * width : (row + 1) * width])
    pixels = tuple(flipped)
    origin = list(meta.get("origin", [0.0, 0.0, 0.0]))
    return OccupancyMap(
        width=width,
        height=height,
        resolution=float(meta.get("resolution", 0.05)),
        origin_x=float(origin[0]),
        origin_y=float(origin[1]),
        pixels=pixels,
    )


def _classify_pixel(value: int) -> str:
    if value <= 100:
        return "occupied"
    if value >= 230:
        return "free"
    return "unknown"


def _counts(grid: OccupancyMap) -> dict[str, int]:
    counts = {"free": 0, "occupied": 0, "unknown": 0}
    for value in grid.pixels:
        counts[_classify_pixel(value)] += 1
    return counts


def _ratio_from_counts(counts: dict[str, int]) -> tuple[float, float]:
    total = sum(counts.values())
    if not total:
        return 0.0, 1.0
    coverage = (counts["free"] + counts["occupied"]) / total
    unknown_ratio = counts["unknown"] / total
    return coverage, unknown_ratio


def _counts_in_world_roi(
    grid: OccupancyMap,
    world_path: str | Path,
    padding: float = 0.12,
) -> tuple[dict[str, int], dict[str, float]]:
    """统计世界内部 ROI 的覆盖情况。

    ROI 向内收缩一个墙厚（padding），只统计机器人有可能观测到的区域；
    外墙外侧永远不可见，算进去会把覆盖率上限压到 90% 以下。
    """
    world = load_world_geometry(world_path)
    min_x = world.min_x + padding
    max_x = world.max_x - padding
    min_y = world.min_y + padding
    max_y = world.max_y - padding
    x_count = max(1, int(math.ceil((max_x - min_x) / grid.resolution)))
    y_count = max(1, int(math.ceil((max_y - min_y) / grid.resolution)))
    counts = {"free": 0, "occupied": 0, "unknown": 0}

    for yi in range(y_count):
        y = min_y + (yi + 0.5) * grid.resolution
        for xi in range(x_count):
            x = min_x + (xi + 0.5) * grid.resolution
            cell = grid.world_to_cell(x, y)
            if cell is None:
                counts["unknown"] += 1
                continue
            counts[_classify_pixel(grid.at(cell[0], cell[1]))] += 1

    return counts, {
        "min_x": round(min_x, 3),
        "max_x": round(max_x, 3),
        "min_y": round(min_y, 3),
        "max_y": round(max_y, 3),
    }


def _neighbors(row: int, col: int) -> Iterable[tuple[int, int]]:
    yield row - 1, col
    yield row + 1, col
    yield row, col - 1
    yield row, col + 1


def _largest_free_component_ratio(grid: OccupancyMap) -> float:
    visited: set[tuple[int, int]] = set()
    total_free = 0
    largest = 0

    for row in range(grid.height):
        for col in range(grid.width):
            if _classify_pixel(grid.at(row, col)) != "free":
                continue
            total_free += 1
            if (row, col) in visited:
                continue
            size = 0
            queue = deque([(row, col)])
            visited.add((row, col))
            while queue:
                r, c = queue.popleft()
                size += 1
                for nr, nc in _neighbors(r, c):
                    if not (0 <= nr < grid.height and 0 <= nc < grid.width):
                        continue
                    if (nr, nc) in visited:
                        continue
                    if _classify_pixel(grid.at(nr, nc)) != "free":
                        continue
                    visited.add((nr, nc))
                    queue.append((nr, nc))
            largest = max(largest, size)

    return largest / total_free if total_free else 0.0


def _occupied_near(grid: OccupancyMap, row: int, col: int, radius: int = 2) -> bool:
    for r in range(max(0, row - radius), min(grid.height, row + radius + 1)):
        for c in range(max(0, col - radius), min(grid.width, col + radius + 1)):
            if _classify_pixel(grid.at(r, c)) == "occupied":
                return True
    return False


def _sample_box(box: AxisAlignedBox, step: float) -> Iterable[tuple[float, float]]:
    x_count = max(2, int(math.ceil(box.width / step)))
    y_count = max(2, int(math.ceil(box.depth / step)))
    for xi in range(x_count + 1):
        x = box.min_x + box.width * xi / x_count
        for yi in range(y_count + 1):
            y = box.min_y + box.depth * yi / y_count
            yield x, y


def _cell_near_boundary(
    grid: OccupancyMap,
    x: float,
    y: float,
    tolerance_cells: int = 2,
) -> tuple[int, int] | None:
    """world_to_cell 的宽松版：允许采样点落在地图边界外不超过 tolerance_cells。

    SLAM 地图通常恰好裁剪在墙体内侧面，墙体外半部分不在栅格范围内，
    这里把边界附近的采样点收敛到最近的边界栅格，避免误判墙体缺失。
    """
    col = int((x - grid.origin_x) / grid.resolution)
    row = int((y - grid.origin_y) / grid.resolution)
    if col < -tolerance_cells or col >= grid.width + tolerance_cells:
        return None
    if row < -tolerance_cells or row >= grid.height + tolerance_cells:
        return None
    return (
        min(max(row, 0), grid.height - 1),
        min(max(col, 0), grid.width - 1),
    )


def _wall_scores_from_world(grid: OccupancyMap, world_path: str | Path | None) -> dict[str, float]:
    if not world_path:
        return {}
    world = load_world_geometry(world_path)
    scores: dict[str, float] = {}
    for side in ("north", "south", "east", "west"):
        wall = next((box for box in world.boxes if f"{side}_wall" in box.name), None)
        if wall is None:
            continue
        total = 0
        hits = 0
        for x, y in _sample_box(wall, grid.resolution):
            total += 1
            cell = _cell_near_boundary(grid, x, y)
            if cell is None:
                continue
            if _occupied_near(grid, cell[0], cell[1]):
                hits += 1
        scores[side] = hits / total if total else 0.0
    return scores


def _edge_scores(grid: OccupancyMap) -> dict[str, float]:
    strip = max(2, int(0.20 / max(grid.resolution, 1.0e-6)))
    sides = {
        "north": [(r, c) for r in range(max(0, grid.height - strip), grid.height) for c in range(grid.width)],
        "south": [(r, c) for r in range(0, min(strip, grid.height)) for c in range(grid.width)],
        "east": [(r, c) for r in range(grid.height) for c in range(max(0, grid.width - strip), grid.width)],
        "west": [(r, c) for r in range(grid.height) for c in range(0, min(strip, grid.width))],
    }
    result = {}
    for side, cells in sides.items():
        occupied = sum(1 for row, col in cells if _classify_pixel(grid.at(row, col)) == "occupied")
        result[side] = occupied / len(cells) if cells else 0.0
    return result


def analyze_map(map_yaml: str | Path, world_path: str | Path | None = None) -> dict[str, object]:
    grid = load_map_yaml(map_yaml)
    raw_counts = _counts(grid)
    raw_coverage, raw_unknown_ratio = _ratio_from_counts(raw_counts)
    roi_bounds = {}
    if world_path:
        counts, roi_bounds = _counts_in_world_roi(grid, world_path)
        coverage_scope = "world_roi"
    else:
        counts = raw_counts
        coverage_scope = "full_map"
    coverage, unknown_ratio = _ratio_from_counts(counts)
    free_component_ratio = _largest_free_component_ratio(grid)
    wall_scores = _wall_scores_from_world(grid, world_path) or _edge_scores(grid)
    missing_walls = [side for side, score in wall_scores.items() if score < 0.35]

    if coverage >= 0.95 and unknown_ratio <= 0.05 and not missing_walls:
        verdict = "commercial_ready"
    elif coverage >= 0.85 and free_component_ratio >= 0.80:
        verdict = "navigation_ready_best_effort"
    else:
        verdict = "needs_remap"

    recommendations = []
    if missing_walls:
        recommendations.append(
            "Revisit or manually drive along missing map edges: " + ", ".join(missing_walls)
        )
    if unknown_ratio > 0.05:
        recommendations.append("Run another frontier/gap-fill pass before accepting the map.")
    if free_component_ratio < 0.80:
        recommendations.append("Check door openings and costmap inflation; free space is fragmented.")
    if verdict == "commercial_ready":
        recommendations.append("Map can enter zone/waypoint annotation and saved-map navigation tests.")

    return {
        "map": str(Path(map_yaml)),
        "size": {"width": grid.width, "height": grid.height, "resolution": grid.resolution},
        "origin": [grid.origin_x, grid.origin_y, 0.0],
        "coverage_scope": coverage_scope,
        "roi_bounds": roi_bounds,
        "counts": counts,
        "raw_counts": raw_counts,
        "coverage": round(coverage, 4),
        "unknown_ratio": round(unknown_ratio, 4),
        "raw_coverage": round(raw_coverage, 4),
        "raw_unknown_ratio": round(raw_unknown_ratio, 4),
        "largest_free_component_ratio": round(free_component_ratio, 4),
        "wall_scores": {key: round(value, 4) for key, value in wall_scores.items()},
        "missing_walls": missing_walls,
        "verdict": verdict,
        "recommendations": recommendations,
    }


def format_report(result: dict[str, object]) -> str:
    size = result["size"]
    counts = result["counts"]
    return "\n".join(
        [
            "MecaMind map quality report",
            f"map: {result['map']}",
            f"size: {size['width']}x{size['height']} @ {size['resolution']}m",
            f"coverage_scope: {result['coverage_scope']}",
            f"counts: free={counts['free']} occupied={counts['occupied']} unknown={counts['unknown']}",
            f"coverage: {float(result['coverage']) * 100.0:.1f}%",
            f"unknown_ratio: {float(result['unknown_ratio']) * 100.0:.1f}%",
            f"raw_full_map_coverage: {float(result['raw_coverage']) * 100.0:.1f}%",
            f"raw_full_map_unknown_ratio: {float(result['raw_unknown_ratio']) * 100.0:.1f}%",
            f"largest_free_component_ratio: {float(result['largest_free_component_ratio']) * 100.0:.1f}%",
            f"wall_scores: {json.dumps(result['wall_scores'], ensure_ascii=False)}",
            f"missing_walls: {', '.join(result['missing_walls']) if result['missing_walls'] else 'none'}",
            f"verdict: {result['verdict']}",
            "recommendations:",
            *[f"- {item}" for item in result["recommendations"]],
        ]
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze MecaMind YAML/PGM map quality.")
    parser.add_argument("map_yaml", help="Path to a nav2_map_server YAML file.")
    parser.add_argument("--world", default=None, help="Optional MecaMind world file for wall checks.")
    parser.add_argument("--json", action="store_true", help="Print JSON instead of text.")
    args = parser.parse_args()

    result = analyze_map(args.map_yaml, args.world)
    if args.json:
        print(json.dumps(result, ensure_ascii=False, indent=2))
    else:
        print(format_report(result))


if __name__ == "__main__":
    main()
