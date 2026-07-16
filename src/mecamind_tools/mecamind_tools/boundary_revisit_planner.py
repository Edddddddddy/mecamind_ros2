from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict, Iterable, List

import yaml

from .map_quality_analyzer import analyze_map
from .world_geometry import load_world_geometry


def _route_for_side(side: str, min_x: float, max_x: float, min_y: float, max_y: float) -> List[Dict[str, float | str]]:
    inset = 0.70
    low_inset = 0.42
    dwell_sec = 1.0
    west_x = min_x + inset
    east_x = max_x - inset
    north_y = max_y - inset
    south_y = min_y + inset
    south_low_y = min_y + low_inset
    if side == "north":
        return [
            {"name": "revisit_north_east", "x": east_x, "y": north_y, "yaw": 1.57, "dwell_sec": dwell_sec},
            {"name": "revisit_north_mid", "x": 0.0, "y": north_y, "yaw": 1.57, "dwell_sec": dwell_sec},
            {"name": "north_to_center_corridor", "x": 0.0, "y": -1.20},
            {"name": "north_to_entry_hall", "x": -2.80, "y": -1.80},
            {"name": "north_west_table_bypass_lower", "x": west_x, "y": 0.60},
            {"name": "north_west_table_bypass_upper", "x": west_x, "y": 2.20, "yaw": 1.57, "dwell_sec": dwell_sec},
            {"name": "revisit_north_west", "x": -2.10, "y": north_y, "yaw": 1.57, "dwell_sec": dwell_sec},
            {"name": "north_west_exit_bypass_upper", "x": west_x, "y": 2.20},
            {"name": "north_west_exit_bypass_lower", "x": west_x, "y": 0.60},
            {"name": "north_west_exit_entry_hall", "x": -2.80, "y": -1.80},
        ]
    if side == "south":
        return [
            {"name": "revisit_south_west", "x": west_x, "y": south_y, "yaw": -1.57, "dwell_sec": dwell_sec},
            {"name": "south_pillar_left", "x": 0.0, "y": south_y, "yaw": -1.57, "dwell_sec": dwell_sec},
            {"name": "south_pillar_upper_bypass", "x": 1.10, "y": south_y},
            {"name": "south_cabinet_lower_bypass", "x": 1.10, "y": south_low_y},
            {"name": "revisit_south_east", "x": east_x, "y": south_low_y, "yaw": -1.57, "dwell_sec": dwell_sec},
        ]
    if side == "east":
        return [
            {"name": "revisit_east_south", "x": east_x, "y": south_low_y, "yaw": 0.0, "dwell_sec": dwell_sec},
            {"name": "revisit_east_mid", "x": east_x, "y": 0.0, "yaw": 0.0, "dwell_sec": dwell_sec},
            {"name": "revisit_east_north", "x": east_x, "y": north_y, "yaw": 0.0, "dwell_sec": dwell_sec},
        ]
    if side == "west":
        return [
            {"name": "west_south_entry_clearance", "x": east_x, "y": south_low_y, "yaw": 3.14, "dwell_sec": dwell_sec},
            {"name": "revisit_west_south", "x": west_x, "y": south_y, "yaw": 3.14, "dwell_sec": dwell_sec},
            {"name": "revisit_west_mid", "x": west_x, "y": 0.0, "yaw": 3.14, "dwell_sec": dwell_sec},
            {"name": "revisit_west_north", "x": west_x, "y": north_y, "yaw": 3.14, "dwell_sec": dwell_sec},
        ]
    return []


def _load_base_waypoints(route_path: str | Path | None) -> List[Dict[str, Any]]:
    if not route_path:
        return []
    with Path(route_path).expanduser().open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    waypoints = config.get("waypoints", [])
    if not isinstance(waypoints, list):
        raise ValueError(f"Base route has no waypoint list: {route_path}")
    return [dict(item) for item in waypoints if isinstance(item, dict)]


def _dedupe_waypoints(waypoints: Iterable[Dict[str, Any]], min_spacing: float) -> List[Dict[str, Any]]:
    result: List[Dict[str, Any]] = []
    for item in waypoints:
        x = float(item["x"])
        y = float(item["y"])
        duplicate = False
        if result:
            old = result[-1]
            dx = x - float(old["x"])
            dy = y - float(old["y"])
            if (dx * dx + dy * dy) ** 0.5 < min_spacing:
                duplicate = True
        if not duplicate:
            result.append(item)
    return result


def build_revisit_route(
    map_yaml: str | Path,
    world_path: str | Path,
    base_route: str | Path | None = None,
    include_base_route: bool = False,
    include_all_sides: bool = False,
    min_spacing: float = 0.20,
) -> Dict[str, object]:
    quality = analyze_map(map_yaml, world_path)
    world = load_world_geometry(world_path)
    missing = list(quality.get("missing_walls", []))
    if not missing and quality.get("verdict") != "commercial_ready":
        missing = ["north", "east"]

    revisit_waypoints: List[Dict[str, float | str]] = []
    selected_sides = ("east", "north", "south", "west") if include_all_sides else tuple(missing)
    ordered_missing = [side for side in ("east", "north", "south", "west") if side in selected_sides]
    for side in ordered_missing:
        revisit_waypoints.extend(_route_for_side(side, world.min_x, world.max_x, world.min_y, world.max_y))
    base_waypoints = _load_base_waypoints(base_route) if include_base_route else []
    waypoints = _dedupe_waypoints([*base_waypoints, *revisit_waypoints], min_spacing=min_spacing)
    return {
        "source_map": str(map_yaml),
        "world": str(world_path),
        "base_route": str(base_route) if base_route else "",
        "quality_verdict": quality["verdict"],
        "missing_walls": missing,
        "include_all_sides": include_all_sides,
        "revisit_waypoint_count": len(revisit_waypoints),
        "waypoints": waypoints,
    }


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Generate a MecaMind boundary revisit route.")
    parser.add_argument("map_yaml")
    parser.add_argument("--world", required=True)
    parser.add_argument("--base-route", default="")
    parser.add_argument("--include-base-route", action="store_true")
    parser.add_argument("--include-all-sides", action="store_true")
    parser.add_argument("--min-spacing", type=float, default=0.20)
    parser.add_argument("--output", default="")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(list(argv) if argv is not None else None)

    route = build_revisit_route(
        args.map_yaml,
        args.world,
        base_route=args.base_route or None,
        include_base_route=args.include_base_route,
        include_all_sides=args.include_all_sides,
        min_spacing=args.min_spacing,
    )
    if args.output:
        Path(args.output).write_text(yaml.safe_dump(route, sort_keys=False, allow_unicode=True), encoding="utf-8")
    print(json.dumps(route, ensure_ascii=False, indent=2) if args.json else yaml.safe_dump(route, sort_keys=False, allow_unicode=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
