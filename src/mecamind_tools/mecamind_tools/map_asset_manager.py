from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Dict, Iterable, List

import yaml

from .map_quality_analyzer import analyze_map


_STATUS_RANK = {
    "invalid": 0,
    "needs_remap": 1,
    "navigation_ready_best_effort": 2,
    "commercial_ready": 3,
}


def _looks_like_map_yaml(path: Path) -> bool:
    try:
        with path.open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
    except Exception:
        return True
    return isinstance(data, dict) and "image" in data


def _is_checkpoint_map(path: Path) -> bool:
    return "_checkpoint_" in path.stem


def _asset_score(asset: Dict[str, object]) -> tuple[int, int, float, float]:
    status = str(asset.get("status", "invalid"))
    missing = asset.get("missing_walls", [])
    missing_count = len(missing) if isinstance(missing, list) else 99
    coverage = float(asset.get("coverage", 0.0))
    unknown_ratio = float(asset.get("unknown_ratio", 1.0))
    return (_STATUS_RANK.get(status, 0), -missing_count, coverage, -unknown_ratio)


def collect_map_assets(
    map_dir: str | Path,
    world_path: str | Path | None = None,
    include_checkpoints: bool = False,
) -> Dict[str, object]:
    root = Path(map_dir)
    assets: List[Dict[str, object]] = []
    for yaml_path in sorted(root.glob("*.yaml")):
        if not include_checkpoints and _is_checkpoint_map(yaml_path):
            continue
        if not _looks_like_map_yaml(yaml_path):
            continue
        try:
            report = analyze_map(yaml_path, world_path)
        except Exception as exc:  # noqa: BLE001 - report broken map assets
            assets.append({"map": str(yaml_path), "status": "invalid", "error": str(exc)})
            continue
        assets.append(
            {
                "map": str(yaml_path),
                "status": report["verdict"],
                "coverage": report["coverage"],
                "unknown_ratio": report["unknown_ratio"],
                "missing_walls": report["missing_walls"],
            }
        )
    valid_assets = [asset for asset in assets if asset.get("status") != "invalid"]
    recommended = max(valid_assets, key=_asset_score) if valid_assets else None
    return {
        "map_dir": str(root),
        "count": len(assets),
        "recommended_map": recommended["map"] if recommended else "",
        "recommended_status": recommended["status"] if recommended else "",
        "recommendation_rule": "status > fewer_missing_walls > coverage > lower_unknown_ratio",
        "assets": assets,
    }


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Build a MecaMind map asset manifest.")
    parser.add_argument("map_dir")
    parser.add_argument("--world", default=None)
    parser.add_argument("--include-checkpoints", action="store_true")
    parser.add_argument("--output", default="")
    args = parser.parse_args(list(argv) if argv is not None else None)

    manifest = collect_map_assets(args.map_dir, args.world, include_checkpoints=args.include_checkpoints)
    text = json.dumps(manifest, ensure_ascii=False, indent=2)
    if args.output:
        Path(args.output).write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
