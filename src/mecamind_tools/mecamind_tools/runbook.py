from dataclasses import dataclass
from typing import List

from .mode_profiles import ModeProfile
from .world_profiles import WorldBlueprint, summarize_blueprint


@dataclass(frozen=True)
class ChecklistItem:
    name: str
    command: str
    required: bool = True


def build_boot_checklist(profile: ModeProfile, blueprint: WorldBlueprint) -> List[ChecklistItem]:
    return [
        ChecklistItem("ros_domain_id", "echo $ROS_DOMAIN_ID", True),
        ChecklistItem("scan_topic", f"ros2 topic hz {profile.topic_map.scan}", True),
        ChecklistItem("imu_topic", f"ros2 topic echo --once {profile.topic_map.imu}", True),
        ChecklistItem("odom_topic", f"ros2 topic echo --once {profile.topic_map.odom}", True),
        ChecklistItem("tf_tree", "ros2 run tf2_tools view_frames", True),
        ChecklistItem("world_assets", f"check world {blueprint.name}", True),
        ChecklistItem("cmd_vel", f"ros2 topic info {profile.topic_map.cmd_vel}", True),
    ]


def render_boot_checklist(items: List[ChecklistItem]) -> str:
    lines = []
    for index, item in enumerate(items, start=1):
        suffix = "required" if item.required else "optional"
        lines.append(f"{index}. {item.name} [{suffix}] -> {item.command}")
    return "\n".join(lines)


def render_mission_summary(profile: ModeProfile, blueprint: WorldBlueprint) -> str:
    return "\n".join(
        [
            "MecaMind mission brief",
            summarize_blueprint(blueprint),
            f"mode={profile.name}",
            f"mission={profile.mission}",
            f"use_sim_time={profile.use_sim_time}",
        ]
    )


def build_mission_brief_payload(
    profile: ModeProfile,
    blueprint: WorldBlueprint,
    world_path: str,
) -> dict:
    checklist = build_boot_checklist(profile, blueprint)
    return {
        "summary": render_mission_summary(profile, blueprint),
        "world_path": world_path,
        "profile": profile.to_dict(),
        "checklist": [
            {"name": item.name, "command": item.command, "required": item.required}
            for item in checklist
        ],
        "checklist_text": render_boot_checklist(checklist),
    }
