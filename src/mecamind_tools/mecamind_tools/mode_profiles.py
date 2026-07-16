from dataclasses import dataclass, asdict
from typing import Dict, List


@dataclass(frozen=True)
class TopicMap:
    scan: str = "/scan_raw"
    imu: str = "/imu/data_raw"
    odom: str = "/odom"
    cmd_vel: str = "/controller/cmd_vel"
    map_topic: str = "/map"
    tf: str = "/tf"
    tf_static: str = "/tf_static"


@dataclass(frozen=True)
class ModeProfile:
    name: str
    mission: str
    use_sim_time: bool
    expects_gazebo: bool
    topic_map: TopicMap
    world_name: str
    notes: List[str]

    def to_dict(self) -> Dict[str, object]:
        return {
            "name": self.name,
            "mission": self.mission,
            "use_sim_time": self.use_sim_time,
            "expects_gazebo": self.expects_gazebo,
            "topic_map": asdict(self.topic_map),
            "world_name": self.world_name,
            "notes": list(self.notes),
        }


def build_mode_profile(mode: str, mission: str = "mapping") -> ModeProfile:
    normalized = mode.strip().lower()
    if normalized not in {"sim", "real"}:
        raise ValueError(f"Unsupported mode: {mode}")

    if normalized == "sim":
        return ModeProfile(
            name="sim",
            mission=mission,
            use_sim_time=True,
            expects_gazebo=False,
            topic_map=TopicMap(),
            world_name="three_room_house.world",
            notes=[
                "Use MecaMind built-in 2D simulator (not Gazebo) with three_room_house.world.",
                "Prefer slower motion and visible obstacles.",
            ],
        )

    return ModeProfile(
        name="real",
        mission=mission,
        use_sim_time=False,
        expects_gazebo=False,
        topic_map=TopicMap(),
        world_name="real_robot",
        notes=[
            "Keep the same topic names as simulation.",
            "Use conservative velocity and strict safety checks.",
        ],
    )


def render_profile_summary(profile: ModeProfile) -> str:
    lines = [
        f"mode: {profile.name}",
        f"mission: {profile.mission}",
        f"use_sim_time: {profile.use_sim_time}",
        f"expects_gazebo: {profile.expects_gazebo}",
        f"world_name: {profile.world_name}",
        f"scan_topic: {profile.topic_map.scan}",
        f"imu_topic: {profile.topic_map.imu}",
        f"odom_topic: {profile.topic_map.odom}",
        f"cmd_vel_topic: {profile.topic_map.cmd_vel}",
    ]
    lines.extend(f"note: {item}" for item in profile.notes)
    return "\n".join(lines)
