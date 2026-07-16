import math
import os
from typing import Any, Dict

import yaml
from geometry_msgs.msg import PoseStamped, Quaternion


def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def load_yaml(path: str) -> Dict[str, Any]:
    expanded = os.path.expanduser(path)
    with open(expanded, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        raise ValueError(f"YAML root must be a mapping: {path}")
    return data


def pose_stamped_from_dict(item: Dict[str, Any], stamp) -> PoseStamped:
    pose = PoseStamped()
    pose.header.stamp = stamp
    pose.header.frame_id = str(item.get("frame_id", "map"))
    pose.pose.position.x = float(item["x"])
    pose.pose.position.y = float(item["y"])
    pose.pose.position.z = float(item.get("z", 0.0))
    pose.pose.orientation = quaternion_from_yaw(float(item.get("yaw", 0.0)))
    return pose


def initial_pose_from_dict(item: Dict[str, Any], stamp) -> PoseStamped:
    pose = PoseStamped()
    pose.header.stamp = stamp
    pose.header.frame_id = str(item.get("frame_id", "map"))
    pose.pose.position.x = float(item["x"])
    pose.pose.position.y = float(item["y"])
    pose.pose.position.z = float(item.get("z", 0.0))
    pose.pose.orientation = quaternion_from_yaw(float(item.get("yaw", 0.0)))
    return pose
