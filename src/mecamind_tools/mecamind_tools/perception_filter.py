from __future__ import annotations

from dataclasses import dataclass
import json
from typing import Any, Dict, Iterable, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


@dataclass(frozen=True)
class Detection:
    label: str
    confidence: float
    cx: float
    cy: float
    width: float = 0.0
    height: float = 0.0


def parse_detections(payload: str) -> List[Detection]:
    data = json.loads(payload)
    items = data.get("detections", data if isinstance(data, list) else [])
    result = []
    for item in items:
        if not isinstance(item, dict):
            continue
        bbox = item.get("bbox", {})
        result.append(
            Detection(
                label=str(item.get("label", item.get("class", ""))),
                confidence=float(item.get("confidence", item.get("score", 0.0))),
                cx=float(item.get("cx", bbox.get("cx", bbox.get("center_x", 0.5)))),
                cy=float(item.get("cy", bbox.get("cy", bbox.get("center_y", 0.5)))),
                width=float(item.get("width", bbox.get("width", 0.0))),
                height=float(item.get("height", bbox.get("height", 0.0))),
            )
        )
    return result


def select_detection(
    detections: Iterable[Detection],
    target_label: str,
    min_confidence: float,
) -> Detection | None:
    candidates = [
        item
        for item in detections
        if item.label == target_label and item.confidence >= min_confidence
    ]
    if not candidates:
        return None
    candidates.sort(key=lambda item: item.confidence, reverse=True)
    return candidates[0]


class PerceptionFilterNode(Node):
    def __init__(self) -> None:
        super().__init__("mecamind_perception_filter")
        self.declare_parameter("input_topic", "/mecamind/raw_detections")
        self.declare_parameter("output_topic", "/mecamind/perception_event")
        self.declare_parameter("target_label", "person")
        self.declare_parameter("min_confidence", 0.55)
        self.declare_parameter("confirm_frames", 2)

        self._confirm_count = 0
        self._last_target: Detection | None = None
        self.create_subscription(String, str(self.get_parameter("input_topic").value), self._cb, 10)
        self.pub = self.create_publisher(String, str(self.get_parameter("output_topic").value), 10)
        self.get_logger().info("MecaMind perception filter ready")

    def _cb(self, msg: String) -> None:
        try:
            detections = parse_detections(msg.data)
        except Exception as exc:  # noqa: BLE001 - teaching diagnostics
            self.get_logger().warn(f"Invalid detection payload: {exc}")
            return
        target = select_detection(
            detections,
            str(self.get_parameter("target_label").value),
            float(self.get_parameter("min_confidence").value),
        )
        if target is None:
            self._confirm_count = 0
            self._last_target = None
            return

        self._last_target = target
        self._confirm_count += 1
        confirmed = self._confirm_count >= int(self.get_parameter("confirm_frames").value)
        event: Dict[str, Any] = {
            "label": target.label,
            "confidence": target.confidence,
            "cx": target.cx,
            "cy": target.cy,
            "width": target.width,
            "height": target.height,
            "confirmed": confirmed,
            "confirm_count": self._confirm_count,
        }
        output = String()
        output.data = json.dumps(event, ensure_ascii=False)
        self.pub.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PerceptionFilterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
