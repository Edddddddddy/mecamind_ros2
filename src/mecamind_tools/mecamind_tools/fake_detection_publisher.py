from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def build_detection_payload(
    label: str,
    confidence: float,
    cx: float,
    cy: float,
    width: float,
    height: float,
) -> str:
    return json.dumps(
        {
            "detections": [
                {
                    "label": label,
                    "confidence": confidence,
                    "bbox": {
                        "cx": cx,
                        "cy": cy,
                        "width": width,
                        "height": height,
                    },
                }
            ]
        },
        ensure_ascii=False,
    )


class FakeDetectionPublisher(Node):
    """Classroom stand-in for a YOLO detector (JSON contract only)."""

    def __init__(self) -> None:
        super().__init__("mecamind_fake_detection_publisher")
        self.declare_parameter("output_topic", "/mecamind/raw_detections")
        self.declare_parameter("cmd_topic", "/mecamind/fake_detection_cmd")
        self.declare_parameter("label", "person")
        self.declare_parameter("confidence", 0.82)
        self.declare_parameter("cx", 0.55)
        self.declare_parameter("cy", 0.50)
        self.declare_parameter("width", 0.22)
        self.declare_parameter("height", 0.40)
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("enabled", True)

        self._label = str(self.get_parameter("label").value)
        self._confidence = float(self.get_parameter("confidence").value)
        self._cx = float(self.get_parameter("cx").value)
        self._cy = float(self.get_parameter("cy").value)
        self._width = float(self.get_parameter("width").value)
        self._height = float(self.get_parameter("height").value)
        self._enabled = bool(self.get_parameter("enabled").value)

        self.pub = self.create_publisher(String, str(self.get_parameter("output_topic").value), 10)
        self.create_subscription(String, str(self.get_parameter("cmd_topic").value), self._cmd_cb, 10)
        rate_hz = max(0.5, float(self.get_parameter("rate_hz").value))
        self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info("MecaMind fake detection publisher ready")

    def _cmd_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Invalid fake detection cmd: {exc}")
            return
        if "enabled" in data:
            self._enabled = bool(data["enabled"])
        if "label" in data:
            self._label = str(data["label"])
        if "confidence" in data:
            self._confidence = float(data["confidence"])
        if "cx" in data:
            self._cx = float(data["cx"])
        if "cy" in data:
            self._cy = float(data["cy"])
        if "width" in data:
            self._width = float(data["width"])
        if "height" in data:
            self._height = float(data["height"])

    def _tick(self) -> None:
        if not self._enabled:
            return
        out = String()
        out.data = build_detection_payload(
            self._label,
            self._confidence,
            self._cx,
            self._cy,
            self._width,
            self._height,
        )
        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeDetectionPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
