import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .mode_profiles import build_mode_profile, render_profile_summary
from .runbook import build_mission_brief_payload
from .world_profiles import three_room_blueprint, validate_three_room_blueprint


class MissionBriefNode(Node):
    def __init__(self) -> None:
        super().__init__("mecamind_mission_brief")
        self.declare_parameter("mode", "sim")
        self.declare_parameter("mission", "mapping")
        self.declare_parameter("world_path", "")
        self.declare_parameter("output_topic", "/mecamind/mission_brief")
        mode = self.get_parameter("mode").value
        mission = self.get_parameter("mission").value
        world_path = self.get_parameter("world_path").value
        output_topic = self.get_parameter("output_topic").value

        self.profile = build_mode_profile(mode, mission=mission)
        self.blueprint = three_room_blueprint()
        if not validate_three_room_blueprint(self.blueprint):
            raise ValueError("three_room_blueprint is not valid")
        self.world_path = world_path

        self.publisher = self.create_publisher(String, output_topic, 10)
        self.timer = self.create_timer(2.0, self._publish_brief)
        self.get_logger().info(render_profile_summary(self.profile))

    def _publish_brief(self) -> None:
        payload = build_mission_brief_payload(
            self.profile,
            self.blueprint,
            self.world_path,
        )
        self.publisher.publish(String(data=json.dumps(payload, ensure_ascii=False)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionBriefNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        finally:
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
