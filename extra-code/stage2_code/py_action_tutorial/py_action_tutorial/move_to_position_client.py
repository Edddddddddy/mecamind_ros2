# move_to_position_client.py —— MoveToPosition Action 客户端
# 向服务器发送目标坐标 (x, y)，沿途打印 Feedback（当前位置与
# 剩余距离），最后打印 Result（最终位置、误差、状态）。
# 运行（需先启动 turtlesim 和 move_to_position_server）：
#   ros2 run py_action_tutorial move_to_position_client
import rclpy
from action_msgs.msg import GoalStatus
from count_until_interfaces.action import MoveToPosition
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class MoveToPositionClient(Node):
    def __init__(self):
        super().__init__("move_to_position_client")
        # Action 名称需与服务器一致
        self.client_ = ActionClient(self, MoveToPosition, "move_to_position")

    def send_goal(self, x=8.0, y=8.0):
        # 等服务器上线 -> 填 Goal -> 异步发送并注册回调，
        # 流程与 count_until_client 完全一致
        self.get_logger().info("Waiting for action server...")
        if not self.client_.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available")
            rclpy.shutdown()
            return
        goal = MoveToPosition.Goal()
        goal.target_x = float(x)
        goal.target_y = float(y)
        future = self.client_.send_goal_async(goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        # 服务器控制循环里每次 publish_feedback 都会触发这里
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"pose=({fb.current_x:.2f}, {fb.current_y:.2f}), remaining={fb.distance_remaining:.2f}")

    def goal_response_callback(self, future):
        # 服务器答复接受/拒绝；被接受后再异步索取 Result
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"Goal request failed: {exc}")
            rclpy.shutdown()
            return
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected")
            rclpy.shutdown()
            return
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        # 打印最终 Result 及其状态码后结束程序
        try:
            wrapped_result = future.result()
        except Exception as exc:
            self.get_logger().error(f"Result request failed: {exc}")
            rclpy.shutdown()
            return
        result = wrapped_result.result
        status_name = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
        }.get(wrapped_result.status, str(wrapped_result.status))
        self.get_logger().info(
            f"final=({result.final_x:.2f}, {result.final_y:.2f}), "
            f"error={result.distance_error:.3f}, status={status_name}")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MoveToPositionClient()
    node.send_goal()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
