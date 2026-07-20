# count_until_client.py —— Action 客户端入门示例
# 演示客户端的完整流程（全部基于异步回调，不阻塞主线程）：
#   发送 Goal -> 服务器答复接受/拒绝 -> 接收 Feedback -> 接收 Result
# 运行（需先启动 count_until_server）：
#   ros2 run py_action_tutorial count_until_client
import rclpy
from action_msgs.msg import GoalStatus
from count_until_interfaces.action import CountUntil
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class CountUntilClient(Node):
    def __init__(self):
        super().__init__("count_until_client")
        # ActionClient 三要素：所属节点、Action 类型、Action 名称
        # （名称必须与服务器端一致才能连上）
        self.client_ = ActionClient(self, CountUntil, "count_until")

    def send_goal(self, target_number=5, period=0.5):
        # 发送前先等服务器上线，避免 Goal 石沉大海
        self.get_logger().info("Waiting for action server...")
        if not self.client_.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available")
            rclpy.shutdown()
            return
        # 填写 Goal 消息并异步发送；send_goal_async 立刻返回 future，
        # 同时注册 feedback_callback 用于接收服务器的进度反馈
        goal = CountUntil.Goal()
        goal.target_number = target_number
        goal.period = period
        future = self.client_.send_goal_async(goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        # 服务器每次 publish_feedback 都会触发这里
        self.get_logger().info(f"Current number: {feedback_msg.feedback.current_number}")

    def goal_response_callback(self, future):
        # 第一步回应：服务器只告诉我们 Goal 被接受还是拒绝，
        # 被接受后还要再异步请求最终的 Result
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
        # 最终 Result 到达：除数据外还带有状态码
        # （SUCCEEDED / CANCELED / ABORTED）
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
            f"Reached number: {result.reached_number}; status={status_name}")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = CountUntilClient()
    # 先发出 Goal，再 spin 等待各个回调被依次触发
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
