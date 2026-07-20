# count_until_server.py —— Action 服务器入门示例（从 1 数到目标值）
# 演示 Action 服务端的三个回调：
#   goal_callback   决定接受还是拒绝 Goal（GoalResponse）
#   cancel_callback 决定是否允许取消（CancelResponse）
#   execute_callback 真正干活：循环计数、发布 Feedback、返回 Result
# 运行：ros2 run py_action_tutorial count_until_server
import time

import rclpy
from count_until_interfaces.action import CountUntil
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node


class CountUntilServer(Node):
    def __init__(self):
        super().__init__("count_until_server")
        # ReentrantCallbackGroup：允许同组内多个回调并行执行。
        # execute_callback 里有 time.sleep 会长时间占用线程，
        # 若不并行，取消请求等其他回调会被卡住无法响应。
        self.callback_group_ = ReentrantCallbackGroup()
        # 创建 Action 服务器：指定 Action 类型、名称和三个回调
        self.action_server_ = ActionServer(
            self,
            CountUntil,
            "count_until",
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group_,
        )
        self.get_logger().info("Action Server ready on /count_until")

    def goal_callback(self, goal_request):
        # Goal 审核：先做参数校验，不合法的目标直接 REJECT，
        # 这样 execute_callback 就只需处理合法输入
        if goal_request.target_number <= 0 or goal_request.period <= 0.0:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        # 收到取消请求时的答复：ACCEPT 表示"允许取消"，
        # 真正停止计数的逻辑在 execute_callback 里检查
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        # Goal 被接受后由这里执行，返回值就是发回客户端的 Result
        feedback = CountUntil.Feedback()
        result = CountUntil.Result()
        period = max(0.1, float(goal_handle.request.period))

        for number in range(1, goal_handle.request.target_number + 1):
            # 每轮都要检查是否被取消，被取消则标记 canceled 并
            # 返回当前进度作为 Result
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.reached_number = number - 1
                self.get_logger().info(
                    f"Goal canceled at {result.reached_number}")
                return result
            # publish_feedback 让客户端实时看到执行进度，
            # 这正是 Action 相比 Service 的核心优势
            feedback.current_number = number
            goal_handle.publish_feedback(feedback)
            time.sleep(period)

        # 计数完成：标记成功并返回最终 Result
        result.reached_number = goal_handle.request.target_number
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = CountUntilServer()
    # MultiThreadedExecutor：多线程执行器，配合 Reentrant 回调组，
    # 让执行中的 Goal 不会阻塞取消请求等其他回调
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
