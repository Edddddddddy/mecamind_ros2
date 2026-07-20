# turtle_controller.py —— P1 项目"抓海龟"的大脑节点
# 职责：订阅 alive_turtles 得到存活海龟列表，挑选目标（可选
# "先抓最近的"策略），通过 MoveToPosition Action 让 turtle1
# 移动过去，到达后调用 catch_turtle Service 请求"抓住"它。
# 综合运用了本阶段全部通信方式：Topic + Service + Action + 参数。
# 运行：ros2 run turtlesim_catch_them_all controller
from functools import partial
import math

from action_msgs.msg import GoalStatus
from my_robot_interfaces.action import MoveToPosition
from my_robot_interfaces.msg import Turtle, TurtleArray
from my_robot_interfaces.srv import CatchTurtle
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from turtlesim.msg import Pose


class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        # 参数：是否优先抓离自己最近的海龟（可在启动时覆盖）
        self.declare_parameter("catch_closest_turtle_first", True)

        self.catch_closest_turtle_first_ = self.get_parameter(
            "catch_closest_turtle_first").value
        self.turtle_to_catch_: Turtle | None = None
        self.pose_: Pose | None = None
        # goal_in_progress_ 防止移动尚未完成时又发出新 Goal
        self.goal_in_progress_ = False
        self.last_feedback_log_time_ = 0.0

        # 通信接口：订阅自身位姿和存活海龟列表；
        # Action 客户端负责移动，Service 客户端负责"抓捕"
        self.pose_subscriber_ = self.create_subscription(
            Pose, "/turtle1/pose", self.callback_pose, 10)
        self.alive_turtles_subscriber_ = self.create_subscription(
            TurtleArray, "alive_turtles", self.callback_alive_turtles, 10)
        self.move_action_client_ = ActionClient(self, MoveToPosition, "move_to_position")
        self.catch_turtle_client_ = self.create_client(CatchTurtle, "catch_turtle")

        self.get_logger().info(
            f"catch_closest_turtle_first = {self.catch_closest_turtle_first_}")

    def callback_pose(self, pose: Pose):
        self.pose_ = pose

    def callback_alive_turtles(self, msg: TurtleArray):
        # 列表为空、正在移动中、还不知道自己位置时都先不行动
        if not msg.turtles or self.goal_in_progress_ or self.pose_ is None:
            return

        # 选目标：遍历所有海龟找距离最近的一只；
        # 若参数关闭该策略，则简单取列表第一只
        if self.catch_closest_turtle_first_:
            closest = None
            closest_distance = None
            for turtle in msg.turtles:
                distance = math.hypot(turtle.x - self.pose_.x, turtle.y - self.pose_.y)
                if closest is None or distance < closest_distance:
                    closest = turtle
                    closest_distance = distance
            self.turtle_to_catch_ = closest
        else:
            self.turtle_to_catch_ = msg.turtles[0]

        self.send_move_goal(self.turtle_to_catch_.x, self.turtle_to_catch_.y)

    def send_move_goal(self, x, y):
        # 先置忙标志，再向 Action 服务器发送目标坐标
        self.goal_in_progress_ = True
        if not self.move_action_client_.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("move_to_position action server is not available")
            self.goal_in_progress_ = False
            return

        goal = MoveToPosition.Goal()
        goal.target_x = float(x)
        goal.target_y = float(y)
        future = self.move_action_client_.send_goal_async(
            goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        # Feedback 频率很高，这里做节流：最多每秒打印一次剩余距离
        feedback = feedback_msg.feedback
        now = self.get_clock().now().nanoseconds / 1_000_000_000.0
        if now - self.last_feedback_log_time_ >= 1.0:
            self.get_logger().info(f"Distance remaining: {feedback.distance_remaining:.2f}")
            self.last_feedback_log_time_ = now

    def goal_response_callback(self, future):
        # 服务器答复 Goal 被接受/拒绝；失败或被拒都要复位忙标志
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"Move goal failed: {exc}")
            self.goal_in_progress_ = False
            return

        if not goal_handle.accepted:
            self.get_logger().warn("Move goal rejected")
            self.goal_in_progress_ = False
            return
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        # 移动结束：只有状态为 SUCCEEDED 才继续抓捕，
        # 取消/中止时放弃该目标，等下一次列表更新重新选
        try:
            wrapped_result = future.result()
        except Exception as exc:
            self.get_logger().error(f"Move result failed: {exc}")
            self.turtle_to_catch_ = None
            self.goal_in_progress_ = False
            return

        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(
                f"Move did not succeed; action status={wrapped_result.status}")
            self.turtle_to_catch_ = None
            self.goal_in_progress_ = False
            return

        result = wrapped_result.result

        self.get_logger().info(
            f"Move finished at ({result.final_x:.2f}, {result.final_y:.2f}), "
            f"error={result.distance_error:.3f}")
        if self.turtle_to_catch_ is not None:
            self.call_catch_turtle_service(self.turtle_to_catch_.name)
        else:
            self.goal_in_progress_ = False

    def call_catch_turtle_service(self, turtle_name):
        # 到达目标后调用 spawner 提供的 catch_turtle Service，
        # 请求把这只海龟从屏幕和存活列表中移除
        if not self.catch_turtle_client_.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("catch_turtle service is not available")
            self.turtle_to_catch_ = None
            self.goal_in_progress_ = False
            return
        request = CatchTurtle.Request()
        request.name = turtle_name
        future = self.catch_turtle_client_.call_async(request)
        future.add_done_callback(partial(self.callback_catch_turtle, turtle_name=turtle_name))

    def callback_catch_turtle(self, future, turtle_name):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Caught turtle: {turtle_name}")
            else:
                self.get_logger().error(f"Failed to catch turtle: {turtle_name}")
        except Exception as exc:
            self.get_logger().error(f"Failed to call catch_turtle for {turtle_name}: {exc}")
        finally:
            # 一轮抓捕结束（无论成败），复位状态准备抓下一只
            self.turtle_to_catch_ = None
            self.goal_in_progress_ = False


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
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
