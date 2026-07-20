# move_to_position_server.py —— 控制 turtlesim 移动的 Action 服务器
# 综合示例：Action 服务器 + 话题订阅（海龟位姿）+ 话题发布（速度）。
# 收到目标坐标后用比例控制（P 控制）驱动海龟走过去，
# 沿途持续发布 Feedback（当前位置和剩余距离）。
# 运行（需先启动 turtlesim）：
#   ros2 run py_action_tutorial move_to_position_server
import math
from threading import Lock
import time

import rclpy
from count_until_interfaces.action import MoveToPosition
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from turtlesim.msg import Pose


class MoveToPositionServer(Node):
    def __init__(self):
        super().__init__("move_to_position_server")
        self.pose_ = None
        # goal_reserved_ 配合线程锁实现"同一时刻只接一个 Goal"，
        # 因为只有一只海龟，不能同时朝两个目标移动
        self.goal_lock_ = Lock()
        self.goal_reserved_ = False
        # 可重入回调组：execute_callback 循环期间仍要接收位姿、
        # 处理取消请求，因此各回调必须能并行执行
        self.callback_group_ = ReentrantCallbackGroup()
        # 发布速度指令控制海龟，订阅海龟当前位姿
        self.publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.subscription_ = self.create_subscription(
            Pose,
            "/turtle1/pose",
            self.pose_callback,
            10,
            callback_group=self.callback_group_)
        self.action_server_ = ActionServer(
            self,
            MoveToPosition,
            "move_to_position",
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group_,
        )
        self.get_logger().info("MoveToPosition Action Server ready on /move_to_position")

    def pose_callback(self, pose: Pose):
        # 持续记录海龟最新位姿，供控制循环读取
        self.pose_ = pose

    def goal_callback(self, goal_request):
        # Goal 审核分两步：
        # 1) 目标坐标必须在 turtlesim 窗口范围（0~11）内
        # 2) 已有 Goal 在执行时拒绝新 Goal（单海龟一次只能一个目标）
        if not (0.0 <= goal_request.target_x <= 11.0 and 0.0 <= goal_request.target_y <= 11.0):
            return GoalResponse.REJECT
        with self.goal_lock_:
            if self.goal_reserved_:
                self.get_logger().warn("Rejecting goal: another move is active")
                return GoalResponse.REJECT
            self.goal_reserved_ = True
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        # 允许取消；实际停车逻辑在 execute_callback 里
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        # 控制主循环：约每 0.05 秒算一次速度指令，直到到达目标
        feedback = MoveToPosition.Feedback()
        result = MoveToPosition.Result()
        pose_wait_started = time.monotonic()
        try:
            while rclpy.ok():
                # 被取消：先让海龟停下，再标记 canceled
                if goal_handle.is_cancel_requested:
                    self.stop_turtle()
                    goal_handle.canceled()
                    return result
                # 还没收到位姿：等一会儿；超过 5 秒说明 turtlesim
                # 没启动，标记 abort（异常终止）
                if self.pose_ is None:
                    if time.monotonic() - pose_wait_started > 5.0:
                        self.get_logger().error("No /turtle1/pose received within 5 seconds")
                        goal_handle.abort()
                        return result
                    time.sleep(0.05)
                    continue

                # 计算到目标的直线距离，足够近（<0.15）就认为到达
                dx = goal_handle.request.target_x - self.pose_.x
                dy = goal_handle.request.target_y - self.pose_.y
                distance = math.sqrt(dx * dx + dy * dy)
                if distance < 0.15:
                    break

                # P 控制：误差越大速度越快，越接近目标越慢
                # 线速度与剩余距离成正比（上限 2.0），
                # 角速度与朝向误差成正比（增益 6.0）
                target_angle = math.atan2(dy, dx)
                angle_error = self.normalize_angle(target_angle - self.pose_.theta)
                cmd = Twist()
                cmd.linear.x = min(2.0, distance)
                cmd.angular.z = 6.0 * angle_error
                self.publisher_.publish(cmd)

                # 把当前进度作为 Feedback 发给客户端
                feedback.current_x = self.pose_.x
                feedback.current_y = self.pose_.y
                feedback.distance_remaining = distance
                goal_handle.publish_feedback(feedback)
                time.sleep(0.05)

            # 到达目标：停车、填写最终位置和误差、标记成功
            self.stop_turtle()
            result.final_x = self.pose_.x
            result.final_y = self.pose_.y
            result.distance_error = math.sqrt(
                (goal_handle.request.target_x - self.pose_.x) ** 2 +
                (goal_handle.request.target_y - self.pose_.y) ** 2)
            goal_handle.succeed()
            return result
        finally:
            # 无论成功/取消/异常都要释放"占用"标记，
            # 否则后续 Goal 会被永远拒绝
            with self.goal_lock_:
                self.goal_reserved_ = False

    def stop_turtle(self):
        # 发布全零的 Twist 让海龟立即停止
        if rclpy.ok():
            try:
                self.publisher_.publish(Twist())
            except Exception:
                pass

    @staticmethod
    def normalize_angle(angle):
        # 把角度归一化到 [-pi, pi]，避免海龟"绕远路"转圈
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = MoveToPositionServer()
    # 多线程执行器：控制循环执行时仍能并行处理位姿订阅与取消请求
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
