# move_to_position_server.py —— P1 项目的移动执行 Action 服务器
# 收到目标坐标后用 P 控制驱动 turtle1 移动，沿途发布 Feedback。
# 相比教学版增加了 max_speed 参数和更完善的关闭/异常处理。
# 运行：ros2 run turtlesim_catch_them_all move_to_position_server
import math
from threading import Lock
import time

from geometry_msgs.msg import Twist
from my_robot_interfaces.action import MoveToPosition
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from turtlesim.msg import Pose


class MoveToPositionServer(Node):
    def __init__(self):
        super().__init__("move_to_position_server")
        # 参数：线速度上限，可在启动时调整海龟的"最快速度"
        self.declare_parameter("max_speed", 2.0)
        self.max_speed_ = self.get_parameter("max_speed").value
        self.pose_: Pose | None = None
        self.shutting_down_ = False
        self.active_goal_handle_ = None
        # goal_reserved_ + 线程锁：保证同一时刻只执行一个 Goal
        self.goal_lock_ = Lock()
        self.goal_reserved_ = False
        # 可重入回调组：执行 Goal 期间仍需并行接收位姿和取消请求
        self.callback_group_ = ReentrantCallbackGroup()

        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pose_subscriber_ = self.create_subscription(
            Pose,
            "/turtle1/pose",
            self.callback_pose,
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
        self.get_logger().info(f"MoveToPosition Action Server ready, max_speed={self.max_speed_}")

    def callback_pose(self, pose: Pose):
        self.pose_ = pose

    def goal_callback(self, goal_request):
        # Goal 审核：坐标须在窗口范围（0~11）内；
        # 已有 Goal 在执行时拒绝新 Goal（一只海龟一次一个目标）
        if not (0.0 <= goal_request.target_x <= 11.0 and
                0.0 <= goal_request.target_y <= 11.0):
            return GoalResponse.REJECT
        with self.goal_lock_:
            if self.goal_reserved_:
                self.get_logger().warn("Rejecting goal: another move is active")
                return GoalResponse.REJECT
            self.goal_reserved_ = True
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        # 允许取消；实际停车在 execute_callback 里完成
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        # 控制主循环：约 30Hz 计算并发布速度指令，直到到达目标
        self.active_goal_handle_ = goal_handle
        feedback = MoveToPosition.Feedback()
        result = MoveToPosition.Result()
        try:
            while rclpy.ok() and not self.shutting_down_:
                # 被取消：先停车再标记 canceled
                if goal_handle.is_cancel_requested:
                    self.stop_turtle()
                    goal_handle.canceled()
                    return result

                # 还没收到位姿就等下一拍
                if self.pose_ is None:
                    time.sleep(1.0 / 30.0)
                    continue

                # 距离目标足够近（<0.2）即视为到达
                distance = math.hypot(
                    goal_handle.request.target_x - self.pose_.x,
                    goal_handle.request.target_y - self.pose_.y)
                if distance < 0.2:
                    break

                # P 控制：线速度正比于剩余距离（增益 2.0，
                # 上限 max_speed），角速度正比于朝向误差（增益 6.0）
                goal_theta = math.atan2(
                    goal_handle.request.target_y - self.pose_.y,
                    goal_handle.request.target_x - self.pose_.x)
                diff = self.normalize_angle(goal_theta - self.pose_.theta)

                cmd = Twist()
                cmd.linear.x = min(float(self.max_speed_), 2.0 * distance)
                cmd.angular.z = 6.0 * diff

                # 发布速度指令并把当前进度作为 Feedback 发给客户端
                feedback.current_x = self.pose_.x
                feedback.current_y = self.pose_.y
                feedback.distance_remaining = distance
                try:
                    self.cmd_vel_publisher_.publish(cmd)
                    goal_handle.publish_feedback(feedback)
                except Exception:
                    return result
                time.sleep(1.0 / 30.0)

            # 因程序关闭而退出循环：中止当前 Goal
            if self.shutting_down_ or not rclpy.ok():
                self.stop_turtle()
                try:
                    goal_handle.abort()
                except Exception:
                    pass
                return result

            # 正常到达：停车、填写最终位置与误差、标记成功
            self.stop_turtle()
            result.final_x = self.pose_.x
            result.final_y = self.pose_.y
            result.distance_error = math.hypot(
                goal_handle.request.target_x - self.pose_.x,
                goal_handle.request.target_y - self.pose_.y)
            try:
                goal_handle.succeed()
            except Exception:
                return result
            return result
        finally:
            # 无论结局如何都要释放"占用"标记，让后续 Goal 可被接受
            if self.active_goal_handle_ is goal_handle:
                self.active_goal_handle_ = None
            with self.goal_lock_:
                self.goal_reserved_ = False

    def stop_turtle(self):
        # 发布全零 Twist 让海龟立即停止
        try:
            if rclpy.ok():
                self.cmd_vel_publisher_.publish(Twist())
        except Exception:
            pass

    def request_shutdown(self):
        # Ctrl+C 时由 main 调用：停车并中止正在执行的 Goal，
        # 让执行线程尽快退出
        self.shutting_down_ = True
        self.stop_turtle()
        if self.active_goal_handle_ is not None:
            try:
                self.active_goal_handle_.abort()
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
    # 多线程执行器：配合 Reentrant 回调组，让控制循环、
    # 位姿订阅、取消请求能同时得到处理
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        node.request_shutdown()
        time.sleep(0.2)
    finally:
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
