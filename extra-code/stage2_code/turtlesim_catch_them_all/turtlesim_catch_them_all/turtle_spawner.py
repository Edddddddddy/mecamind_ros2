#!/usr/bin/env python3
# turtle_spawner.py —— P1 项目的海龟生成/管理节点
# 职责：定时调用 turtlesim 的 /spawn 生成随机海龟，
# 维护存活列表并发布到 alive_turtles 话题；同时提供
# catch_turtle Service，被抓的海龟通过 /kill 从屏幕移除。
# 运行：ros2 run turtlesim_catch_them_all spawner
from functools import partial
import math
import random

from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from turtlesim.srv import Kill
from turtlesim.srv import Spawn


class TurtleSpawnerNode(Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        # 三个可调参数：名字前缀、生成频率（Hz）、存活上限
        self.declare_parameter("turtle_name_prefix", "turtle")
        self.declare_parameter("spawn_frequency", 1.0)
        self.declare_parameter("max_alive_turtles", 8)

        self.turtle_name_prefix_ = self.get_parameter("turtle_name_prefix").value
        self.spawn_frequency_ = self.get_parameter("spawn_frequency").value
        self.max_alive_turtles_ = self.get_parameter("max_alive_turtles").value
        self.turtle_counter_ = 1
        self.alive_turtles_ = []
        # 发布存活海龟列表，controller 靠它来选目标
        self.alive_turtles_publisher_ = self.create_publisher(
            TurtleArray, "alive_turtles", 10)
        # 作为客户端调用 turtlesim 自带的 /spawn 和 /kill 服务
        self.spawn_client_ = self.create_client(Spawn, "/spawn")
        self.kill_client_ = self.create_client(Kill, "/kill")
        # 作为服务端对外提供 catch_turtle 服务（供 controller 调用）
        self.catch_turtle_service_ = self.create_service(
            CatchTurtle, "catch_turtle", self.callback_catch_turtle)
        # 定时器周期 = 1/频率，到点就尝试生成一只新海龟
        self.spawn_turtle_timer_ = self.create_timer(
            1.0 / self.spawn_frequency_, self.spawn_new_turtle)

    def callback_catch_turtle(
            self, request: CatchTurtle.Request, response: CatchTurtle.Response):
        # catch_turtle 服务的处理逻辑：按名字找到目标海龟，
        # 先安排 /kill 把它从屏幕移除，成功后才从存活列表删除
        # 并重新发布列表；找不到或 kill 失败则返回 success=False
        for i, turtle in enumerate(self.alive_turtles_):
            if turtle.name == request.name:
                if not self.call_kill_service(request.name):
                    response.success = False
                    return response
                del self.alive_turtles_[i]
                self.publish_alive_turtles()
                response.success = True
                return response

        response.success = False
        return response

    def publish_alive_turtles(self):
        # 把当前存活列表打包成 TurtleArray 发布出去
        msg = TurtleArray()
        msg.turtles = self.alive_turtles_
        self.alive_turtles_publisher_.publish(msg)

    def spawn_new_turtle(self):
        # 定时器回调：达到存活上限就跳过，避免屏幕上海龟过多
        if len(self.alive_turtles_) >= self.max_alive_turtles_:
            return

        # 生成随机位置和朝向（turtle1 已存在，编号从 2 开始）
        self.turtle_counter_ += 1
        name = self.turtle_name_prefix_ + str(self.turtle_counter_)
        x = random.uniform(1.0, 10.0)
        y = random.uniform(1.0, 10.0)
        theta = random.uniform(0.0, 2 * math.pi)
        self.call_spawn_service(name, x, y, theta)

    def call_spawn_service(self, turtle_name, x, y, theta):
        # 异步调用 /spawn，不阻塞定时器；结果在回调里处理
        if not self.spawn_client_.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("spawn service is not available")
            return

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = turtle_name

        future = self.spawn_client_.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_spawn_service, request=request))

    def callback_call_spawn_service(self, future, request: Spawn.Request):
        # /spawn 成功会返回海龟名字；此时才把它加入存活列表并发布
        response: Spawn.Response = future.result()
        if response.name != "":
            self.get_logger().info("New alive turtle: " + response.name)
            new_turtle = Turtle()
            new_turtle.name = response.name
            new_turtle.x = request.x
            new_turtle.y = request.y
            new_turtle.theta = request.theta
            self.alive_turtles_.append(new_turtle)
            self.publish_alive_turtles()

    def call_kill_service(self, turtle_name):
        # 安排调用 /kill；返回 True 只代表请求已发出，
        # 实际结果在 callback_call_kill_service 里打印
        if not self.kill_client_.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("kill service is not available")
            return False

        request = Kill.Request()
        request.name = turtle_name

        future = self.kill_client_.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_kill_service, turtle_name=turtle_name))
        return True

    def callback_call_kill_service(self, future, turtle_name):
        try:
            future.result()
            self.get_logger().info(f"Killed turtle: {turtle_name}")
        except Exception as exc:
            self.get_logger().error(f"Failed to kill turtle {turtle_name}: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode()
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
