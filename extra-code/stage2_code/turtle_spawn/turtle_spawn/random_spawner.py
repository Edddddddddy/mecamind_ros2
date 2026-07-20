# random_spawner.py —— Service 客户端入门示例：随机生成海龟
# 用定时器每 2 秒异步调用一次 turtlesim 的 /spawn 服务，
# 在随机位置生成一只新海龟。演示 call_async + 回调的标准写法。
# 运行（需先启动 turtlesim）：ros2 run turtle_spawn random_spawner
import math
import random
from functools import partial

import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn


class RandomSpawnerNode(Node):
    """Spawn one new turtle every two seconds using the /spawn service."""

    def __init__(self):
        super().__init__("random_spawner")
        self.counter_ = 0
        # Service 客户端：服务类型 Spawn，服务名 /spawn
        self.spawn_client_ = self.create_client(Spawn, "/spawn")
        self.waiting_for_service_ = False
        # 定时器：每 2 秒触发一次生成逻辑
        self.timer_ = self.create_timer(2.0, self.spawn_random_turtle)
        self.get_logger().info("Random spawner started, spawning every 2 seconds")

    def spawn_random_turtle(self):
        # 随机生成名字、位置和朝向（turtle1 已存在，编号从 2 起）
        self.counter_ += 1
        name = f"turtle{self.counter_ + 1}"
        x = random.uniform(0.5, 10.5)
        y = random.uniform(0.5, 10.5)
        theta = random.uniform(0.0, 2.0 * math.pi)

        # 服务还没上线就先跳过本轮（只在第一次打印提示）
        if not self.spawn_client_.wait_for_service(timeout_sec=0.1):
            if not self.waiting_for_service_:
                self.get_logger().warn("Waiting for /spawn service...")
                self.waiting_for_service_ = True
            return
        self.waiting_for_service_ = False

        # 填写请求并异步调用：call_async 不阻塞定时器，
        # 结果通过 done_callback 通知；partial 用来附带额外参数
        request = Spawn.Request()
        request.name = name
        request.x = x
        request.y = y
        request.theta = theta
        future = self.spawn_client_.call_async(request)
        future.add_done_callback(partial(self.callback_spawn, name=name, x=x, y=y))

    def callback_spawn(self, future, name, x, y):
        # /spawn 成功时返回的名字非空
        response = future.result()
        if response.name:
            self.get_logger().info(
                f"Spawned '{response.name}' at ({x:.1f}, {y:.1f})")
        else:
            self.get_logger().error(f"Failed to spawn '{name}'")


def main(args=None):
    rclpy.init(args=args)
    node = RandomSpawnerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.timer_.cancel()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
