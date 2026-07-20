# ============================================================
# Service 通信示例：服务端（Server）
# 演示概念：create_service、request/response 请求-响应模型
# 提供 /add_two_ints 服务：收到两个整数 a、b，返回它们的和
# 运行方式：ros2 run py_srv add_two_ints_server
# ============================================================
import rclpy
# AddTwoInts 接口定义：Request 含 a、b 两个整数，
# Response 含 sum 一个整数
from example_interfaces.srv import AddTwoInts
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class AddTwoIntsServer(Node):
    """Provide the /add_two_ints service."""

    def __init__(self):
        super().__init__("add_two_ints_server")
        # 创建 Service：参数是接口类型、服务名、处理回调；
        # 之后客户端每发一次请求就触发一次回调
        self.service_ = self.create_service(
            AddTwoInts, "add_two_ints", self.callback_add_two_ints)
        self.get_logger().info("Add Two Ints server has been started.")

    def callback_add_two_ints(self, request, response):
        # Service 回调固定签名：收到 request，填好 response
        # 并 return，框架会把它发回给客户端
        response.sum = request.a + request.b
        self.get_logger().info(f"{request.a} + {request.b} = {response.sum}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServer()
    # 服务端必须一直 spin 才能持续响应客户端请求
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
