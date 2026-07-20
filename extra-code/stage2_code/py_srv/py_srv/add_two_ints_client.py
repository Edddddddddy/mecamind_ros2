# ============================================================
# Service 通信示例：异步客户端（OOP 写法）
# 演示概念：call_async 异步调用、Future 回调、多请求并发
# 连续发 3 个请求而不阻塞等待，收齐全部响应后自动退出
# 先启动服务端，再运行：ros2 run py_srv add_two_ints_client
# ============================================================
# partial 用于给回调函数预先绑定额外参数（这里是 request）
from functools import partial

import rclpy
from example_interfaces.srv import AddTwoInts
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class AddTwoIntsClient(Node):
    """Asynchronous OOP AddTwoInts client that exits after all replies arrive."""

    def __init__(self):
        super().__init__("add_two_ints_client")
        # 创建 Client：接口类型和服务名必须与服务端一致
        self.client_ = self.create_client(AddTwoInts, "add_two_ints")
        # 记录未完成的请求数，用于判断何时全部完成
        self.pending_requests_ = 0

    def wait_for_service(self):
        # 先确认服务端在线（最多等 5 秒），
        # 否则请求会一直挂着得不到响应
        if self.client_.wait_for_service(timeout_sec=5.0):
            return True
        self.get_logger().error("/add_two_ints service is not available")
        return False

    def call_add_two_ints(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        # call_async 不阻塞：立即返回一个 Future（未来结果的
        # 占位符），响应到达后由 spin 触发 done 回调
        future = self.client_.call_async(request)
        self.pending_requests_ += 1
        # 用 partial 把 request 一起传进回调，
        # 这样打印结果时能知道对应的是哪个请求
        future.add_done_callback(
            partial(self.callback_call_add_two_ints, request=request))

    def callback_call_add_two_ints(self, future, request):
        # Future 完成时被调用：result() 取出响应，
        # 若服务端出错则会抛异常
        try:
            response = future.result()
            self.get_logger().info(f"{request.a} + {request.b} = {response.sum}")
        except Exception as exc:
            self.get_logger().error(
                f"Service call {request.a} + {request.b} failed: {exc}")
        finally:
            # 所有请求都完成后主动 shutdown，让 spin 退出
            self.pending_requests_ -= 1
            if self.pending_requests_ == 0:
                self.get_logger().info("All requests completed.")
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClient()
    try:
        if node.wait_for_service():
            # 连续发出 3 个异步请求（互不等待），
            # 然后 spin 等待响应回调逐个触发
            node.call_add_two_ints(2, 7)
            node.call_add_two_ints(1, 4)
            node.call_add_two_ints(10, 20)
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
