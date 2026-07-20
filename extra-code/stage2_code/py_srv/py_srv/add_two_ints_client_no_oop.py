# ============================================================
# Service 通信示例：客户端（非 OOP、"同步式"写法）
# 演示概念：call_async + spin_until_future_complete 组合，
# 发一个请求并原地等待结果，适合一次性的简单调用
# 先启动服务端，再运行：
#   ros2 run py_srv add_two_ints_client_no_oop
# ============================================================
import rclpy
from example_interfaces.srv import AddTwoInts
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def main(args=None):
    rclpy.init(args=args)
    node = Node("add_two_ints_client_no_oop")
    client = node.create_client(AddTwoInts, "add_two_ints")

    try:
        # 先等服务端上线（最多 5 秒），不在线就直接退出
        if not client.wait_for_service(timeout_sec=5.0):
            node.get_logger().error("/add_two_ints service is not available")
            return

        request = AddTwoInts.Request()
        request.a = 3
        request.b = 8
        # ROS 2 没有真正的同步 call（容易死锁），惯用做法是：
        # call_async 拿到 Future，再用 spin_until_future_complete
        # 一边处理事件一边阻塞，直到该 Future 有结果
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        # 到这里 Future 已完成，result() 即服务端的响应
        response = future.result()
        node.get_logger().info(f"{request.a} + {request.b} = {response.sum}")
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        node.get_logger().error(f"Service call failed: {exc}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
