# ============================================================
# 自定义服务接口示例：服务端（Server）
# 演示概念：自定义 srv（ComputeRectangleArea）的
# request/response 模型
# 提供 /compute_rectangle_area 服务：输入长和宽，返回面积
# 运行方式：
#   ros2 run py_hardware_tutorial rectangle_area_server
# ============================================================
import rclpy
# 自定义服务接口：Request 含 length、width，Response 含 area
from hardware_interfaces.srv import ComputeRectangleArea
from rclpy.node import Node


class RectangleAreaServer(Node):
    def __init__(self):
        super().__init__("rectangle_area_server")
        # 创建 Service：接口类型、服务名、处理回调
        self.service_ = self.create_service(
            ComputeRectangleArea, "compute_rectangle_area", self.callback_area)
        self.get_logger().info("Rectangle area service ready on /compute_rectangle_area")

    def callback_area(self, request, response):
        # 从 request 取输入，算好结果填进 response 再返回，
        # 框架负责把 response 发回客户端
        response.area = request.length * request.width
        self.get_logger().info(
            f"{request.length:.2f} x {request.width:.2f} = {response.area:.2f}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RectangleAreaServer()
    # 服务端必须一直 spin 才能持续响应请求
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
