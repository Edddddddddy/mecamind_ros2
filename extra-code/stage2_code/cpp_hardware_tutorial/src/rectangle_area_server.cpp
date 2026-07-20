// ============================================================
// 自定义 Service 示例:矩形面积计算服务端
// 演示:使用自己定义的 hardware_interfaces/srv/ComputeRectangleArea 接口
// (Request: length、width;Response: area)提供 /compute_rectangle_area 服务
// 运行:ros2 run cpp_hardware_tutorial rectangle_area_server
// 测试(命令一行输入):ros2 service call /compute_rectangle_area
//       hardware_interfaces/srv/ComputeRectangleArea "{length: 3.0, width: 4.0}"
// ============================================================

#include <memory>

// 自定义 Service 的头文件同样由 rosidl 构建接口包时自动生成
#include "hardware_interfaces/srv/compute_rectangle_area.hpp"
#include "rclcpp/rclcpp.hpp"

using ComputeRectangleArea = hardware_interfaces::srv::ComputeRectangleArea;

class RectangleAreaServer : public rclcpp::Node
{
public:
  RectangleAreaServer() : Node("rectangle_area_server")
  {
    // Service 回调有 request、response 两个参数,std::bind 用 _1、_2 两个占位符对应
    service_ = create_service<ComputeRectangleArea>(
      "compute_rectangle_area",
      std::bind(&RectangleAreaServer::callbackArea, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(get_logger(), "Rectangle area service ready on /compute_rectangle_area");
  }

private:
  // 读取 request 字段计算面积,写入 response;回调返回后框架自动把结果发回 Client
  void callbackArea(
    const ComputeRectangleArea::Request::SharedPtr request,
    const ComputeRectangleArea::Response::SharedPtr response)
  {
    response->area = request->length * request->width;
    RCLCPP_INFO(
      get_logger(), "%.2f x %.2f = %.2f",
      request->length, request->width, response->area);
  }

  rclcpp::Service<ComputeRectangleArea>::SharedPtr service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RectangleAreaServer>());
  rclcpp::shutdown();
  return 0;
}
