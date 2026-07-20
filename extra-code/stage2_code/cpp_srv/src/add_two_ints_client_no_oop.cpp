// ============================================================
// Service 通信示例:客户端(非面向对象的最简写法)
// 演示:发一次 AddTwoInts 请求,用 spin_until_future_complete 同步等待结果后退出
// 适合"调用一次就结束"的命令行小工具;长期运行的节点建议用 OOP + 回调写法
// 运行:先启动 add_two_ints_server,再 ros2 run cpp_srv add_two_ints_client_no_oop
// ============================================================

#include <chrono>
#include <memory>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // 不写节点类,直接创建一个通用 Node 对象,再在它上面创建 Client
  auto node = std::make_shared<rclcpp::Node>("add_two_ints_client_no_oop");
  auto client = node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

  // 最多等 5 秒确认服务端在线,不在线就报错退出
  if (!client->wait_for_service(5s)) {
    RCLCPP_ERROR(node->get_logger(), "/add_two_ints service is not available");
    rclcpp::shutdown();
    return 1;
  }

  // Request 用 make_shared 创建(框架要求传 SharedPtr),填入两个加数
  auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  request->a = 3;
  request->b = 8;
  // async_send_request 立即返回一个 future("未来的结果"占位符),此时结果还没到
  auto future = client->async_send_request(request);

  // spin_until_future_complete:一边跑事件循环一边等 future 出结果,最多等 5 秒
  // 不能只用 future.wait() 干等——不 spin 的话回复永远不会被处理,会死锁
  if (rclcpp::spin_until_future_complete(node, future, 5s) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    // future.get() 取出 Response,访问其中的 sum 字段
    RCLCPP_INFO(node->get_logger(), "%ld + %ld = %ld", request->a, request->b, future.get()->sum);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Service call timed out or was interrupted");
  }

  rclcpp::shutdown();
  return 0;
}
