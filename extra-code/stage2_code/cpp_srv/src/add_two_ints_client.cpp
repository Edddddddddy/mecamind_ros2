// ============================================================
// Service 通信示例:客户端(Client,面向对象 + 异步回调写法)
// 演示:连续发出 3 个 AddTwoInts 请求,用回调异步接收结果,全部完成后自动退出
// 运行:先启动 add_two_ints_server,再 ros2 run cpp_srv add_two_ints_client
// 对比:add_two_ints_client_no_oop.cpp 是更简单的"发一次等一次"写法
// ============================================================

#include <chrono>
#include <exception>
#include <memory>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

// 引入时间字面量,使 5s 表示 5 秒
using namespace std::chrono_literals;

class AddTwoIntsClient : public rclcpp::Node
{
public:
  AddTwoIntsClient() : Node("add_two_ints_client")
  {
    // create_client 只是创建客户端句柄,并不保证服务端已经在线
    client_ = create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
    pending_requests_ = 0;
  }

  // 发请求前先确认服务端在线:wait_for_service 最多阻塞 5 秒,超时返回 false
  bool waitForService()
  {
    if (client_->wait_for_service(5s)) {
      return true;
    }
    RCLCPP_ERROR(get_logger(), "/add_two_ints service is not available");
    return false;
  }

  // 异步发送一个请求:async_send_request 立即返回不等结果,
  // 结果到达时由 spin 的事件循环调用我们注册的 lambda 回调
  void callAddTwoInts(int64_t a, int64_t b)
  {
    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a = a;
    request->b = b;
    pending_requests_++;
    // 第二个参数是 lambda 回调,[this, request] 是捕获列表:
    // 捕获 this 才能在 lambda 里用成员变量,捕获 request 是为了打印时还能拿到原始参数
    // 回调参数 SharedFuture 代表"将来才有的结果",future.get() 取出 Response
    client_->async_send_request(
      request,
      [this, request](rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future) {
        try {
          RCLCPP_INFO(get_logger(), "%ld + %ld = %ld", request->a, request->b, future.get()->sum);
        } catch (const std::exception & error) {
          RCLCPP_ERROR(get_logger(), "Service call failed: %s", error.what());
        }
        // 用计数器跟踪未完成的请求,全部收到回复后主动 shutdown 让 main 里的 spin 退出
        pending_requests_--;
        if (pending_requests_ == 0) {
          RCLCPP_INFO(get_logger(), "All requests completed.");
          rclcpp::shutdown();
        }
      });
  }

private:
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
  int pending_requests_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AddTwoIntsClient>();
  if (!node->waitForService()) {
    rclcpp::shutdown();
    return 1;
  }
  // 三个请求几乎同时发出(异步的优势:不用发一个等一个)
  node->callAddTwoInts(2, 7);
  node->callAddTwoInts(1, 4);
  node->callAddTwoInts(10, 20);
  // 必须 spin 才能收到回复:回调都在事件循环中执行
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
