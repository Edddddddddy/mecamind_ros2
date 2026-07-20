// ============================================================
// Service 通信示例:服务端(Server)
// 演示:提供 /add_two_ints 服务,收到两个整数 a、b 后返回它们的和
// Service 是"一问一答"模式:Client 发 Request,Server 回 Response(区别于 Topic 的持续广播)
// 运行:ros2 run cpp_srv add_two_ints_server
// 测试:ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 2, b: 3}"
// ============================================================

#include <memory>

// Service 接口头文件:AddTwoInts 定义了 Request(a, b)和 Response(sum)两部分
#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

// 类型别名,后面写 AddTwoInts 就不用重复冗长的完整命名空间
using AddTwoInts = example_interfaces::srv::AddTwoInts;

class AddTwoIntsServer : public rclcpp::Node
{
public:
  AddTwoIntsServer() : Node("add_two_ints_server")
  {
    // create_service:服务名 + 回调。回调有两个参数(request 和 response),
    // 所以 std::bind 需要两个占位符 _1、_2 分别对应它们
    service_ = create_service<AddTwoInts>(
      "add_two_ints",
      std::bind(&AddTwoIntsServer::callback, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(get_logger(), "Add Two Ints server has been started.");
  }

private:
  // Service 回调:框架传入已填好的 request,我们把结果写进 response 即可,
  // 无需手动"发送"——回调返回后框架自动把 response 发回 Client
  void callback(const AddTwoInts::Request::SharedPtr request,
                const AddTwoInts::Response::SharedPtr response)
  {
    response->sum = request->a + request->b;
    RCLCPP_INFO(get_logger(), "%ld + %ld = %ld", request->a, request->b, response->sum);
  }

  rclcpp::Service<AddTwoInts>::SharedPtr service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // Server 靠 spin 事件循环等待请求:请求到达时才触发回调
  rclcpp::spin(std::make_shared<AddTwoIntsServer>());
  rclcpp::shutdown();
  return 0;
}

