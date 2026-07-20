// ============================================================
// Topic 通信示例:Subscriber(订阅者)节点
// 演示:订阅 /chatter 话题,每收到一条 std_msgs/String 消息就打印出来
// 运行:ros2 run cpp_pubsub subscriber_node
// 配合:先运行 publisher_node,再运行本节点即可看到 "I heard: ..." 输出
// ============================================================

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class SubscriberNode : public rclcpp::Node
{
public:
  SubscriberNode() : Node("subscriber_node")
  {
    // create_subscription:模板参数是消息类型,实参依次为话题名、队列深度、回调函数
    // 消息类型和话题名必须与 Publisher 一致才能建立连接
    // std::placeholders::_1 是占位符,表示"回调被调用时的第 1 个参数"(即收到的消息),
    // 由事件循环在消息到达时填入,std::bind 借此把带参数的成员函数包装成回调
    subscription_ = create_subscription<std_msgs::msg::String>(
      "chatter", 10,
      std::bind(&SubscriberNode::callbackMessage, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Subscriber node started, listening on /chatter");
  }

private:
  // 消息回调:每收到一条消息,spin 的事件循环就调用一次
  // 参数是 SharedPtr,避免拷贝整条消息,用 -> 访问消息字段
  void callbackMessage(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "I heard: \"%s\"", msg->data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // Subscriber 完全靠 spin 事件循环驱动:没有 spin 就永远收不到消息
  rclcpp::spin(std::make_shared<SubscriberNode>());
  rclcpp::shutdown();
  return 0;
}
