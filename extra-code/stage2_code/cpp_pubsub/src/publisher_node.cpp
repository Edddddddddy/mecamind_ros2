// ============================================================
// Topic 通信示例:Publisher(发布者)节点
// 演示:向 /chatter 话题以 1 Hz 频率发布 std_msgs/String 消息
// 运行:ros2 run cpp_pubsub publisher_node
// 配合:另开终端运行 subscriber_node 接收,或 ros2 topic echo /chatter 查看
// ============================================================

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
// 每种消息类型对应一个头文件:std_msgs/msg/String 消息 -> std_msgs/msg/string.hpp
#include "std_msgs/msg/string.hpp"

// 引入时间字面量,让下面可以直接写 1s 表示 1 秒(还支持 500ms、2min 等写法)
using namespace std::chrono_literals;

class PublisherNode : public rclcpp::Node
{
public:
  PublisherNode() : Node("publisher_node"), counter_(0)
  {
    // create_publisher 的模板参数 <std_msgs::msg::String> 指定消息类型,
    // 两个实参分别是话题名 "chatter" 和队列深度 10(网络繁忙时最多缓存 10 条待发消息)
    publisher_ = create_publisher<std_msgs::msg::String>("chatter", 10);
    // ROS 2 中周期性发布靠定时器驱动:每 1 秒由 spin 调用一次 publishMessage
    timer_ = create_wall_timer(1s, std::bind(&PublisherNode::publishMessage, this));
    RCLCPP_INFO(get_logger(), "Publisher node started, publishing at 1 Hz");
  }

private:
  // 定时器回调:构造一条消息并发布出去
  void publishMessage()
  {
    // 消息就是一个普通 C++ 对象,String 类型只有一个 data 字段(std::string)
    auto msg = std_msgs::msg::String();
    msg.data = "Hello ROS 2! [" + std::to_string(counter_) + "]";
    publisher_->publish(msg);
    // 日志用 printf 风格格式化,%s 需要 C 字符串,所以对 std::string 调用 .c_str()
    RCLCPP_INFO(get_logger(), "Publishing: \"%s\"", msg.data.c_str());
    counter_++;
  }

  // Publisher 和定时器都保存为 SharedPtr 成员变量,保证节点存活期间它们一直有效
  int counter_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // spin 进入事件循环,持续处理定时器回调,直到 Ctrl+C
  rclcpp::spin(std::make_shared<PublisherNode>());
  rclcpp::shutdown();
  return 0;
}
