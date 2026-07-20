// ============================================================
// 自定义消息示例:HardwareStatus Subscriber
// 演示:订阅 /hardware_status 话题,解析自定义消息的各字段并打印,
// 电机未就绪时输出 WARN 级警告(演示不同日志级别的用法)
// 运行:先启动 hardware_status_publisher,再
// ros2 run cpp_hardware_tutorial hardware_status_subscriber
// ============================================================

#include <memory>

#include "hardware_interfaces/msg/hardware_status.hpp"
#include "rclcpp/rclcpp.hpp"

class HardwareStatusSubscriber : public rclcpp::Node
{
public:
  HardwareStatusSubscriber() : Node("hardware_status_subscriber")
  {
    // 订阅自定义消息:模板参数和话题名必须与 Publisher 完全一致
    // _1 占位符对应回调的消息参数,由事件循环在消息到达时填入
    subscription_ = create_subscription<hardware_interfaces::msg::HardwareStatus>(
      "hardware_status", 10,
      std::bind(&HardwareStatusSubscriber::callbackStatus, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "HardwareStatus subscriber started");
  }

private:
  void callbackStatus(const hardware_interfaces::msg::HardwareStatus::SharedPtr msg)
  {
    // 消息里的温度是毫摄氏度整数,除以 1000.0 转成摄氏度浮点数便于阅读
    const double temp_celsius = msg->temperature / 1000.0;
    RCLCPP_INFO(
      get_logger(), "Received: %.1f C, motors_ready=%s, debug=\"%s\"",
      temp_celsius, msg->are_motors_ready ? "true" : "false",
      msg->debug_message.c_str());
    // WARN 级日志:终端里会以黄色高亮显示,用于提醒异常但不致命的状态
    if (!msg->are_motors_ready) {
      RCLCPP_WARN(get_logger(), "Motors not ready!");
    }
  }

  rclcpp::Subscription<hardware_interfaces::msg::HardwareStatus>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HardwareStatusSubscriber>());
  rclcpp::shutdown();
  return 0;
}
