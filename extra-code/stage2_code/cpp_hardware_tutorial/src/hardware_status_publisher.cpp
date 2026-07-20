// ============================================================
// 自定义消息示例:HardwareStatus Publisher
// 演示:使用自己定义的 hardware_interfaces/msg/HardwareStatus 消息类型
// (包含 temperature、are_motors_ready、debug_message 三个字段)发布模拟硬件状态
// 温度每秒上升 0.5 度,超过 80 度后 are_motors_ready 变为 false
// 运行:ros2 run cpp_hardware_tutorial hardware_status_publisher
// ============================================================

#include <chrono>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

// 自定义消息的头文件由 rosidl 在构建 hardware_interfaces 包时自动生成,
// 命名规则:包名/msg/消息名(蛇形命名).hpp
#include "hardware_interfaces/msg/hardware_status.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class HardwareStatusPublisher : public rclcpp::Node
{
public:
  // 初始温度 42000(消息里温度用"毫摄氏度"整数存储,即 42.0 C)
  HardwareStatusPublisher() : Node("hardware_status_publisher"), temperature_(42000)
  {
    // 用法和内置消息完全一样,只是模板参数换成了自定义消息类型
    publisher_ = create_publisher<hardware_interfaces::msg::HardwareStatus>(
      "hardware_status", 10);
    timer_ = create_wall_timer(1s, std::bind(&HardwareStatusPublisher::publishStatus, this));
    RCLCPP_INFO(get_logger(), "HardwareStatus publisher started");
  }

private:
  // 定时器回调:填充自定义消息的三个字段并发布
  void publishStatus()
  {
    auto msg = hardware_interfaces::msg::HardwareStatus();
    msg.temperature = temperature_;
    // 阈值 80000 毫摄氏度 = 80.0 C,超过就认为电机未就绪
    msg.are_motors_ready = temperature_ < 80000;
    // ostringstream 是 C++ 里拼接格式化字符串的常用方式(类似 Python 的 f-string)
    std::ostringstream debug_stream;
    debug_stream << "Temperature: " << std::fixed << std::setprecision(1)
                 << temperature_ / 1000.0 << " C";
    msg.debug_message = debug_stream.str();
    publisher_->publish(msg);
    RCLCPP_INFO(
      get_logger(), "Publishing: temp=%ld, ready=%s, msg=\"%s\"",
      msg.temperature, msg.are_motors_ready ? "true" : "false",
      msg.debug_message.c_str());
    // 每次发布后温度上升 500 毫摄氏度(0.5 C),模拟硬件逐渐升温
    temperature_ += 500;
  }

  int64_t temperature_;
  rclcpp::Publisher<hardware_interfaces::msg::HardwareStatus>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HardwareStatusPublisher>());
  rclcpp::shutdown();
  return 0;
}
