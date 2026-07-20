// ============================================================
// Parameter(参数)示例:可动态调节频率和内容的 Publisher
// 演示:declare/get parameter、启动时传参、运行中用 ros2 param set 动态修改,
// 以及两种参数回调:on_set(修改前校验,可否决)和 post_set(修改后生效)
// 运行:ros2 run my_cpp_pkg param_publisher --ros-args -p frequency:=2.0
// 动态修改:ros2 param set /param_publisher frequency 5.0
// ============================================================

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

// 参数校验回调需要返回这个消息类型,表示"是否允许本次修改"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class ParamPublisherNode : public rclcpp::Node
{
public:
  ParamPublisherNode()
  : Node("param_publisher")
  {
    // declare_parameter 声明参数并给默认值:未声明的参数外部无法读写
    // 默认值的类型同时决定了参数类型(1.0 是 double,"Hello ROS 2" 是 string)
    this->declare_parameter("frequency", 1.0);
    this->declare_parameter("message", "Hello ROS 2");

    // 读取参数实际值(可能来自命令行 -p 或 YAML 文件,没传就用默认值)
    frequency_ = this->get_parameter("frequency").as_double();
    message_ = this->get_parameter("message").as_string();
    // 启动时的非法值直接抛异常终止:参数回调只拦截"运行中"的修改,管不到启动值
    if (frequency_ <= 0.0) {
      throw std::invalid_argument("frequency must be positive");
    }
    publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
    // 定时器周期由参数决定:频率 frequency Hz 对应周期 1/frequency 秒
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / frequency_),
      std::bind(&ParamPublisherNode::timer_callback, this));
    // 注册两种参数回调(返回的 handle 必须保存,否则回调立即失效):
    //   on_set  - 参数被修改"之前"调用,用于校验,返回 false 可拒绝本次修改
    //   post_set - 修改已生效"之后"调用,用于应用新值(如重建定时器)
    on_set_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&ParamPublisherNode::on_set_parameters, this, std::placeholders::_1));
    post_set_callback_handle_ = this->add_post_set_parameters_callback(
      std::bind(&ParamPublisherNode::post_set_parameters, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(), "Param publisher started: frequency=%.2f, message=\"%s\"",
      frequency_, message_.c_str());
  }

private:
  // 定时器回调:按当前参数指定的内容发布消息
  void timer_callback()
  {
    auto msg = std_msgs::msg::String();
    msg.data = message_;
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Publishing: \"%s\"", msg.data.c_str());
  }

  // 修改前校验回调:一次可能同时修改多个参数,所以收到的是 vector
  // 只要发现 frequency 非正数就返回 successful=false,整批修改都会被拒绝
  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    for (const auto & parameter : parameters) {
      if (parameter.get_name() == "frequency") {
        const double value = parameter.as_double();
        if (value <= 0.0) {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = false;
          result.reason = "frequency must be positive";
          return result;
        }
      }
    }

    // 校验通过:允许修改生效
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  // 修改后生效回调:此时新值已通过校验,把它们同步到成员变量
  void post_set_parameters(const std::vector<rclcpp::Parameter> & parameters)
  {
    bool frequency_changed = false;
    for (const auto & parameter : parameters) {
      if (parameter.get_name() == "frequency") {
        frequency_ = parameter.as_double();
        frequency_changed = true;
      } else if (parameter.get_name() == "message") {
        message_ = parameter.as_string();
      }
    }

    // 定时器创建后周期不可改,只能取消旧定时器再按新频率重建一个
    if (frequency_changed) {
      timer_->cancel();
      timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / frequency_),
        std::bind(&ParamPublisherNode::timer_callback, this));
    }

    RCLCPP_INFO(
      this->get_logger(), "Parameters updated: frequency=%.2f, message=\"%s\"",
      frequency_, message_.c_str());
  }

  double frequency_;
  std::string message_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  // 两个回调 handle:节点存活期间必须持有,handle 销毁则对应回调自动注销
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_callback_handle_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_set_callback_handle_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // spin 事件循环同时处理定时器回调和外部的参数读写请求
  rclcpp::spin(std::make_shared<ParamPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
