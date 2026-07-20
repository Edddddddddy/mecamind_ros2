// ============================================================
// 面向对象(OOP)写法的 C++ ROS 2 节点
// 演示:继承 rclcpp::Node 编写节点类 + 定时器回调,这是 ROS 2 官方推荐的标准写法
// 每秒打印一次 "Hello N",N 递增
// 运行:ros2 run cpp_first_pkg cpp_oop
// ============================================================

#include "rclcpp/rclcpp.hpp"

// 自定义节点类继承 rclcpp::Node,这样类内部可以直接使用节点的全部功能
// (创建定时器、Publisher、日志等),相当于 Python 里 class OopNode(Node)
class OopNode : public rclcpp::Node
{
public:
  // 冒号后面是 C++ 的"构造函数初始化列表":先调用父类构造函数给节点命名为 "cpp_oop",
  // 再把成员变量 counter_ 初始化为 0(比在函数体内赋值更高效,也是初始化父类的唯一方式)
  OopNode() : Node("cpp_oop"), counter_(0)
  {
    RCLCPP_INFO(this->get_logger(), "OOP node started");
    // create_wall_timer 创建定时器:每 1 秒触发一次回调
    // std::bind 把成员函数 timer_callback 和当前对象 this 绑定成一个可调用对象,
    // 因为成员函数必须依附于某个对象才能被调用(Python 中方法自带 self,C++ 需要显式绑定)
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&OopNode::timer_callback, this));
  }

private:
  // 定时器回调:spin 的事件循环每秒调用一次这个函数
  void timer_callback()
  {
    RCLCPP_INFO(this->get_logger(), "Hello %d", counter_++);
  }

  // 定时器必须保存为成员变量(SharedPtr),否则构造函数结束后就被销毁,回调不会触发
  rclcpp::TimerBase::SharedPtr timer_;
  int counter_;
};

int main(int argc, char * argv[])
{
  // 初始化 ROS 2 -> 创建节点 -> spin 进入事件循环处理回调 -> Ctrl+C 后清理退出
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OopNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
