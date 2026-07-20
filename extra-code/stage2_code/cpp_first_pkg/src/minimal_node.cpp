// ============================================================
// 最小 C++ ROS 2 节点(非面向对象写法)
// 演示:一个节点程序的最基本骨架 init -> 创建节点 -> spin -> shutdown
// 运行:ros2 run cpp_first_pkg cpp_minimal
// 对比:oop_node.cpp 是同样功能的面向对象(推荐)写法
// ============================================================

// rclcpp 是 ROS 2 的 C++ 客户端库,相当于 Python 里的 rclpy
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  // 初始化 ROS 2 通信层,必须在创建任何节点之前调用(相当于 rclpy.init())
  rclcpp::init(argc, argv);
  // make_shared 创建一个 SharedPtr(共享智能指针)管理的节点对象,名字叫 "cpp_minimal"
  // ROS 2 的 C++ API 普遍用 SharedPtr:自动管理内存,多处共享同一对象,无需手动 delete
  auto node = rclcpp::Node::make_shared("cpp_minimal");
  // RCLCPP_INFO 是 ROS 2 的日志宏,输出带时间戳和节点名的 INFO 级日志
  RCLCPP_INFO(node->get_logger(), "Hello from cpp_first_pkg!");
  // spin 进入事件循环:阻塞在这里等待并处理回调(定时器、消息等),直到 Ctrl+C
  rclcpp::spin(node);
  // 收到 Ctrl+C 后 spin 返回,这里做清理释放 ROS 2 资源
  rclcpp::shutdown();
  return 0;
}
