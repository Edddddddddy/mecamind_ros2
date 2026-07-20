// ============================================================
// Action 通信示例:CountUntil 客户端(Action Client)
// 演示:发送 goal(数到 5,每 0.5 秒一步),通过 feedback 回调实时接收进度,
// result 回调拿到最终结果后退出
// 运行:先启动 count_until_server,再 ros2 run cpp_action_tutorial count_until_client
// ============================================================

#include <memory>
#include <chrono>

#include "count_until_interfaces/action/count_until.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using CountUntil = count_until_interfaces::action::CountUntil;
// 客户端侧的 GoalHandle,代表"已发出的那个 goal",可用于取消等操作
using GoalHandleCountUntil = rclcpp_action::ClientGoalHandle<CountUntil>;
using namespace std::chrono_literals;

class CountUntilClient : public rclcpp::Node
{
public:
  CountUntilClient() : Node("count_until_client")
  {
    // Action Client 也来自 rclcpp_action,名字必须与服务端的 "count_until" 一致
    client_ = rclcpp_action::create_client<CountUntil>(this, "count_until");
  }

  void sendGoal()
  {
    // 发 goal 前先确认服务端在线,最多等 5 秒
    RCLCPP_INFO(get_logger(), "Waiting for action server...");
    if (!client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(get_logger(), "Action server not available");
      rclcpp::shutdown();
      return;
    }

    // 填写 goal:数到 5,每步间隔 0.5 秒
    CountUntil::Goal goal;
    goal.target_number = 5;
    goal.period = 0.5;

    // SendGoalOptions 用来注册客户端侧的回调,这里用 lambda(匿名函数)而不是 std::bind,
    // [this] 捕获当前对象以便在 lambda 里调用 get_logger() 等成员
    rclcpp_action::Client<CountUntil>::SendGoalOptions options;
    // feedback 回调:服务端每 publish_feedback 一次就触发一次,实时显示进度
    options.feedback_callback = [this](
      GoalHandleCountUntil::SharedPtr,
      const std::shared_ptr<const CountUntil::Feedback> feedback) {
        RCLCPP_INFO(get_logger(), "Current number: %ld", feedback->current_number);
      };
    // result 回调:goal 结束(成功/取消/中止)时触发一次,拿到最终结果后关闭节点
    options.result_callback = [this](const GoalHandleCountUntil::WrappedResult & result) {
        if (result.result) {
          RCLCPP_INFO(get_logger(), "Reached number: %ld", result.result->reached_number);
        } else {
          RCLCPP_ERROR(get_logger(), "Goal failed without result");
        }
        rclcpp::shutdown();
      };

    // 异步发送 goal,立即返回;后续 feedback/result 都由 spin 的事件循环调用
    client_->async_send_goal(goal, options);
  }

private:
  rclcpp_action::Client<CountUntil>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CountUntilClient>();
  node->sendGoal();
  // spin 保持节点运行以接收 feedback 和 result;result 回调里的 shutdown 会让 spin 退出
  rclcpp::spin(node);
  return 0;
}
