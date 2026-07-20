// ============================================================
// Action 通信示例:CountUntil 服务端(Action Server)
// 演示:Action 的三段式接口 goal(target_number, period)/ feedback(current_number)
// / result(reached_number)。Action 适合"执行时间长、要汇报进度、可中途取消"的任务
// 收到目标后每隔 period 秒数一个数,边数边发 feedback,数到 target_number 返回 result
// 运行:ros2 run cpp_action_tutorial count_until_server
// ============================================================

#include <chrono>
#include <memory>
#include <thread>

// 自定义 Action 接口的头文件(由 count_until_interfaces 包构建时生成)
#include "count_until_interfaces/action/count_until.hpp"
#include "rclcpp/rclcpp.hpp"
// Action 的 API 在独立的 rclcpp_action 库中,不在 rclcpp 里
#include "rclcpp_action/rclcpp_action.hpp"

using CountUntil = count_until_interfaces::action::CountUntil;
// GoalHandle(目标句柄)代表服务端正在处理的一个 goal,
// 通过它发 feedback、查询取消状态、宣布成功/失败
using GoalHandleCountUntil = rclcpp_action::ServerGoalHandle<CountUntil>;

class CountUntilServer : public rclcpp::Node
{
public:
  CountUntilServer() : Node("count_until_server")
  {
    // Action Server 需要注册一组(共 3 个)回调,分别响应 goal 生命周期的不同阶段:
    //   handleGoal     - 收到新 goal 时决定接受还是拒绝(_1 是 goal 的 UUID,_2 是 goal 内容)
    //   handleCancel   - 客户端请求取消时决定是否允许
    //   handleAccepted - goal 被接受后开始真正执行
    server_ = rclcpp_action::create_server<CountUntil>(
      this,
      "count_until",
      std::bind(&CountUntilServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&CountUntilServer::handleCancel, this, std::placeholders::_1),
      std::bind(&CountUntilServer::handleAccepted, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Action Server ready on /count_until");
  }

private:
  // 目标审核回调:先校验参数合法性,不合法直接 REJECT,合法则接受并立即执行
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const CountUntil::Goal> goal)
  {
    if (goal->target_number <= 0) {
      RCLCPP_WARN(get_logger(), "Rejecting goal: target_number must be positive");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->period <= 0.0f) {
      RCLCPP_WARN(get_logger(), "Rejecting goal: period must be positive");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // 取消审核回调:本示例无条件允许取消(实际项目中可根据任务状态拒绝)
  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleCountUntil>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // goal 被接受后:在新线程中执行耗时任务,避免阻塞 spin 的事件循环
  // (回调里长时间阻塞会导致节点无法响应其他消息)detach 表示线程独立运行不必等待
  void handleAccepted(const std::shared_ptr<GoalHandleCountUntil> goal_handle)
  {
    std::thread{std::bind(&CountUntilServer::execute, this, goal_handle)}.detach();
  }

  // 真正的任务执行:循环数数,每一步发 feedback,期间随时检查是否被取消
  void execute(const std::shared_ptr<GoalHandleCountUntil> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<CountUntil::Feedback>();
    auto result = std::make_shared<CountUntil::Result>();
    // 用 std::max 给周期兜底,最小 0.1 秒,防止过快的循环
    auto period = std::chrono::duration<double>(std::max(0.1f, goal->period));

    for (int64_t number = 1; number <= goal->target_number; ++number) {
      // 客户端可能中途请求取消:确认后调用 canceled() 结束 goal 并返回部分结果
      if (goal_handle->is_canceling()) {
        result->reached_number = number - 1;
        goal_handle->canceled(result);
        return;
      }
      // publish_feedback 把当前进度实时推送给客户端(Action 特有,Service 做不到)
      feedback->current_number = number;
      goal_handle->publish_feedback(feedback);
      std::this_thread::sleep_for(period);
    }

    // 任务完成:succeed() 把最终 result 发给客户端并标记 goal 成功
    result->reached_number = goal->target_number;
    goal_handle->succeed(result);
  }

  rclcpp_action::Server<CountUntil>::SharedPtr server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // spin 处理 goal 请求、取消请求等事件;耗时执行已放到独立线程,不会卡住事件循环
  rclcpp::spin(std::make_shared<CountUntilServer>());
  rclcpp::shutdown();
  return 0;
}
