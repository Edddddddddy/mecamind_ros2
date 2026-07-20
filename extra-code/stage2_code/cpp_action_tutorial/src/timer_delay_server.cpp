// ============================================================
// Action 通信示例:TimerDelay 服务端(定时等待任务)
// 演示:goal(wait_seconds)/ feedback(已等待、剩余秒数)/ result(实际等待、是否完成)
// 每秒发一次 feedback,支持中途取消;与 count_until_server 相比,
// 这里的取消/接受回调直接用 lambda 内联注册,不再单独写成员函数
// 运行:ros2 run cpp_action_tutorial timer_delay_server
// ============================================================

#include <chrono>
#include <memory>
#include <thread>

#include "count_until_interfaces/action/timer_delay.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using TimerDelay = count_until_interfaces::action::TimerDelay;
using GoalHandleTimerDelay = rclcpp_action::ServerGoalHandle<TimerDelay>;

class TimerDelayServer : public rclcpp::Node
{
public:
  TimerDelayServer() : Node("timer_delay_server")
  {
    // 三个回调依次是:goal 审核(成员函数 + std::bind)、取消审核(lambda,无条件同意)、
    // goal 被接受(lambda,开新线程执行避免阻塞事件循环)
    // 简短逻辑用 lambda 内联更紧凑,复杂逻辑仍建议写成员函数
    server_ = rclcpp_action::create_server<TimerDelay>(
      this,
      "timer_delay",
      std::bind(&TimerDelayServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      [](const std::shared_ptr<GoalHandleTimerDelay>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandleTimerDelay> goal_handle) {
        std::thread{std::bind(&TimerDelayServer::execute, this, goal_handle)}.detach();
      });
    RCLCPP_INFO(get_logger(), "Action Server ready on /timer_delay");
  }

private:
  // goal 审核:等待时长必须在 (0, 60] 秒范围内,否则拒绝
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const TimerDelay::Goal> goal)
  {
    if (goal->wait_seconds <= 0.0f) {
      RCLCPP_WARN(get_logger(), "Rejecting goal: wait_seconds must be positive");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->wait_seconds > 60.0f) {
      RCLCPP_WARN(get_logger(), "Rejecting goal: wait_seconds must not exceed 60");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // 任务执行(运行在独立线程):每秒醒来一次,发 feedback 并检查退出条件
  void execute(const std::shared_ptr<GoalHandleTimerDelay> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<TimerDelay::Feedback>();
    auto result = std::make_shared<TimerDelay::Result>();

    // rclcpp::Rate(1.0) 表示 1 Hz:rate.sleep() 会精确补足到每次循环恰好 1 秒
    rclcpp::Rate rate(1.0);
    float elapsed = 0.0f;
    while (elapsed < goal->wait_seconds) {
      // rclcpp::ok() 为 false 说明整个 ROS 2 正在关闭(如 Ctrl+C),用 abort 中止 goal
      if (!rclcpp::ok()) {
        result->elapsed_seconds = elapsed;
        result->completed = false;
        goal_handle->abort(result);
        return;
      }
      // 客户端请求了取消:返回部分结果并把 goal 标记为 canceled
      if (goal_handle->is_canceling()) {
        result->elapsed_seconds = elapsed;
        result->completed = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(get_logger(), "Goal canceled after %.1f s", elapsed);
        return;
      }
      // 每秒推送一次进度:已等待多久、还剩多久
      feedback->elapsed_seconds = elapsed;
      feedback->remaining_seconds = goal->wait_seconds - elapsed;
      goal_handle->publish_feedback(feedback);
      rate.sleep();
      elapsed += 1.0f;
    }

    // 等待完成:返回最终 result 并标记成功
    result->elapsed_seconds = goal->wait_seconds;
    result->completed = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Goal succeeded after %.1f s", goal->wait_seconds);
  }

  rclcpp_action::Server<TimerDelay>::SharedPtr server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TimerDelayServer>());
  rclcpp::shutdown();
  return 0;
}
