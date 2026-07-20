// ============================================================
// Action 通信示例:TimerDelay 客户端(演示中途取消 goal)
// 演示:请求服务端等待 15 秒,通过 feedback 监控进度;
// 当已等待时间达到 10 秒时主动发送取消请求,最终 result 状态应为 CANCELED
// 运行:先启动 timer_delay_server,再 ros2 run cpp_action_tutorial timer_delay_client
// ============================================================

#include <chrono>
#include <memory>

#include "count_until_interfaces/action/timer_delay.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using TimerDelay = count_until_interfaces::action::TimerDelay;
using GoalHandleTimerDelay = rclcpp_action::ClientGoalHandle<TimerDelay>;
using namespace std::chrono_literals;

class TimerDelayClient : public rclcpp::Node
{
public:
  TimerDelayClient() : Node("timer_delay_client")
  {
    client_ = rclcpp_action::create_client<TimerDelay>(this, "timer_delay");
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
    // goal 只有一个字段:请求服务端等待 15 秒
    TimerDelay::Goal goal;
    goal.wait_seconds = 15.0f;

    rclcpp_action::Client<TimerDelay>::SendGoalOptions options;
    // feedback 回调:[this, goal] 额外捕获 goal 以便计算百分比进度
    // 回调的第一个参数是客户端侧 GoalHandle,拿着它才能对这个 goal 发起取消
    options.feedback_callback = [this, goal](
      GoalHandleTimerDelay::SharedPtr goal_handle,
      const std::shared_ptr<const TimerDelay::Feedback> feedback) {
        const float percent = 100.0f * feedback->elapsed_seconds / goal.wait_seconds;
        RCLCPP_INFO(
          get_logger(), "Progress: %.0f%% (elapsed %.1f s, remaining %.1f s)",
          percent, feedback->elapsed_seconds, feedback->remaining_seconds);
        // 演示取消:等满 10 秒就发取消请求,用 cancel_sent_ 标志保证只发一次
        if (feedback->elapsed_seconds >= 10.0f && !cancel_sent_) {
          cancel_sent_ = true;
          RCLCPP_INFO(get_logger(), "Elapsed >= 10 s, sending cancel request");
          client_->async_cancel_goal(goal_handle);
        }
      };
    // result 回调:WrappedResult 包含状态码(code)和结果数据(result)两部分
    options.result_callback = [this](const GoalHandleTimerDelay::WrappedResult & result) {
        // 把枚举状态码翻译成可读字符串:成功 / 被取消 / 被服务端中止
        const char * status = "UNKNOWN";
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED: status = "SUCCEEDED"; break;
          case rclcpp_action::ResultCode::CANCELED: status = "CANCELED"; break;
          case rclcpp_action::ResultCode::ABORTED: status = "ABORTED"; break;
          default: break;
        }
        if (result.result) {
          RCLCPP_INFO(
            get_logger(), "Result: status=%s, elapsed=%.1f s, completed=%s",
            status, result.result->elapsed_seconds,
            result.result->completed ? "true" : "false");
        } else {
          RCLCPP_ERROR(get_logger(), "Goal failed without result (status=%s)", status);
        }
        rclcpp::shutdown();
      };

    client_->async_send_goal(goal, options);
  }

private:
  rclcpp_action::Client<TimerDelay>::SharedPtr client_;
  // 是否已发送过取消请求的标志(C++11 起成员变量可以直接在声明处给默认值)
  bool cancel_sent_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TimerDelayClient>();
  node->sendGoal();
  // spin 驱动 feedback/result 回调;result 回调里的 shutdown 会结束 spin
  rclcpp::spin(node);
  return 0;
}
