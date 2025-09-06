#include <base_tutorial_msg/action/progress.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class DemoActionClientNode : public rclcpp::Node {
public:
  DemoActionClientNode() : Node("demo_action_client_node") {
    action_client_ =
        rclcpp_action::create_client<base_tutorial_msg::action::Progress>(
            this, "get_sum");
  }

private:
  rclcpp_action::Client<base_tutorial_msg::action::Progress>::SharedPtr
      action_client_;

  void send_goal(int32_t target) {
    if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "无法连接到动作服务器");
      return;
    }

    auto goal_msg = base_tutorial_msg::action::Progress::Goal();
    goal_msg.num = target;

    auto send_goal_options = rclcpp_action::Client<
        base_tutorial_msg::action::Progress>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        [this](rclcpp_action::ClientGoalHandle<
               base_tutorial_msg::action::Progress>::SharedPtr goal_handle) {
          if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "目标被拒绝");
          } else {
            RCLCPP_INFO(this->get_logger(), "目标已接受");
          }
        };
    send_goal_options.feedback_callback =
        [this](rclcpp_action::ClientGoalHandle<
                   base_tutorial_msg::action::Progress>::SharedPtr,
               const std::shared_ptr<
                   const base_tutorial_msg::action::Progress::Feedback>
                   feedback) {
          RCLCPP_INFO(this->get_logger(), "收到反馈: 进度=%.2f",
                      feedback->progress);
        };
    send_goal_options.result_callback =
        [this](const rclcpp_action::ClientGoalHandle<
               base_tutorial_msg::action::Progress>::WrappedResult &result) {
          switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "目标成功完成，结果: sum=%ld",
                        result.result->sum);
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "目标被中止");
            return;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "目标被取消");
            return;
          default:
            RCLCPP_ERROR(this->get_logger(), "未知的结果码");
            return;
          }
        };

    action_client_->async_send_goal(goal_msg, send_goal_options);
  }
};
