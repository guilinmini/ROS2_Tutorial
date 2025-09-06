#include <base_tutorial_msg/action/progress.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class DemoActionServiceNode : public rclcpp::Node {
public:
  DemoActionServiceNode() : Node("demo_action_service_node") {
    action_server_ =
        rclcpp_action::create_server<base_tutorial_msg::action::Progress>(
            this, "get_sum",
            std::bind(&DemoActionServiceNode::handle_goal, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&DemoActionServiceNode::handle_cancel, this,
                      std::placeholders::_1),
            std::bind(&DemoActionServiceNode::handle_accepted, this,
                      std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "动作服务节点已启动，等待目标...");
  }

private:
  rclcpp_action::Server<base_tutorial_msg::action::Progress>::SharedPtr
      action_server_;

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const base_tutorial_msg::action::Progress::Goal> goal) {
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "收到新目标请求: target=%d", goal->num);
    if (goal->num < 1) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<
          rclcpp_action::ServerGoalHandle<base_tutorial_msg::action::Progress>>
          goal_handle) {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "收到取消请求");
    // 接受所有取消请求
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  void handle_accepted(
      const std::shared_ptr<
          rclcpp_action::ServerGoalHandle<base_tutorial_msg::action::Progress>>
          goal_handle) {
    // 在新线程中执行目标
    std::thread{
        std::bind(&DemoActionServiceNode::execute, this, std::placeholders::_1),
        goal_handle}
        .detach();
  }
  void
  execute(const std::shared_ptr<
          rclcpp_action::ServerGoalHandle<base_tutorial_msg::action::Progress>>
              goal_handle) {
    RCLCPP_INFO(this->get_logger(), "开始执行目标");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback =
        std::make_shared<base_tutorial_msg::action::Progress::Feedback>();
    auto result =
        std::make_shared<base_tutorial_msg::action::Progress::Result>();
    int64_t sum = 0;
    for (int i = 1; (i <= goal->num) && rclcpp::ok(); ++i) {
      // 检查是否有取消请求
      if (goal_handle->is_canceling()) {
        result->sum = sum;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "任务取消");
        return;
      }
      // 发布反馈
      feedback->progress = i;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "连续反馈中，进度：%.2f",
                  feedback->progress);
      loop_rate.sleep();
    }
    // 目标完成
    if (rclcpp::ok()) {
      result->sum = sum;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "目标已完成");
    } else {
      RCLCPP_INFO(this->get_logger(), "目标未完成，节点正在关闭");
    }
  };
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DemoActionServiceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
