#include "base_tutorial_msg/srv/add_ints.hpp"
#include <rclcpp/rclcpp.hpp>

class DemoServiceNode : public rclcpp::Node {
public:
  DemoServiceNode() : Node("demo_service_node") {
    service_ = this->create_service<base_tutorial_msg::srv::AddInts>(
        "/example/add_ints",
        std::bind(&DemoServiceNode::demo_service_callback, this,
                  std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "服务节点已启动，等待请求...");
  }

private:
  void demo_service_callback(
      const std::shared_ptr<base_tutorial_msg::srv::AddInts::Request> request,
      std::shared_ptr<base_tutorial_msg::srv::AddInts::Response> response) {
    RCLCPP_INFO(this->get_logger(), "收到请求: num1=%d,num2=%d", request->num1,
                request->num2);
    response->sum = request->num1 + request->num2;
    RCLCPP_INFO(this->get_logger(), "响应结果: sum=%d", response->sum);
  }
  rclcpp::Service<base_tutorial_msg::srv::AddInts>::SharedPtr service_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DemoServiceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
