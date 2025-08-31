#include <base_tutorial_msg/srv/add_ints.hpp>
#include <rclcpp/rclcpp.hpp>

class DemoClientNode : public rclcpp::Node {
public:
  DemoClientNode() : Node("demo_client_node") {
    client_ = this->create_client<base_tutorial_msg::srv::AddInts>(
        "/example/add_ints");
  }

  bool connect_service() {
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Interrupted while waiting for the service. Exiting.");
        return false;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting ...");
    }
    return true;
  }

  rclcpp::Client<base_tutorial_msg::srv::AddInts>::FutureAndRequestId
  send_request(int64_t a, int64_t b) {
    auto request = std::make_shared<base_tutorial_msg::srv::AddInts::Request>();
    request->num1 = a;
    request->num2 = b;
    return client_->async_send_request(request);
  }

private:
  rclcpp::Client<base_tutorial_msg::srv::AddInts>::SharedPtr client_;
};

int main(int argc, char *argv[]) {
  if (argc != 3) {
    std::cerr << "Usage: demo_client_node <num1> <num2>" << std::endl;
    return 1;
  }
  rclcpp::init(argc, argv);
  auto client_node = std::make_shared<DemoClientNode>();
  if (!client_node->connect_service()) {
    RCLCPP_ERROR(client_node->get_logger(), "Failed to connect to service.");
    return 1;
  }
  auto response =
      client_node->send_request(std::stoll(argv[1]), std::stoll(argv[2]));

  if (rclcpp::spin_until_future_complete(client_node, response) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(client_node->get_logger(), "Result: %d",
                response.get().get()->sum);
  } else {
    RCLCPP_ERROR(client_node->get_logger(), "Failed to call service add_ints");
  }
  rclcpp::shutdown();
  return 0;
}