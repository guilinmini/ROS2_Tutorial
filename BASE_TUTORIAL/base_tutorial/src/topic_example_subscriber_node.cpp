#include <rclcpp/rclcpp.hpp>
#include <base_tutorial_msg/msg/students.hpp>

class TopicExampleSubscriberNode : public rclcpp::Node
{
public:
  TopicExampleSubscriberNode():Node("topic_example_subscriber_node"){
    subscription_ = this->create_subscription<base_tutorial_msg::msg::Students>(
      "/example/students", 10,
      [this](base_tutorial_msg::msg::Students::UniquePtr msg) {
        RCLCPP_INFO(this->get_logger(), "订阅的学生消息：name=%s,age=%d,height=%.2f,weight=%.2f", msg->name.c_str(),msg->age, msg->height, msg->weight);
      });
  }
private:
  rclcpp::Subscription<base_tutorial_msg::msg::Students>::SharedPtr subscription_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TopicExampleSubscriberNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

