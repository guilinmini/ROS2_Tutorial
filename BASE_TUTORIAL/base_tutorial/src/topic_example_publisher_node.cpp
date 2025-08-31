#include <rclcpp/rclcpp.hpp>
#include <base_tutorial_msg/msg/students.hpp>

class TopicExamplePublisherNode : public rclcpp::Node
{
public:
  TopicExamplePublisherNode():Node("topic_example_publisher_node")
  {
    publisher_ = this->create_publisher<base_tutorial_msg::msg::Students>("/example/students", 10);
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&TopicExamplePublisherNode::timer_callback, this));
  }
private:
  void timer_callback()
  {
    auto message = base_tutorial_msg::msg::Students();
    message.name = "Guilin";
    message.age = 24;
    message.height = 176.5;
    message.weight = 60.0;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s', %d, %.1f, %.1f", message.name.c_str(), message.age, message.height, message.weight);
    publisher_->publish(message);
  }
  rclcpp::Publisher<base_tutorial_msg::msg::Students>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TopicExamplePublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

